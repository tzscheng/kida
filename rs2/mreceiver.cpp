#include <iostream>
#include <thread>
#include <atomic>
#include <vector>
#include <mutex>
#include <chrono>
#include <cstring>
#include <string>
#include <sstream>
#include <cmath>
#include <memory>
#include <algorithm>
#include <cctype>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <zstd.h>
#include <zmq.h>

#define WIDTH   640
#define HEIGHT  480

// tact 가상 LiDAR 프레임 해상도. depth처럼 float32 meters 이미지지만 wire가
// RAW(무압축)라는 점이 다르고 (zstd lidar는 2026-06-06 폐지 — tact lidar_frames 참조),
// 카메라보다 작고 고정 해상도이며, no-return 픽셀은 -1로 들어온다.
#define LIDAR_WIDTH   320
#define LIDAR_HEIGHT  240

enum
{
    IMG_RGB   = 1,
    IMG_DEPTH = 2,
    IMG_LIDAR = 3    // tact virtual LiDAR: RAW float32 range map, -1 = no return
};

struct StreamState
{
    std::string name;          // logical name / ZMQ endpoint (e.g., "topcam")
    int type = IMG_RGB;        // per-stream decode: IMG_RGB (JPEG), IMG_DEPTH (zstd), IMG_LIDAR (raw f32)
    int width = WIDTH;         // expected frame dims (640x480 cameras; 120x90 lidar)
    int height = HEIGHT;
    void* sock = nullptr;
    cv::Mat latest_image;      // RGB: CV_8UC3 BGR; depth/lidar: CV_32FC1 meters
    std::mutex latest_mutex;
};

static const char* type_name(int t)
{
    return t == IMG_RGB ? "RGB" : (t == IMG_DEPTH ? "DEPTH" : "LIDAR");
}

static std::atomic<bool> running(true);
static std::vector<std::unique_ptr<StreamState>> g_streams;

void process_rgb(StreamState& stream, const uint8_t* data, size_t size)
{
    std::vector<uint8_t> buf(data, data + size);
    cv::Mat img = cv::imdecode(buf, cv::IMREAD_COLOR);
    if(img.empty())
    {
        std::cerr << "[" << stream.name << " " << size << "] RGB decode failed\n";
        return;
    }

    std::lock_guard<std::mutex> lock(stream.latest_mutex);
    stream.latest_image = img;
}

void process_lidar(StreamState& stream, const uint8_t* data, size_t size)
{
    // tact lidar 2d wire is RAW little-endian float32 since 2026-06-06 (the
    // sim's Python-side zstd was removed — see tact lidar_frames): W*H range
    // map in meters, range-along-ray, -1 = no return. No decompression.
    const int W = stream.width, H = stream.height;
    if(size != (size_t)W * H * 4)
    {
        std::cerr << "[" << stream.name << "] unexpected LIDAR size: " << size
                  << " (expected raw float32 " << (W*H*4) << " for " << W << "x" << H
                  << "; zstd lidar frames were retired 2026-06-06)\n";
        return;
    }
    cv::Mat depth_m;   // CV_32FC1, meters
    cv::Mat(H, W, CV_32FC1, (void*)data).copyTo(depth_m);

    std::lock_guard<std::mutex> lock(stream.latest_mutex);
    stream.latest_image = depth_m;
}

void process_depth(StreamState& stream, const uint8_t* data, size_t size)
{
    // Depth-camera path (zstd-compressed wire). The two senders are told apart by
    // the decompressed size against the stream's expected dims (W*H from the
    // name:type hint):
    //   tact sim depth -> float32 meters,         W*H*4
    //   RealSense      -> Z16 uint16 millimeters, W*H*2
    // Decompress into a W*H*4 buffer (fits both) and normalize to a CV_32FC1 map in
    // meters so the display path is format-agnostic.
    const int W = stream.width, H = stream.height;
    const size_t cap = (size_t)W * H * 4;
    std::vector<uint8_t> raw(cap);

    size_t ret = ZSTD_decompress(raw.data(), cap, data, size);
    if(ZSTD_isError(ret))
    {
        std::cerr << "[" << stream.name << "] ZSTD_decompress failed: " << ZSTD_getErrorName(ret) << std::endl;
        return;
    }

    cv::Mat depth_m;   // CV_32FC1, meters
    if(ret == (size_t)W * H * 4)
    {
        // float32 meters (tact depth) — already metric, take ownership of a copy.
        cv::Mat(H, W, CV_32FC1, raw.data()).copyTo(depth_m);
    }
    else if(ret == (size_t)W * H * 2)
    {
        // RealSense Z16 millimeters -> meters.
        cv::Mat z16(H, W, CV_16UC1, raw.data());
        z16.convertTo(depth_m, CV_32FC1, 1.0 / 1000.0);
    }
    else
    {
        std::cerr << "[" << stream.name << "] unexpected " << type_name(stream.type)
                  << " size: " << ret << " (expected " << (W*H*4) << " float32 or "
                  << (W*H*2) << " Z16 for " << W << "x" << H << ")\n";
        return;
    }

    std::lock_guard<std::mutex> lock(stream.latest_mutex);
    stream.latest_image = depth_m;
}

void process_frame(StreamState& stream, const uint8_t* data, size_t size)
{
    if(stream.type == IMG_RGB)        process_rgb(stream, data, size);
    else if(stream.type == IMG_LIDAR) process_lidar(stream, data, size);   // raw float32
    else                              process_depth(stream, data, size);   // IMG_DEPTH (zstd)
}

cv::Mat make_display_tile(const cv::Mat& img, const std::string& label, int type, int tile_w, int tile_h)
{
    cv::Mat vis;

    if(img.empty())
    {
        vis = cv::Mat::zeros(tile_h, tile_w, CV_8UC3);
        cv::putText(vis, "No Signal", cv::Point(40, tile_h / 2),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(100, 100, 255), 2);
    }
    else
    {
        if(type == IMG_RGB)
        {
            vis = img.clone();
        }
        else // IMG_DEPTH or IMG_LIDAR: img is CV_32FC1 in meters
        {
            cv::Mat depth_clamped, depth8;
            const float MAX_VIS_DEPTH_M = 5.0f;  // vis clamp (m), depth + lidar; beyond this saturates to red

            // Lidar marks no-return pixels as -1 (cameras have no negatives). Clamp negatives
            // to 0 for the colormap, then paint those pixels black so a miss reads as "no echo"
            // rather than near range. For depth the mask is empty, so this is a no-op there.
            cv::Mat invalid = img < 0.0f;          // CV_8U mask
            cv::Mat d;
            cv::max(img, 0.0f, d);
            cv::threshold(d, depth_clamped, MAX_VIS_DEPTH_M, MAX_VIS_DEPTH_M, cv::THRESH_TRUNC);
            depth_clamped.convertTo(depth8, CV_8U, 255.0 / MAX_VIS_DEPTH_M);
            cv::applyColorMap(depth8, vis, cv::COLORMAP_JET);
            vis.setTo(cv::Scalar(0, 0, 0), invalid);
        }
        // Resize the content to the tile BEFORE the label overlay, so the fixed-pixel label
        // bar (28 px) and text (scale 0.55) stay a consistent size regardless of source
        // resolution. Drawing the label first then upscaling magnified it — a 120x90 lidar
        // frame upscaled ~5x turned the bar into a third of the view with giant, clipped text.
        cv::resize(vis, vis, cv::Size(tile_w, tile_h));
    }

    cv::rectangle(vis, cv::Rect(0, 0, vis.cols, 28), cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(vis, label, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    return vis;
}

cv::Mat make_empty_tile(int tile_w, int tile_h)
{
    return cv::Mat::zeros(tile_h, tile_w, CV_8UC3);
}

void compute_grid(int count, int& rows, int& cols)
{
    if(count <= 0)
    {
        rows = 1;
        cols = 1;
        return;
    }

    cols = (int)std::ceil(std::sqrt((double)count));
    rows = (int)std::ceil((double)count / cols);
}

void receiver_thread(void* ctx)
{
    const int n = (int)g_streams.size();

    // Create one SUB socket per stream, connect to ipc:///dev/shm/<name>
    // (the name is used verbatim — RGB vs depth interpretation is per-stream, from
    //  the name:type hint on the command line)
    for(int i = 0; i < n; i++)
    {
        auto& s = *g_streams[i];

        s.sock = zmq_socket(ctx, ZMQ_SUB);
        if(!s.sock)
        {
            std::cerr << "zmq_socket failed for " << s.name << ": " << zmq_strerror(zmq_errno()) << std::endl;
            running = false;
            return;
        }

        int conflate = 1;
        if(zmq_setsockopt(s.sock, ZMQ_CONFLATE, &conflate, sizeof(conflate)) != 0)
        {
            std::cerr << "zmq_setsockopt(CONFLATE) failed: " << zmq_strerror(zmq_errno()) << std::endl;
        }

        // Subscribe to everything on this socket (endpoint itself is the filter)
        if(zmq_setsockopt(s.sock, ZMQ_SUBSCRIBE, "", 0) != 0)
        {
            std::cerr << "zmq_setsockopt(SUBSCRIBE) failed: " << zmq_strerror(zmq_errno()) << std::endl;
            running = false;
            return;
        }

        std::string endpoint = "ipc:///dev/shm/" + s.name;
        if(zmq_connect(s.sock, endpoint.c_str()) != 0)
        {
            std::cerr << "zmq_connect(" << endpoint << ") failed: " << zmq_strerror(zmq_errno()) << std::endl;
            running = false;
            return;
        }

        std::cout << "Subscribed to " << endpoint << std::endl;
    }

    std::vector<zmq_pollitem_t> items(n);
    for(int i = 0; i < n; i++)
    {
        items[i].socket = g_streams[i]->sock;
        items[i].fd = 0;
        items[i].events = ZMQ_POLLIN;
        items[i].revents = 0;
    }

    while(running)
    {
        int rc = zmq_poll(items.data(), n, 100);  // 100 ms timeout
        if(rc < 0)
        {
            if(zmq_errno() == EINTR) continue;
            std::cerr << "zmq_poll failed: " << zmq_strerror(zmq_errno()) << std::endl;
            break;
        }
        if(rc == 0) continue;

        for(int i = 0; i < n; i++)
        {
            if(!(items[i].revents & ZMQ_POLLIN)) continue;

            zmq_msg_t msg;
            zmq_msg_init(&msg);

            int nbytes = zmq_msg_recv(&msg, g_streams[i]->sock, ZMQ_DONTWAIT);
            if(nbytes < 0)
            {
                if(zmq_errno() != EAGAIN)
                {
                    std::cerr << "[" << g_streams[i]->name << "] zmq_msg_recv failed: " << zmq_strerror(zmq_errno()) << std::endl;
                }
                zmq_msg_close(&msg);
                continue;
            }

            if(nbytes > 0)
            {
                process_frame(*g_streams[i], (const uint8_t*)zmq_msg_data(&msg), (size_t)nbytes);
            }

            zmq_msg_close(&msg);
        }
    }

    for(int i = 0; i < n; i++)
    {
        if(g_streams[i]->sock)
        {
            zmq_close(g_streams[i]->sock);
            g_streams[i]->sock = nullptr;
        }
    }
}

void print_usage(const char* cmd)
{
    printf("Usage : %s <name[:type]> [name[:type] ...]\n", cmd);
    printf("   name     ZMQ endpoint, used verbatim as ipc:///dev/shm/<name>\n");
    printf("   :type    per-stream decode hint: 'r'/'rgb' (default), 'd'/'depth', or 'l'/'lidar'\n");
    printf("\n");
    printf("RGB streams are JPEG (cv::imdecode). Depth streams are zstd-compressed and\n");
    printf("auto-detected by decompressed size: %dx%d float32 meters (tact sim) or\n", WIDTH, HEIGHT);
    printf("%dx%d Z16 millimeters (RealSense); both are shown as a JET colormap in meters.\n", WIDTH, HEIGHT);
    printf("Lidar streams are %dx%d RAW float32 range maps (tact sim, no compression); no-return\n", LIDAR_WIDTH, LIDAR_HEIGHT);
    printf("pixels (-1) render black, the rest as a JET colormap in meters.\n");
    printf("\n");
    printf("Examples:\n");
    printf("   %s topcam sidecam:r topdepth:d   # two RGB + one depth\n", cmd);
    printf("   %s top:d                         # single depth stream\n", cmd);
    printf("   %s lidar1:l                      # single lidar stream\n", cmd);
}

int main(int argc, char** argv)
{
    struct Spec { std::string name; int type; };
    std::vector<Spec> specs;

    for(int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];

        if(arg == "-h" || arg == "--help")
        {
            print_usage(argv[0]);
            return 0;
        }
        if(arg == "-D" || arg == "-d")
        {
            std::cerr << "-D/-d is no longer a global flag; use a per-stream hint, "
                         "e.g. 'name:d' for depth.\n";
            print_usage(argv[0]);
            return 1;
        }
        if(!arg.empty() && arg[0] == '-')
        {
            std::cerr << "Unknown argument: " << arg << std::endl;
            print_usage(argv[0]);
            return 1;
        }

        // Parse name[:type]. type hint: r/rgb (default) or d/depth.
        std::string name = arg;
        int type = IMG_RGB;
        auto colon = arg.find(':');
        if(colon != std::string::npos)
        {
            name = arg.substr(0, colon);
            std::string hint = arg.substr(colon + 1);
            std::transform(hint.begin(), hint.end(), hint.begin(),
                           [](unsigned char c){ return std::tolower(c); });
            if(hint == "d" || hint == "depth")                    type = IMG_DEPTH;
            else if(hint == "l" || hint == "lidar")               type = IMG_LIDAR;
            else if(hint.empty() || hint == "r" || hint == "rgb") type = IMG_RGB;
            else
            {
                std::cerr << "Unknown type hint '" << hint << "' for '" << name
                          << "' (use ':r' RGB, ':d' depth, or ':l' lidar)\n";
                return 1;
            }
        }
        if(name.empty())
        {
            std::cerr << "Empty stream name in argument '" << arg << "'\n";
            return 1;
        }

        // Dedup by endpoint name (first occurrence wins); preserve listed order.
        bool dup = false;
        for(const auto& sp : specs) if(sp.name == name) { dup = true; break; }
        if(!dup) specs.push_back({name, type});
    }

    if(specs.empty())
    {
        print_usage(argv[0]);
        return 0;
    }

    g_streams.clear();
    g_streams.reserve(specs.size());
    for(const auto& sp : specs)
    {
        auto s = std::make_unique<StreamState>();
        s->name = sp.name;
        s->type = sp.type;
        if(sp.type == IMG_LIDAR) { s->width = LIDAR_WIDTH; s->height = LIDAR_HEIGHT; }
        g_streams.push_back(std::move(s));
    }

    const int g_stream_count = (int)g_streams.size();
    std::cout << "Stream count = " << g_stream_count << std::endl;
    for(const auto& s : g_streams)
        std::cout << "  " << s->name << " [" << type_name(s->type) << "]" << std::endl;

    void* ctx = zmq_ctx_new();
    if(!ctx)
    {
        std::cerr << "zmq_ctx_new failed" << std::endl;
        return 1;
    }

    const int TILE_W = (g_stream_count <= 1) ? 640 : 400;
    const int TILE_H = (g_stream_count <= 1) ? 480 : 300;

    std::thread t_recv(receiver_thread, ctx);

    while(running)
    {
        std::vector<cv::Mat> tiles;
        tiles.reserve(g_stream_count);

        for(int i = 0; i < g_stream_count; i++)
        {
            cv::Mat img;
            {
                std::lock_guard<std::mutex> lock(g_streams[i]->latest_mutex);
                if(!g_streams[i]->latest_image.empty()) img = g_streams[i]->latest_image.clone();
            }

            std::string label = g_streams[i]->name + " [" + type_name(g_streams[i]->type) + "]";
            cv::Mat tile = make_display_tile(img, label, g_streams[i]->type, TILE_W, TILE_H);
            tiles.push_back(tile);
        }

        int grid_rows = 1;
        int grid_cols = 1;
        compute_grid(g_stream_count, grid_rows, grid_cols);

        while((int)tiles.size() < grid_rows * grid_cols)
        {
            tiles.push_back(make_empty_tile(TILE_W, TILE_H));
        }

        std::vector<cv::Mat> row_imgs;
        for(int r = 0; r < grid_rows; r++)
        {
            std::vector<cv::Mat> row;
            for(int c = 0; c < grid_cols; c++)
            {
                int idx = r * grid_cols + c;
                row.push_back(tiles[idx]);
            }

            cv::Mat row_img;
            cv::hconcat(row, row_img);
            row_imgs.push_back(row_img);
        }

        cv::Mat canvas;
        cv::vconcat(row_imgs, canvas);

        cv::imshow("ZMQ Multi Stream Viewer", canvas);

        int key = cv::waitKey(1) & 0xFF;
        if(key == 27)
        {
            running = false;
            break;
        }
    }

    cv::destroyAllWindows();

    running = false;
    t_recv.join();

    zmq_ctx_term(ctx);

    std::cout << "Receiver shutdown cleanly" << std::endl;
    return 0;
}
