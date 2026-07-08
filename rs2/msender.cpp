#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <algorithm>
#include <signal.h>
#include <cstring>
#include <cmath>
#include <mutex>
#include <atomic>
#include <memory>

#include <unistd.h>
#include <cstdlib>
#include <cerrno>

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <zstd.h>
#include <turbojpeg.h>
#include <zmq.h>

#define WIDTH         640
#define HEIGHT        480
#define FPS           30

enum
{
    IMG_RGB   = 1,
    IMG_DEPTH = 2
};

struct DisplaySlot
{
    cv::Mat image;
    std::string label;
    std::mutex mtx;
};

struct CameraWorkerContext
{
    int camera_idx = -1;
    std::string serial;
    std::string logical_name;     // 명령행 인자 그대로 (예: "cam1:d"). ipc:///dev/shm/<logical_name> 으로 bind.
    int image_type = IMG_RGB;     // 카메라별 IMG_RGB / IMG_DEPTH. ':type' hint 로 결정.
    void* sock = nullptr;
    int display_index = -1;       // display_slots 인덱스. 명령행 입력 순서를 따름.
};

static std::atomic<bool> g_run(true);
static bool g_enable_display = false;

// 자동 복구용. 첫 프레임 timeout 발생 시 worker 가 g_request_exec 를 set 하고 g_run 을
// 내림 → main 이 정상 종료 절차 후 execv 로 자기 자신 재실행. in-process hardware_reset
// 만으로는 librealsense/libusb 내부 state 가 stale 해져 /dev/videoN 매핑이 어긋나는
// 케이스가 있어 fresh process 가 가장 확실한 복구 경로.
static char** g_argv = nullptr;
static std::atomic<bool> g_request_exec(false);

void print_usage(const char* cmd)
{
    printf("Usage : %s <name[:type]> [name[:type] ...] [options]\n", cmd);
    printf("   name[:type]       per-camera spec; full token is the ZMQ endpoint name.\n");
    printf("                     ':type' is the per-camera mode hint: 'r'/'rgb' (default),\n");
    printf("                     'd'/'depth', or omitted (= rgb). The hint stays in the\n");
    printf("                     endpoint name, so 'cam1:d' binds ipc:///dev/shm/cam1:d.\n");
    printf("   -d                Enable display window (default: OFF)\n");
    printf("   -q <jpeg quality> JPEG quality, RGB streams only (default: 80)\n");
    printf("   -z <zstd level>   ZSTD level, depth streams only (default: 1)\n");
    printf("\n");
    printf("Endpoint tokens are sorted alphabetically and matched against RealSense\n");
    printf("serials sorted ascending. Command-line order is preserved only for display\n");
    printf("tile layout (-d). Duplicate tokens are deduped silently.\n");
    printf("\n");
    printf("Examples:\n");
    printf("   %s cam1:r cam2:d cam0   # cam0 + cam1:r → RGB, cam2:d → depth\n", cmd);
    printf("   %s top side             # both RGB (no hint = rgb)\n", cmd);
    printf("   %s top:d side:d         # both depth\n", cmd);
}

void signal_handler(int)
{
    // async-signal-safe 해야 하므로 std::cout, printf 대신 write() 사용
    if(g_run.exchange(false))
    {
        static const char msg[] = "\nSIGINT received. Shutting down sender...\n";
        ssize_t r = write(STDERR_FILENO, msg, sizeof(msg) - 1);
        (void)r;
    }
    else
    {
        // 두 번째 SIGINT — 깔끔한 종료 포기하고 즉시 나감
        static const char msg[] = "\nSecond SIGINT received. Force exit.\n";
        ssize_t r = write(STDERR_FILENO, msg, sizeof(msg) - 1);
        (void)r;
        _exit(1);
    }
}

cv::Mat make_depth_vis(const cv::Mat& depth16)
{
    cv::Mat depth_clamped;
    cv::Mat depth8;
    cv::Mat depth_color;

    const float MAX_VIS_DEPTH = 5000.0f; // 5m

    cv::threshold(depth16, depth_clamped, MAX_VIS_DEPTH, MAX_VIS_DEPTH, cv::THRESH_TRUNC);
    depth_clamped.setTo(0, depth16 == 0);
    depth_clamped.convertTo(depth8, CV_8U, 255.0 / MAX_VIS_DEPTH);
    cv::applyColorMap(depth8, depth_color, cv::COLORMAP_JET);

    return depth_color;
}

cv::Mat make_tile(const cv::Mat& img, const std::string& label)
{
    cv::Mat vis;

    if(img.empty())
    {
        vis = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
        cv::putText(vis, "No Signal", cv::Point(40, HEIGHT / 2), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(100, 100, 255), 2);
    }
    else if(img.type() == CV_8UC3)
    {
        vis = img.clone();
    }
    else
    {
        vis = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
    }

    cv::rectangle(vis, cv::Rect(0, 0, vis.cols, 28), cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(vis, label, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);

    return vis;
}

cv::Mat make_canvas(std::vector<std::shared_ptr<DisplaySlot>>& slots)
{
    if(slots.empty())
    {
        cv::Mat canvas = cv::Mat::zeros(240, 320, CV_8UC3);
        cv::putText(canvas, "No active stream", cv::Point(20, 120), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
        return canvas;
    }

    const int TILE_W = 320;
    const int TILE_H = 240;

    int n = (int)slots.size();
    int cols = (int)std::ceil(std::sqrt((double)n));
    int rows = (n + cols - 1) / cols;

    cv::Mat canvas = cv::Mat::zeros(rows * TILE_H, cols * TILE_W, CV_8UC3);

    for(int i = 0; i < n; i++)
    {
        cv::Mat img_copy;
        std::string label;

        {
            std::lock_guard<std::mutex> lock(slots[i]->mtx);
            if(!slots[i]->image.empty())
                img_copy = slots[i]->image.clone();
            label = slots[i]->label;
        }

        cv::Mat tile = make_tile(img_copy, label);
        cv::resize(tile, tile, cv::Size(TILE_W, TILE_H));

        int r = i / cols;
        int c = i % cols;
        tile.copyTo(canvas(cv::Rect(c * TILE_W, r * TILE_H, TILE_W, TILE_H)));
    }

    return canvas;
}

bool compress_jpeg_turbojpeg(tjhandle tj, const cv::Mat& color_mat, int jpeg_quality, std::vector<uint8_t>& out_buf)
{
    if(!tj) return false;
    if(color_mat.empty() || color_mat.type() != CV_8UC3) return false;

    unsigned char* jpeg_buf = nullptr;
    unsigned long jpeg_size = 0;

    int ret = tjCompress2(tj, color_mat.data, color_mat.cols, (int)color_mat.step, color_mat.rows, TJPF_BGR, &jpeg_buf, &jpeg_size, TJSAMP_420, jpeg_quality, TJFLAG_FASTDCT);

    if(ret != 0)
    {
        if(jpeg_buf) tjFree(jpeg_buf);
        return false;
    }

    out_buf.assign(jpeg_buf, jpeg_buf + jpeg_size);
    tjFree(jpeg_buf);
    return true;
}

// ZMQ PUB 로 한 프레임 전송. CONFLATE 소켓이므로 큐잉되지 않고 latest 만 유지됨.
void zmq_send_frame(void* sock, const std::vector<uint8_t>& data)
{
    if(!sock) return;

    int rc = zmq_send(sock, data.data(), data.size(), ZMQ_DONTWAIT);
    if(rc < 0)
    {
        int err = zmq_errno();
        if(err == ETERM)
        {
            // context shutdown 요청됨 - worker 루프를 빠져나가도록 신호
            g_run = false;
        }
        else if(err != EAGAIN)
        {
            std::cerr << "zmq_send failed: " << zmq_strerror(err) << std::endl;
        }
    }
}

// 진단용: 카메라별 prefix 를 일관되게 찍기 위한 helper.
static std::string cam_tag(const CameraWorkerContext& ctx)
{
    return "[cam" + std::to_string(ctx.camera_idx + 1) + " " + ctx.logical_name + "]";
}

void camera_worker_func(CameraWorkerContext ctx, int jpeg_quality, int zstd_level, std::vector<std::shared_ptr<DisplaySlot>>* display_slots)
{
    rs2::pipeline pipe;
    rs2::config cfg;
    tjhandle tj = nullptr;

    const bool is_rgb = (ctx.image_type == IMG_RGB);
    const std::string tag = cam_tag(ctx);

    try
    {
        if(is_rgb)
        {
            tj = tjInitCompress();
            if(!tj)
            {
                std::cerr << tag << " tjInitCompress failed" << std::endl;
                return;
            }
        }

        cfg.enable_device(ctx.serial);

        if(is_rgb) cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_BGR8, FPS);
        else       cfg.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16,  FPS);

        // --- 1) pipe.start + 첫 프레임 대기.
        //
        //     librealsense V4L2 backend 가 STREAMON 은 성공시켰는데 isoc payload 가
        //     안 올라오는 cold-start stuck 패턴이 D400 시리즈에서 종종 발생.
        //     이 때 in-process 에서 hardware_reset + pipe 재시작을 시도해도 librealsense
        //     /libusb 의 내부 device 매핑 state 가 stale 해져 /dev/videoN ENOENT 로
        //     실패하는 경우가 잦음. 따라서 첫 프레임 timeout 시 process 자체를 execv 로
        //     재시작 → main() 이 fresh state 로 다시 들어가 정상 startup. 사용자가
        //     수동으로 Ctrl+C → 재실행 하던 워크플로우를 자동화한 형태.
        const int FIRST_FRAME_DEADLINE_MS = 3000;

        auto t_start_pre = std::chrono::steady_clock::now();
        rs2::pipeline_profile profile = pipe.start(cfg);
        auto t_start_post = std::chrono::steady_clock::now();
        auto start_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_start_post - t_start_pre).count();

        const char* mode = is_rgb ? "rgb" : "depth";
        std::cout << tag << " pipe.start ok in " << start_ms << "ms"
                  << " serial=" << ctx.serial
                  << " endpoint=ipc:///dev/shm/" << ctx.logical_name
                  << " (" << mode << ")"
                  << std::endl;

        try
        {
            auto active_dev = profile.get_device();
            std::string active_usb = active_dev.supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR)
                                     ? active_dev.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR) : "(n/a)";
            std::cout << tag << " active usb=" << active_usb;
            for(auto& s : profile.get_streams())
            {
                std::cout << " | stream=" << s.stream_name() << " fmt=" << s.format() << " " << s.fps() << "fps";
            }
            std::cout << std::endl;
            if(active_usb.size() >= 1 && active_usb[0] == '2')
            {
                std::cerr << tag << " WARN: device fell back to USB " << active_usb
                          << ". 640x480x" << FPS << " "
                          << (is_rgb ? "BGR8 RGB" : "Z16 depth")
                          << " 대역폭이 빠듯함. 첫 프레임이 안 들어오면 이게 원인일 가능성 높음." << std::endl;
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << tag << " profile introspection failed: " << e.what() << std::endl;
        }

        // 첫 프레임 폴링. 받은 프레임은 버리고 main 루프에서 새로 받음 (분기 단순화).
        bool first_frame_seen = false;
        auto t_wait = std::chrono::steady_clock::now();
        while(g_run)
        {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t_wait).count();
            if(elapsed >= FIRST_FRAME_DEADLINE_MS) break;

            rs2::frameset f;
            if(pipe.poll_for_frames(&f))
            {
                std::cout << tag << " first frame received after " << elapsed << "ms" << std::endl;
                first_frame_seen = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        if(!first_frame_seen)
        {
            if(!g_run)
            {
                // SIGINT 등으로 셧다운 중 — 정상 종료 경로.
                try { pipe.stop(); } catch(...) {}
                if(tj) tjDestroy(tj);
                return;
            }

            // Cold-start stuck 감지. process re-exec 으로 회복.
            std::cerr << tag << " WARN: " << FIRST_FRAME_DEADLINE_MS
                      << "ms 동안 첫 프레임 없음. process re-exec 으로 자동 복구 시도." << std::endl;

            // 현재 pipe 정리. 펌웨어 측 stuck 도 같이 풀어주기 위해 hardware_reset 한 번
            // 발행 (best-effort, 실패해도 무시). 그 후 USB 재열거가 충분히 진행되도록
            // 4초 sleep — 이래야 exec 된 새 프로세스가 fresh enumeration 으로 device 를
            // 잡을 수 있음.
            try { pipe.stop(); } catch(...) {}

            try
            {
                rs2::context rs_ctx;
                for(auto&& d : rs_ctx.query_devices())
                {
                    std::string s = d.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)
                                    ? d.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) : "";
                    if(s == ctx.serial)
                    {
                        std::cerr << tag << " issuing hardware_reset() before exec..." << std::endl;
                        d.hardware_reset();
                        break;
                    }
                }
            }
            catch(const std::exception& e)
            {
                std::cerr << tag << " hardware_reset before exec threw: " << e.what() << std::endl;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(4000));

            // main 한테 정리 후 execv 하라고 신호. g_run=false 로 main 루프 깨움.
            g_request_exec = true;
            g_run = false;

            if(tj) tjDestroy(tj);
            return;
        }

        // --- 2) 정상 스트리밍 루프 ---
        auto t_started      = std::chrono::steady_clock::now();
        auto t_last_frame   = t_started;
        auto t_last_hb      = t_started;
        bool warned_stalled        = false;
        bool first_display_logged  = false;
        uint64_t rx_count          = 0;   // poll_for_frames 성공 횟수
        uint64_t tx_count          = 0;   // ZMQ 전송 시도 성공 (compress 통과)
        uint64_t tx_bytes          = 0;
        uint64_t poll_misses       = 0;   // poll_for_frames false
        uint64_t null_frame_count  = 0;   // frameset 은 왔는데 color/depth frame 이 null
        uint64_t encode_fail_count = 0;

        while(g_run)
        {
            auto now = std::chrono::steady_clock::now();

            // 첫 프레임 후 stall 감지: 1.5s 이상 프레임이 안 들어오면 한번 경고, 회복하면 재무장
            {
                auto since_last_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - t_last_frame).count();
                if(!warned_stalled && since_last_ms >= 1500)
                {
                    std::cerr << tag << " WARN: frame stall, " << since_last_ms
                              << "ms since last frame (rx=" << rx_count << ")" << std::endl;
                    warned_stalled = true;
                }
                if(warned_stalled && since_last_ms < 500)
                {
                    std::cerr << tag << " stall recovered" << std::endl;
                    warned_stalled = false;
                }
            }

            // 5초 heartbeat — 정상 동작 시에도 흐름 확인용
            if(std::chrono::duration_cast<std::chrono::milliseconds>(now - t_last_hb).count() >= 5000)
            {
                auto since_last_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - t_last_frame).count();
                std::cerr << tag << " hb"
                          << " rx=" << rx_count
                          << " tx=" << tx_count
                          << " bytes=" << tx_bytes
                          << " poll_miss=" << poll_misses
                          << " null_frame=" << null_frame_count
                          << " enc_fail=" << encode_fail_count
                          << " last_frame=" << since_last_ms << "ms"
                          << std::endl;
                t_last_hb = now;
            }

            rs2::frameset frames;
            if(!pipe.poll_for_frames(&frames))
            {
                poll_misses++;
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            rx_count++;
            t_last_frame = now;

            if(is_rgb)
            {
                auto color = frames.get_color_frame();
                if(!color)
                {
                    null_frame_count++;
                    continue;
                }

                cv::Mat color_mat(HEIGHT, WIDTH, CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);

                std::vector<uint8_t> color_buf;
                if(compress_jpeg_turbojpeg(tj, color_mat, jpeg_quality, color_buf))
                {
                    zmq_send_frame(ctx.sock, color_buf);
                    tx_count++;
                    tx_bytes += color_buf.size();
                }
                else
                {
                    encode_fail_count++;
                    std::cerr << tag << " TurboJPEG compress failed" << std::endl;
                }

                if(g_enable_display && ctx.display_index >= 0)
                {
                    std::lock_guard<std::mutex> lock((*display_slots)[ctx.display_index]->mtx);
                    (*display_slots)[ctx.display_index]->image = color_mat.clone();
                    if(!first_display_logged)
                    {
                        std::cout << tag << " first image written to display slot" << std::endl;
                        first_display_logged = true;
                    }
                }
            }
            else
            {
                auto depth = frames.get_depth_frame();
                if(!depth)
                {
                    null_frame_count++;
                    continue;
                }

                cv::Mat depth_mat(HEIGHT, WIDTH, CV_16UC1, (void*)depth.get_data(), cv::Mat::AUTO_STEP);

                size_t depth_raw_size = WIDTH * HEIGHT * 2;
                size_t zstd_bound = ZSTD_compressBound(depth_raw_size);
                std::vector<uint8_t> depth_buf(zstd_bound);

                size_t compressed_size = ZSTD_compress(depth_buf.data(), zstd_bound, depth_mat.data, depth_raw_size, zstd_level);

                if(!ZSTD_isError(compressed_size))
                {
                    depth_buf.resize(compressed_size);
                    zmq_send_frame(ctx.sock, depth_buf);
                    tx_count++;
                    tx_bytes += compressed_size;

                    if(g_enable_display && ctx.display_index >= 0)
                    {
                        cv::Mat vis = make_depth_vis(depth_mat);
                        std::lock_guard<std::mutex> lock((*display_slots)[ctx.display_index]->mtx);
                        (*display_slots)[ctx.display_index]->image = vis;
                        if(!first_display_logged)
                        {
                            std::cout << tag << " first image written to display slot" << std::endl;
                            first_display_logged = true;
                        }
                    }
                }
                else
                {
                    encode_fail_count++;
                    std::cerr << tag << " ZSTD_compress failed: " << ZSTD_getErrorName(compressed_size) << std::endl;
                }
            }
        }

        // 종료 시 final 통계 — 비정상 종료 추적용
        auto t_end = std::chrono::steady_clock::now();
        auto run_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_started).count();
        std::cout << tag << " stopping pipeline..."
                  << " final: rx=" << rx_count
                  << " tx=" << tx_count
                  << " bytes=" << tx_bytes
                  << " poll_miss=" << poll_misses
                  << " null_frame=" << null_frame_count
                  << " enc_fail=" << encode_fail_count
                  << " ran=" << run_ms << "ms"
                  << " first_frame=yes"
                  << std::endl;
        try { pipe.stop(); } catch(const std::exception& e) { std::cerr << tag << " pipe.stop exception: " << e.what() << std::endl; } catch(...) {}
        if(tj) tjDestroy(tj);

        std::cout << tag << " stopped." << std::endl;
    }
    catch(const std::exception& e)
    {
        std::cerr << tag << " camera worker exception: " << e.what() << std::endl;

        try { pipe.stop(); } catch(...) {}
        if(tj) tjDestroy(tj);
    }
}

// ZMQ PUB 소켓 생성 + CONFLATE=1 + ipc bind
void* create_pub_socket(void* zctx, const std::string& logical_name)
{
    void* sock = zmq_socket(zctx, ZMQ_PUB);
    if(!sock)
    {
        std::cerr << "zmq_socket(PUB) failed: " << zmq_strerror(zmq_errno()) << std::endl;
        return nullptr;
    }

    int conflate = 1;
    if(zmq_setsockopt(sock, ZMQ_CONFLATE, &conflate, sizeof(conflate)) != 0)
    {
        std::cerr << "zmq_setsockopt(CONFLATE) failed: " << zmq_strerror(zmq_errno()) << std::endl;
        // non-fatal; 계속 진행
    }

    // LINGER=0: 종료 시 아직 안 보낸 메시지는 즉시 폐기. 실시간 스트리밍에서는
    // 늦은 메시지 가치 없음. 이게 없으면 zmq_ctx_term 이 무한 대기할 수 있음.
    int linger = 0;
    if(zmq_setsockopt(sock, ZMQ_LINGER, &linger, sizeof(linger)) != 0)
    {
        std::cerr << "zmq_setsockopt(LINGER) failed: " << zmq_strerror(zmq_errno()) << std::endl;
    }

    std::string endpoint = "ipc:///dev/shm/" + logical_name;

    if(zmq_bind(sock, endpoint.c_str()) != 0)
    {
        std::cerr << "zmq_bind(" << endpoint << ") failed: " << zmq_strerror(zmq_errno()) << std::endl;
        zmq_close(sock);
        return nullptr;
    }

    std::cout << "bound " << endpoint << std::endl;
    return sock;
}

int main(int argc, char *argv[])
{
    // 자동 복구 시 execv 로 재실행할 때 사용하기 위해 원본 argv 보관.
    g_argv = argv;

    int jpeg_quality = 80;
    int zstd_level = 1;

    // 사전 검사: --help 와 더 이상 지원되지 않는 legacy 전역 -D 플래그.
    // RGB/Depth 는 이제 카메라별 ':type' hint 로 지정 (예: cam1:d).
    for(int i = 1; i < argc; i++)
    {
        std::string s = argv[i];
        if(s == "--help")
        {
            print_usage(argv[0]);
            return 0;
        }
        if(s == "-D")
        {
            std::cerr << "-D is no longer a global flag; use a per-camera ':d' hint, "
                         "e.g. 'cam1:d' for depth.\n";
            print_usage(argv[0]);
            return 1;
        }
    }

    int opt;
    while((opt = getopt(argc, argv, "hdq:z:")) != -1)
    {
        switch(opt)
        {
            case 'h':
                print_usage(argv[0]);
                return 0;
            case 'd':
                g_enable_display = true;
                break;
            case 'q':
                jpeg_quality = atoi(optarg);
                break;
            case 'z':
                zstd_level = atoi(optarg);
                break;
            case '?':
            default:
                print_usage(argv[0]);
                return -1;
        }
    }

    // Positional: 각 토큰 'name[:type]'. 토큰 전체를 ZMQ endpoint 이름으로 사용 (ipc:///dev/shm/<token>).
    // ':type' 뒤 hint 는 RGB/Depth 판정에만 쓰고 endpoint 문자열에서 떼지 않는다.
    // 즉 'cam1:d' → endpoint="cam1:d", type=DEPTH. 'cam0' → endpoint="cam0", type=RGB.
    struct Spec { std::string endpoint; int image_type; int input_idx; };
    std::vector<Spec> specs;

    for(int i = optind; i < argc; i++)
    {
        std::string arg = argv[i];
        if(arg.empty())
        {
            std::cerr << "Empty positional argument\n";
            return -1;
        }

        int type = IMG_RGB;
        auto colon = arg.find(':');
        if(colon != std::string::npos)
        {
            std::string hint = arg.substr(colon + 1);
            std::transform(hint.begin(), hint.end(), hint.begin(),
                           [](unsigned char c){ return std::tolower(c); });
            if(hint == "d" || hint == "depth")                    type = IMG_DEPTH;
            else if(hint.empty() || hint == "r" || hint == "rgb") type = IMG_RGB;
            else
            {
                std::cerr << "Unknown type hint '" << hint << "' in '" << arg
                          << "' (use ':r' RGB, ':d' depth, or omit for RGB)\n";
                return -1;
            }
        }

        // Endpoint(=full token) 기준 dedup. 동일 토큰 두 번이면 같은 IPC 경로에 bind 두 번 → 실패.
        // 따라서 silent dedup. 'cam1:r' 과 'cam1:d' 는 서로 다른 토큰이므로 충돌 아님.
        bool dup = false;
        for(const auto& sp : specs) if(sp.endpoint == arg) { dup = true; break; }
        if(!dup) specs.push_back({arg, type, (int)specs.size()});
    }

    if(specs.empty())
    {
        print_usage(argv[0]);
        return 0;
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // librealsense 자체 진단 메시지를 stderr 로 흘려보내기. WARN 부터 받으면
    // USB2 fallback, isoc 채널 실패, frame drop, format 협상 실패 등이 잡힘.
    // 무프레임 증상 진단의 1차 단서 — 변경 후 동일 증상이 재현되면 이 로그를 먼저 보면 됨.
    rs2::log_to_console(RS2_LOG_SEVERITY_WARN);

    // 디스플레이 backend 점검용. -d 인데 창이 안 뜨거나 빈 창이면 여기 값이 의심.
    {
        const char* disp = std::getenv("DISPLAY");
        const char* wdisp = std::getenv("WAYLAND_DISPLAY");
        const char* xdg = std::getenv("XDG_SESSION_TYPE");
        std::cout << "display env: DISPLAY=" << (disp  ? disp  : "(unset)")
                  << " WAYLAND_DISPLAY=" << (wdisp ? wdisp : "(unset)")
                  << " XDG_SESSION_TYPE=" << (xdg ? xdg : "(unset)")
                  << std::endl;
    }

    // --- Enumerate RealSense devices, sort by serial ascending ---
    rs2::context rs_ctx;
    auto devs = rs_ctx.query_devices();

    std::vector<rs2::device> dev_list;
    for(auto&& dev : devs) dev_list.push_back(dev);

    std::sort(dev_list.begin(), dev_list.end(), [](const rs2::device& a, const rs2::device& b) {
        std::string sa = a.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        std::string sb = b.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        return sa < sb;
    });

    auto safe_info = [](const rs2::device& d, rs2_camera_info info) -> std::string {
        try { if(d.supports(info)) return d.get_info(info); } catch(...) {}
        return "(n/a)";
    };

    // USB type 은 무프레임 진단에 가장 중요한 단서. "USB 2.x" 면 대역폭 부족 가능성 즉시 의심.
    std::cout << "=== Sorted Devices ===" << std::endl;
    for(size_t i = 0; i < dev_list.size(); i++)
    {
        const auto& d = dev_list[i];
        std::cout << "cam" << (i + 1)
                  << " serial=" << safe_info(d, RS2_CAMERA_INFO_SERIAL_NUMBER)
                  << " name="   << safe_info(d, RS2_CAMERA_INFO_NAME)
                  << " fw="     << safe_info(d, RS2_CAMERA_INFO_FIRMWARE_VERSION)
                  << " usb="    << safe_info(d, RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR)
                  << " port="   << safe_info(d, RS2_CAMERA_INFO_PHYSICAL_PORT)
                  << std::endl;
    }

    int cam_count = (int)dev_list.size();

    if(cam_count < (int)specs.size())
    {
        std::cerr << "Requested " << specs.size() << " cameras but only " << cam_count << " RealSense devices found." << std::endl;
        return -1;
    }

    void* zctx = zmq_ctx_new();
    if(!zctx)
    {
        std::cerr << "zmq_ctx_new failed" << std::endl;
        return -1;
    }

    // sorted_specs: endpoint(=full token) 알파벳 오름차순. realsense serial 오름차순과 zip 해서 매핑.
    // 명령행 입력 순서는 display 타일 배치용으로만 보존 (각 spec 의 input_idx).
    std::vector<Spec> sorted_specs = specs;
    std::sort(sorted_specs.begin(), sorted_specs.end(),
              [](const Spec& a, const Spec& b){ return a.endpoint < b.endpoint; });

    std::vector<CameraWorkerContext> workers;
    std::vector<std::shared_ptr<DisplaySlot>> display_slots;

    if(g_enable_display)
    {
        // 입력 순서 기준으로 slot 을 미리 할당 → 워커는 자신의 input_idx 로 직접 접근.
        display_slots.resize(specs.size());
        for(size_t i = 0; i < specs.size(); i++)
            display_slots[i] = std::make_shared<DisplaySlot>();
    }

    for(size_t i = 0; i < sorted_specs.size(); i++)
    {
        const auto& sp = sorted_specs[i];

        CameraWorkerContext w;
        w.camera_idx = (int)i;
        w.serial = dev_list[i].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        w.logical_name = sp.endpoint;
        w.image_type = sp.image_type;

        w.sock = create_pub_socket(zctx, sp.endpoint);
        if(!w.sock) { zmq_ctx_term(zctx); return -1; }

        if(g_enable_display)
        {
            w.display_index = sp.input_idx;
            const char* mode = (sp.image_type == IMG_RGB) ? "rgb" : "depth";
            display_slots[sp.input_idx]->label = sp.endpoint + " (" + mode + ", serial=" + w.serial + ")";
        }

        workers.push_back(w);
    }

    std::cout << "=== Name <-> Serial mapping ===" << std::endl;
    for(const auto& w : workers)
    {
        const char* mode = (w.image_type == IMG_RGB) ? "rgb" : "depth";
        std::cout << w.logical_name << " <- serial " << w.serial << " (" << mode << ")" << std::endl;
    }
    std::cout << "display: " << (g_enable_display ? "ON" : "OFF") << std::endl;

    // --- Launch camera worker threads ---
    std::vector<std::thread> threads;
    for(const auto& w : workers)
    {
        threads.emplace_back(camera_worker_func, w, jpeg_quality, zstd_level, &display_slots);
    }

    if(g_enable_display)
    {
        cv::namedWindow("RealSense Multi Sender", cv::WINDOW_NORMAL);
        cv::resizeWindow("RealSense Multi Sender", 1280, 720);

        while(g_run)
        {
            cv::Mat canvas = make_canvas(display_slots);
            cv::imshow("RealSense Multi Sender", canvas);

            int key = cv::waitKey(1) & 0xFF;
            if(key == 27 || key == 'q')
            {
                g_run = false;
                break;
            }
        }

        cv::destroyAllWindows();
    }
    else
    {
        while(g_run)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    std::cout << "Shutting down: waiting for camera workers..." << std::endl;

    // ZMQ context shutdown 을 먼저 호출하면 worker 가 zmq_send 에서 blocking
    // 중일 경우 ETERM 으로 깨어남. LINGER=0 과 함께 있으면 안전함.
    zmq_ctx_shutdown(zctx);

    for(auto& t : threads)
    {
        if(t.joinable()) t.join();
    }

    std::cout << "Shutting down: closing ZMQ sockets..." << std::endl;
    for(auto& w : workers)
    {
        if(w.sock) { zmq_close(w.sock); w.sock = nullptr; }
    }

    std::cout << "Shutting down: terminating ZMQ context..." << std::endl;
    zmq_ctx_term(zctx);

    std::cout << "Sender clean shutdown." << std::endl;

    // worker 가 cold-start stuck 을 만나 자동 복구를 요청한 경우, 모든 fd/스레드 정리가
    // 끝난 지금 시점에 자기 자신을 재실행. fresh process 는 librealsense/libusb state 를
    // 깨끗하게 다시 잡으므로 다음 enumeration 이 정상으로 가는 게 일반적.
    if(g_request_exec)
    {
        std::cerr << "Auto-recovery: re-execing self for clean state..." << std::endl;
        execv("/proc/self/exe", g_argv);
        // execv 가 돌아왔으면 실패. 일반적으로 발생하지 않음.
        std::cerr << "execv failed: " << std::strerror(errno) << std::endl;
        return 1;
    }

    return 0;
}
