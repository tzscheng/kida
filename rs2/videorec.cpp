#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

static volatile std::sig_atomic_t g_stop = 0;

static void signal_handler(int)
{
    g_stop = 1;
}

static std::string make_timestamp_string()
{
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf;
#ifdef _WIN32
    localtime_s(&tm_buf, &t);
#else
    localtime_r(&t, &tm_buf);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");
    return oss.str();
}

static std::string make_output_filename()
{
    std::ostringstream oss;
    oss << "record_" << make_timestamp_string() << ".mp4";
    return oss.str();
}

static bool open_writer_h264(cv::VideoWriter& writer, const std::string& filename, int fps, int width, int height)
{
    writer.release();

    const std::vector<int> fourccs = {
        cv::VideoWriter::fourcc('a', 'v', 'c', '1'),
        cv::VideoWriter::fourcc('H', '2', '6', '4'),
        cv::VideoWriter::fourcc('X', '2', '6', '4'),
        cv::VideoWriter::fourcc('m', 'p', '4', 'v')
    };

    for (int fourcc : fourccs) {
        if (writer.open(filename, fourcc, fps, cv::Size(width, height), true)) {
            return true;
        }
    }
    return false;
}

static void print_usage(const char* prog)
{
    std::cout << "usage: " << prog << " [-d] [-s seconds]\n";
    std::cout << "  -d            enable display\n";
    std::cout << "  -s seconds    segment duration in seconds (default: 3600)\n";
}

int main(int argc, char** argv)
{
    bool display = false;
    int segment_seconds = 3600;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "-d") {
            display = true;
        }
        else if (arg == "-s") {
            if (i + 1 >= argc) {
                std::cerr << "error: -s requires an integer argument\n";
                print_usage(argv[0]);
                return 1;
            }
            segment_seconds = std::atoi(argv[++i]);
            if (segment_seconds <= 0) {
                std::cerr << "error: segment seconds must be > 0\n";
                return 1;
            }
        }
        else if (arg == "-h" || arg == "--help") {
            print_usage(argv[0]);
            return 0;
        }
        else {
            std::cerr << "unknown argument: " << arg << "\n";
            print_usage(argv[0]);
            return 1;
        }
    }

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    const int width = 640;
    const int height = 480;
    const int fps = 30;
    const auto segment_duration = std::chrono::seconds(segment_seconds);

    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);

    cv::VideoWriter writer;
    std::string current_filename;
    auto segment_start_time = std::chrono::steady_clock::now();
    bool pipe_started = false;

    try {
        pipe.start(cfg);
        pipe_started = true;

        current_filename = make_output_filename();
        if (!open_writer_h264(writer, current_filename, fps, width, height)) {
            std::cerr << "failed to open H.264 writer for file: " << current_filename << std::endl;
            std::cerr << "check whether your OpenCV build/backend supports H.264 encoding." << std::endl;
            pipe.stop();
            return 1;
        }

        std::cout << "recording started\n";
        std::cout << "current file: " << current_filename << "\n";
        std::cout << "display: " << (display ? "on" : "off") << "\n";
        std::cout << "segment duration: " << segment_seconds << " sec\n";
        std::cout << "press Ctrl-C to stop";
        if (display) std::cout << ", or press q / ESC in the display window";
        std::cout << std::endl;

        while (!g_stop) {
            rs2::frameset frames;
            try {
                frames = pipe.wait_for_frames(1000);
            }
            catch (const rs2::error& e) {
                if (g_stop) break;
                std::cerr << "wait_for_frames failed: " << e.what() << std::endl;
                continue;
            }

            rs2::video_frame color_frame = frames.get_color_frame();
            if (!color_frame) continue;

            cv::Mat color(cv::Size(width, height), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

            writer.write(color);

            auto now = std::chrono::steady_clock::now();
            if (now - segment_start_time >= segment_duration) {
                writer.release();

                current_filename = make_output_filename();
                if (!open_writer_h264(writer, current_filename, fps, width, height)) {
                    std::cerr << "failed to open next H.264 writer for file: " << current_filename << std::endl;
                    break;
                }

                segment_start_time = now;
                std::cout << "switched to new file: " << current_filename << std::endl;
            }

            if (display) {
                cv::imshow("RealSense RGB", color);
                int key = cv::waitKey(1) & 0xFF;
                if (key == 'q' || key == 27) {
                    g_stop = 1;
                    break;
                }
            }
        }

        std::cout << "stopping..." << std::endl;

        if (writer.isOpened()) writer.release();
        if (pipe_started) pipe.stop();
        if (display) cv::destroyAllWindows();

        std::cout << "final file saved: " << current_filename << std::endl;
    }
    catch (const rs2::error& e) {
        std::cerr << "realsense error: " << e.what() << std::endl;
        if (writer.isOpened()) writer.release();
        if (pipe_started) {
            try { pipe.stop(); } catch (...) {}
        }
        if (display) cv::destroyAllWindows();
        return 1;
    }
    catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << std::endl;
        if (writer.isOpened()) writer.release();
        if (pipe_started) {
            try { pipe.stop(); } catch (...) {}
        }
        if (display) cv::destroyAllWindows();
        return 1;
    }

    return 0;
}
