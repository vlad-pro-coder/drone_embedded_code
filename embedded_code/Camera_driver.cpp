#include "./All_drivers_Header.hpp"

CameraDriver::CameraDriver(int deviceIndex = 0, int w = 416, int h = 416)
    : width(w), height(h)
{
    cap.open(deviceIndex, cv::CAP_V4L2);
    if (!cap.isOpened())
    {
        std::cerr << "ERROR: Cannot open camera\n";
        exit(1);
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
}

cv::Mat CameraDriver::captureFrame()
{
    cv::Mat frame, rgb, resized;
    cap >> frame;

    if (frame.empty())
    {
        std::cerr << "ERROR: Empty frame\n";
        return {};
    }

    cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);

    cv::resize(rgb, resized, cv::Size(width, height));

    resized.convertTo(resized, CV_32F, 1.0 / 255.0);
    return resized;
}

CameraDriver::~CameraDriver()
{
    cap.release();
}

/*int main() {
    CameraDriver camera;

    while (true) {
        cv::Mat input = camera.captureFrame();

        if (!input.empty()) {
            cv::Mat display;
            input.convertTo(display, CV_8U, 255.0);
            cv::imshow("YOLO Input", display);
        }

        if (cv::waitKey(1) == 27) break;
    }

    return 0;
}*/
