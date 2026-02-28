#include <opencv2/opencv.hpp>
#include <iostream>
#include <algorithm>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <cstring>

// ─── Arduino Serial ───────────────────────────────────────────────────────────
int openSerial(const char* port, int baud) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        std::cerr << "Error: Cannot open serial port " << port << "\n";
        return -1;
    }
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

void sendToArduino(int fd, float realX, float realY) {
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "T %.2f %.2f\n", realX, realY);
    write(fd, cmd, strlen(cmd));
    std::cout << "Sent to Arduino: " << cmd;
}

// ─── Main ─────────────────────────────────────────────────────────────────────
int main() {
    // ── Camera ──
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Error: Cannot open camera\n";
        return -1;
    }

    // ── Serial ──
    int serialFd = openSerial("/dev/ttyACM0", B9600);
    if (serialFd == -1) return -1;
    sleep(2); // wait for Arduino to reset after serial connect

    // ── Perspective transform ──
    // 4 pixel coords        →   4 real-world coords (mm)
    // (547,176) → (0,   0)
    // (449, 96) → (0,   150)
    // (357,268) → (150, 0)
    // (293,126) → (150, 150)
    std::vector<cv::Point2f> pixelPts = {
        {547, 176},
        {449,  96},
        {357, 268},
        {293, 126}
    };
    std::vector<cv::Point2f> realPts = {
        {  0,   0},
        {  0, 150},
        {150,   0},
        {150, 150}
    };
    cv::Mat H = cv::getPerspectiveTransform(pixelPts, realPts);

    // ── Detection constants ──
    const int   A_THRESH  = 169;
    const int   MIN_AREA  = 3000;
    const int   ERODE_IT  = 3;
    const int   DILATE_IT = 4;

    cv::namedWindow("Benchy Arm Control");
    cv::namedWindow("Mask");

    cv::Mat frame, lab, mask, result, aChan;

    // Last detected center in pixel and real-world space
    cv::Point2f lastPixelCenter(-1, -1);
    cv::Point2f lastRealWorld(-1, -1);
    bool        benchyFound = false;

    std::cout << "Press SPACE to send current benchy position to arm.\n";
    std::cout << "Press ESC to quit.\n";

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        result = frame.clone();

        // ── LAB red detection ──
        cv::cvtColor(frame, lab, cv::COLOR_BGR2Lab);
        cv::extractChannel(lab, aChan, 1);
        cv::threshold(aChan, mask, A_THRESH, 255, cv::THRESH_BINARY);
        cv::erode (mask, mask, cv::Mat(), cv::Point(-1,-1), ERODE_IT);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), DILATE_IT);

        // ── Find biggest contour ──
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        benchyFound = false;

        if (!contours.empty()) {
            auto biggest = std::max_element(contours.begin(), contours.end(),
                [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                    return cv::contourArea(a) < cv::contourArea(b);
                });

            double area = cv::contourArea(*biggest);

            if (area >= MIN_AREA) {
                benchyFound = true;
                int idx = std::distance(contours.begin(), biggest);

                // Bounding box center in pixels
                cv::Rect bbox = cv::boundingRect(*biggest);
                lastPixelCenter = cv::Point2f(
                    bbox.x + bbox.width  / 2.0f,
                    bbox.y + bbox.height / 2.0f
                );

                // ── Perspective transform → real-world mm ──
                std::vector<cv::Point2f> src = { lastPixelCenter };
                std::vector<cv::Point2f> dst;
                cv::perspectiveTransform(src, dst, H);
                lastRealWorld = dst[0];

                // ── Draw ──
                cv::drawContours(result, contours, idx, cv::Scalar(0, 255, 0), 2);
                cv::rectangle(result, bbox, cv::Scalar(0, 0, 255), 2);
                cv::circle(result, lastPixelCenter, 5, cv::Scalar(0, 255, 255), -1);

                // Labels
                std::string posLabel = "Real: (" +
                    std::to_string((int)lastRealWorld.x) + ", " +
                    std::to_string((int)lastRealWorld.y) + ") mm";
                cv::putText(result, "BENCHY", bbox.tl() + cv::Point(0, -22),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
                cv::putText(result, posLabel, bbox.tl() + cv::Point(0, -5),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
            }
        }

        // ── HUD ──
        std::string status = benchyFound
            ? "Benchy detected — press SPACE to move arm"
            : "No benchy detected";
        cv::Scalar statusColor = benchyFound
            ? cv::Scalar(0, 255, 0)
            : cv::Scalar(0, 0, 255);
        cv::putText(result, status, cv::Point(10, 25),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, statusColor, 2);
        cv::putText(result, "ESC = quit", cv::Point(10, 50),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);

        cv::imshow("Benchy Arm Control", result);
        cv::imshow("Mask", mask);

        int key = cv::waitKey(1);

        if (key == 27) break; // ESC

        if (key == 32) { // SPACE
            if (benchyFound) {
                std::cout << "Sending position: X=" << lastRealWorld.x
                          << " Y=" << lastRealWorld.y << " mm\n";
                sendToArduino(serialFd, lastRealWorld.x, lastRealWorld.y);
            } else {
                std::cout << "No benchy detected, nothing sent.\n";
            }
        }
    }

    close(serialFd);
    cap.release();
    cv::destroyAllWindows();
    return 0;
}
