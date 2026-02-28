#include <opencv2/opencv.hpp>
#include <iostream>
#include <algorithm>

int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Error: Cannot open camera\n";
        return -1;
    }

    // Trackbar values
    int aThresh = 169;
    int minArea = 3000;
    int erodeIter = 3;
    int dilateIter = 4;

    cv::namedWindow("Red Detection");
    cv::namedWindow("Mask");
    cv::namedWindow("Controls");

    cv::createTrackbar("A Threshold", "Controls", &aThresh,   255);
    cv::createTrackbar("Min Area",    "Controls", &minArea,   20000);
    cv::createTrackbar("Erode",       "Controls", &erodeIter, 10);
    cv::createTrackbar("Dilate",      "Controls", &dilateIter,10);

    cv::Mat frame, lab, mask, result;
    cv::Mat aChan;

    // Mouse callback to sample HSV/LAB values for tuning
    cv::Mat* labPtr = &lab;
    cv::setMouseCallback("Red Detection", [](int event, int x, int y, int, void* data) {
        if (event == cv::EVENT_LBUTTONDOWN) {
            cv::Mat* labImg = (cv::Mat*)data;
            if (labImg->empty()) return;
            cv::Vec3b pixel = labImg->at<cv::Vec3b>(y, x);
            std::cout << "LAB -> L: " << (int)pixel[0]
                      << "  A: " << (int)pixel[1]
                      << "  B: " << (int)pixel[2] << "\n";
        }
    }, labPtr);

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        result = frame.clone();

        // Convert to LAB — separates lightness from color, robust to shadows
        cv::cvtColor(frame, lab, cv::COLOR_BGR2Lab);

        // Extract the A channel (128 = neutral, >128 = red, <128 = green)
        cv::extractChannel(lab, aChan, 1);

        // Threshold the A channel to isolate red
        cv::threshold(aChan, mask, aThresh, 255, cv::THRESH_BINARY);

        // Morphological cleanup
        if (erodeIter  > 0) cv::erode (mask, mask, cv::Mat(), cv::Point(-1,-1), erodeIter);
        if (dilateIter > 0) cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), dilateIter);

        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            // Find the largest contour
            auto biggest = std::max_element(contours.begin(), contours.end(),
                [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                    return cv::contourArea(a) < cv::contourArea(b);
                });

            double area = cv::contourArea(*biggest);

            if (area >= minArea) {
                int idx = std::distance(contours.begin(), biggest);

                // Draw contour outline
                cv::drawContours(result, contours, idx, cv::Scalar(0, 255, 0), 2);

                // Draw bounding box
                cv::Rect bbox = cv::boundingRect(*biggest);
                cv::rectangle(result, bbox, cv::Scalar(0, 0, 255), 2);

                // Label with area for debugging
                std::string label = "BENCHY  area: " + std::to_string((int)area);
                cv::putText(result, label, bbox.tl() + cv::Point(0, -8),
                            cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 0), 2);
            }
        }

        // Show current threshold value on frame
        cv::putText(result, "A thresh: " + std::to_string(aThresh),
                    cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(255, 255, 255), 2);

        cv::imshow("Red Detection", result);
        cv::imshow("Mask", mask);

        // ESC to quit
        if (cv::waitKey(1) == 27) break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}