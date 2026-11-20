// Ball tracker on a platform - C++ port of the Python/OpenCV script.
// The tracking/calibration code is the original; only main() was turned into BallTracker methods.

#include "ball_tracker.hpp"

#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

static const double PLATFORM_CM = 30.0;
static const int MIN_SATURATION = 60;
static const int DISPLAY_W = 640;

enum class Step { Colour, Axes };

struct Axes {
    cv::Point2d centre;
    cv::Matx22d toCm;
    double radius = 0;
};

// Global state (shared with the mouse callback, like the Python globals)
static Step step = Step::Colour;
static std::vector<cv::Vec3i> samples;
static std::vector<cv::Point2d> clicks;
static bool haveColour = false;
static cv::Scalar lower, upper;
static bool haveAxes = false;
static Axes axes;
static cv::Mat frame;

// Truncating conversion, matches Python int() / astype(int)
static cv::Point toPt(const cv::Point2d& p) { return cv::Point((int)p.x, (int)p.y); }

static void setColour() {
    // HSV range from the clicked pixels plus a margin
    cv::Vec3i mn = samples[0], mx = samples[0];
    for (const auto& s : samples)
        for (int k = 0; k < 3; k++) {
            mn[k] = std::min(mn[k], s[k]);
            mx[k] = std::max(mx[k], s[k]);
        }
    const int margin[3] = {5, 40, 50};
    const int hiMax[3] = {179, 255, 255};
    int lo[3], hi[3];
    for (int k = 0; k < 3; k++) {
        lo[k] = mn[k] - margin[k];
        hi[k] = mx[k] + margin[k];
    }
    lo[1] = std::max(lo[1], MIN_SATURATION);
    for (int k = 0; k < 3; k++) {
        lo[k] = std::clamp(lo[k], 0, 255);
        hi[k] = std::clamp(hi[k], 0, hiMax[k]);
    }
    lower = cv::Scalar(lo[0], lo[1], lo[2]);
    upper = cv::Scalar(hi[0], hi[1], hi[2]);
    haveColour = true;
    std::printf("LOWER = [%d, %d, %d]   UPPER = [%d, %d, %d]\n",
                lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]);
}

static void onClick(int event, int x, int y, int, void*) {
    // colour step: sample a 7x7 patch
    if (event != cv::EVENT_LBUTTONDOWN || frame.empty()) return;
    if (step == Step::Colour) {
        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        int x0 = std::max(0, x - 3), y0 = std::max(0, y - 3);
        int x1 = std::min(hsv.cols, x + 4), y1 = std::min(hsv.rows, y + 4);
        for (int yy = y0; yy < y1; yy++)
            for (int xx = x0; xx < x1; xx++) {
                const cv::Vec3b& p = hsv.at<cv::Vec3b>(yy, xx);
                samples.emplace_back(p[0], p[1], p[2]);
            }
        if (!samples.empty()) setColour();
    }
}

static Axes makeAxes() {
    const cv::Point2d &top = clicks[0], &bottom = clicks[1];
    Axes a;
    a.centre = (top + bottom) * 0.5;
    cv::Point2d ey = (top - bottom) / PLATFORM_CM;   // pixels per cm along +Y
    cv::Point2d ex(-ey.y, ey.x);                     // ey rotated 90deg -> +X (to the right when +Y is up)
    a.toCm = cv::Matx22d(ex.x, ey.x,
                         ex.y, ey.y).inv();
    a.radius = cv::norm(top - bottom) / 2;
    return a;
}

// Largest blob in the colour range -> ball = (u, v, r) in pixels. Returns false if none.
static bool findBall(const cv::Mat& img, const cv::Mat& area, cv::Vec3f& ball, cv::Mat& mask) {
    cv::Mat hsv;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, lower, upper, mask);
    if (!area.empty()) cv::bitwise_and(mask, area, mask);
    cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (!contours.empty()) {
        auto biggest = std::max_element(contours.begin(), contours.end(),
            [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                return cv::contourArea(a) < cv::contourArea(b);
            });
        cv::Point2f c;
        float r;
        cv::minEnclosingCircle(*biggest, c, r);
        if (r > 8) {
            ball = cv::Vec3f(c.x, c.y, r);
            return true;
        }
    }
    return false;
}

static cv::Size openCamera(cv::VideoCapture& cap, int index) {
    // ask for the largest frame the camera supports so nothing is cropped, zoom fully out
    cap.open(index, cv::CAP_DSHOW);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 10000);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 10000);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
    cap.set(cv::CAP_PROP_ZOOM, 0);
    int w = (int)cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int h = (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    std::printf("Camera full frame: %dx%d\n", w, h);
    if (w <= 0 || h <= 0) return cv::Size(DISPLAY_W, DISPLAY_W * 3 / 4);
    return cv::Size(DISPLAY_W, (int)std::lround((double)DISPLAY_W * h / w));
}

static void text(cv::Mat& img, const std::string& s, const cv::Scalar& colour = cv::Scalar(0, 0, 255)) {
    cv::putText(img, s, {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 3);
    cv::putText(img, s, {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 0.6, colour, 2);
}

// ======================= BallTracker =======================

bool BallTracker::open(int cameraIndex) {
    size_ = openCamera(cap_, cameraIndex);
    cv::namedWindow("Ball");
    cv::setMouseCallback("Ball", onClick);
    return cap_.isOpened();
}

bool BallTracker::update(BallReading& out) {
    cv::Mat img;
    if (!cap_.read(img) || img.empty()) return false;
    out.time = std::chrono::steady_clock::now();
    cv::resize(img, img, size_);
    frame = img.clone();
    cv::Mat mask = cv::Mat::zeros(img.size(), CV_8UC1);
    cv::Vec3f ball;
    bool found = false;
    char buf[128];

    if (step == Step::Colour) {
        text(img, "STEP 1: click the ball, N = next");
        if (haveColour && findBall(frame, cv::Mat(), ball, mask))
            cv::circle(img, cv::Point((int)ball[0], (int)ball[1]), (int)ball[2], cv::Scalar(0, 255, 0), 2);
    } else {
        if (!haveAxes) {
            axes = makeAxes();
            haveAxes = true;
        }
        cv::Point c = toPt(axes.centre);
        cv::Point2d ey = (clicks[0] - clicks[1]) * 0.5;
        cv::Point2d ex(ey.y, -ey.x);
        cv::line(img, toPt(axes.centre - ex), toPt(axes.centre + ex), cv::Scalar(0, 0, 255), 1);
        cv::line(img, toPt(clicks[1]), toPt(clicks[0]), cv::Scalar(0, 200, 0), 1);
        cv::circle(img, c, (int)axes.radius, cv::Scalar(200, 200, 200), 1);

        cv::Mat area = cv::Mat::zeros(img.size(), CV_8UC1);
        cv::circle(area, c, (int)axes.radius, cv::Scalar(255), -1);
        if (findBall(frame, area, ball, mask)) {
            cv::Vec2d d(ball[0] - axes.centre.x, ball[1] - axes.centre.y);
            cv::Vec2d cm = axes.toCm * d;
            out.x_cm = (float)cm[0];
            out.y_cm = (float)cm[1];
            found = true;
            cv::circle(img, cv::Point((int)ball[0], (int)ball[1]), (int)ball[2], cv::Scalar(0, 255, 0), 2);
            std::snprintf(buf, sizeof(buf), "x = %+.1f cm   y = %+.1f cm", cm[0], cm[1]);
            text(img, buf, cv::Scalar(0, 255, 0));
        } else {
            text(img, "ball not found");
        }
    }

    cv::imshow("Ball", img);
    cv::imshow("Mask", mask);
    return found;
}

void BallTracker::handleKey(int key) {
    if (key == 'c' && step == Step::Colour) {
        samples.clear();
        haveColour = false;
    }
    if (key == 'n' && haveColour) {
        step = Step::Axes;
        double cx = size_.width / 2.0, cy = size_.height / 2.0;
        double r = size_.height * 0.47;                  // platform radius in pixels
        clicks = {{cx, cy - r}, {cx, cy + r}};           // top, bottom
    }
    if (key == 'r') {
        step = Step::Colour;
        haveColour = haveAxes = false;
        samples.clear();
        clicks.clear();
    }
}

bool BallTracker::calibrated() const { return haveAxes; }

void BallTracker::close() { cap_.release(); }
