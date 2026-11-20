#include "pid_ui.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace {

const char* WIN = "PID Tuning";
const char* PLOT_WIN = "PID Results";
const char* PLOT_FILE = "pid_results.png";

const int W = 520, H = 420;
const int ROW_Y0 = 60, ROW_H = 50;
const int LABEL_X = 20, SLIDER_X0 = 170, SLIDER_X1 = 370;
const int BOX_X0 = 390, BOX_X1 = 500, BOX_H = 26;
const cv::Rect BTN_APPLY(60, 320, 180, 34), BTN_PLOT(280, 320, 180, 34);

const cv::Scalar BG(240, 240, 240), TEXT(0, 0, 0), DIM(110, 110, 110);
const cv::Scalar TRACK(205, 205, 205), EDGE(160, 160, 160), THUMB(225, 225, 225);
const cv::Scalar WHITE(255, 255, 255), BTN(225, 225, 225), FOCUS(215, 120, 0);
const cv::Scalar BLUE(180, 119, 31), ORANGE(14, 127, 255), GRID(225, 225, 225);

const int FONT = cv::FONT_HERSHEY_SIMPLEX;

void put(cv::Mat& img, const std::string& s, cv::Point p, double scale, cv::Scalar c, int th = 1) {
    cv::putText(img, s, p, FONT, scale, c, th, cv::LINE_AA);
}

void putCentred(cv::Mat& img, const std::string& s, cv::Rect r, double scale, cv::Scalar c, int th = 1) {
    int base = 0;
    cv::Size sz = cv::getTextSize(s, FONT, scale, th, &base);
    put(img, s, {r.x + (r.width - sz.width) / 2, r.y + (r.height + sz.height) / 2}, scale, c, th);
}

void button(cv::Mat& img, cv::Rect r, const std::string& s) {
    cv::rectangle(img, r, BTN, cv::FILLED);
    cv::rectangle(img, r, EDGE, 1);
    putCentred(img, s, r, 0.5, TEXT);
}

std::string fmt(const char* f, double v) {
    char b[48];
    std::snprintf(b, sizeof(b), f, v);
    return b;
}

int rowY(int i) { return ROW_Y0 + i * ROW_H + ROW_H / 2; }
cv::Rect boxRect(int i) { return {BOX_X0, rowY(i) - BOX_H / 2, BOX_X1 - BOX_X0, BOX_H}; }
cv::Rect sliderHit(int i) { return {SLIDER_X0 - 10, rowY(i) - 15, SLIDER_X1 - SLIDER_X0 + 20, 30}; }

}  // namespace

PidUi::PidUi(float kp, float ki, float kd, float setX, float setY, float lim)
    : pending_{kp, ki, kd, setX, setY}, active_(pending_) {
    const float BIG = 1e6f;
    rows_ = {
        {"Kp",              &Values::kp,   0.0f, 3.0f, 0.0f, BIG},
        {"Ki",              &Values::ki,   0.0f, 2.0f, 0.0f, BIG},
        {"Kd",              &Values::kd,   0.0f, 2.0f, 0.0f, BIG},
        {"Setpoint X (cm)", &Values::setX, -lim, lim,  -lim, lim},
        {"Setpoint Y (cm)", &Values::setY, -lim, lim,  -lim, lim},
    };
    start_ = Clock::now();
}

void PidUi::log(Clock::time_point t, float x, float y, float tiltX, float tiltY) {
    double ts = std::chrono::duration<double>(t - start_).count();
    log_.push_back({ts, x, y, active_.setX, active_.setY, tiltX, tiltY});
}

void PidUi::apply() {
    commitEdit();
    active_ = pending_;
    if (onApply) onApply(active_.kp, active_.ki, active_.kd);
    std::printf("[APPLY] Kp=%.4f  Ki=%.4f  Kd=%.4f\n", active_.kp, active_.ki, active_.kd);
    std::printf("[APPLY] Setpoint = (%.2f, %.2f) cm\n", active_.setX, active_.setY);
}

void PidUi::mouseThunk(int event, int x, int y, int flags, void* self) {
    static_cast<PidUi*>(self)->onMouse(event, x, y, flags);
}

void PidUi::onMouse(int event, int x, int y, int flags) {
    cv::Point p(x, y);
    if (event == cv::EVENT_LBUTTONDOWN) {
        for (int i = 0; i < (int)rows_.size(); i++)
            if (boxRect(i).contains(p)) {
                if (editRow_ != i) {
                    commitEdit();
                    editRow_ = i;
                    editText_.clear();
                }
                return;
            }
        commitEdit();
        for (int i = 0; i < (int)rows_.size(); i++)
            if (sliderHit(i).contains(p)) {
                dragging_ = i;
                setFromSlider(i, x);
                return;
            }
        if (BTN_APPLY.contains(p)) apply();
        else if (BTN_PLOT.contains(p)) plotRequested_ = true;
    } else if (event == cv::EVENT_MOUSEMOVE) {
        if (dragging_ >= 0 && (flags & cv::EVENT_FLAG_LBUTTON)) setFromSlider(dragging_, x);
    } else if (event == cv::EVENT_LBUTTONUP) {
        dragging_ = -1;
    }
}

void PidUi::setFromSlider(int i, int x) {
    const Row& r = rows_[i];
    float f = std::clamp((float)(x - SLIDER_X0) / (SLIDER_X1 - SLIDER_X0), 0.0f, 1.0f);
    float step = (r.sliderMax - r.sliderMin) / 1000.0f;
    float v = r.sliderMin + f * (r.sliderMax - r.sliderMin);
    pending_.*r.field = std::round(v / step) * step;
}

void PidUi::commitEdit() {
    if (editRow_ < 0) return;
    const Row& r = rows_[editRow_];
    char* end = nullptr;
    float v = std::strtof(editText_.c_str(), &end);
    if (!editText_.empty() && end && *end == '\0' && std::isfinite(v))
        pending_.*r.field = std::clamp(v, r.hardMin, r.hardMax);
    editRow_ = -1;
    editText_.clear();
}

bool PidUi::handleKey(int key) {
    if (editRow_ < 0) return false;
    if (key == 13 || key == 10) commitEdit();
    else if (key == 27) { editRow_ = -1; editText_.clear(); }
    else if (key == 8 || key == 127) {
        if (!editText_.empty()) editText_.pop_back();
    } else if ((key >= '0' && key <= '9') || key == '.' || key == '-' || key == '+') {
        if (editText_.size() < 12) editText_ += (char)key;
    }
    return true;
}

void PidUi::draw() {
    cv::namedWindow(WIN, cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback(WIN, &PidUi::mouseThunk, this);
    cv::imshow(WIN, renderPanel());

    if (plotRequested_) {
        plotRequested_ = false;
        cv::Mat plot = renderPlot();
        cv::imwrite(PLOT_FILE, plot);
        cv::imshow(PLOT_WIN, plot);
        std::printf("[PLOT] Saved %s\n", PLOT_FILE);
    }
}

cv::Mat PidUi::renderPanel() const {
    cv::Mat img(H, W, CV_8UC3, BG);
    putCentred(img, "PID Gains", {0, 10, W, 36}, 0.9, TEXT, 2);

    for (int i = 0; i < (int)rows_.size(); i++) {
        const Row& r = rows_[i];
        int cy = rowY(i);
        float v = pending_.*r.field;

        put(img, r.label, {LABEL_X, cy + 5}, 0.5, TEXT);

        cv::Rect trough(SLIDER_X0, cy - 3, SLIDER_X1 - SLIDER_X0, 6);
        cv::rectangle(img, trough, TRACK, cv::FILLED);
        cv::rectangle(img, trough, EDGE, 1);
        float f = std::clamp((v - r.sliderMin) / (r.sliderMax - r.sliderMin), 0.0f, 1.0f);
        int kx = SLIDER_X0 + (int)std::lround(f * (SLIDER_X1 - SLIDER_X0));
        cv::Rect thumb(kx - 6, cy - 10, 12, 20);
        cv::rectangle(img, thumb, THUMB, cv::FILLED);
        cv::rectangle(img, thumb, EDGE, 1);

        bool editing = editRow_ == i;
        cv::Rect box = boxRect(i);
        cv::rectangle(img, box, WHITE, cv::FILLED);
        cv::rectangle(img, box, editing ? FOCUS : EDGE, 1);
        put(img, editing ? editText_ + "|" : fmt("%.4f", v), {box.x + 6, box.y + 18}, 0.5, TEXT);
    }

    button(img, BTN_APPLY, "Apply");
    button(img, BTN_PLOT, "Plot");

    putCentred(img, fmt("Active: Kp=%.4f  ", active_.kp) + fmt("Ki=%.4f  ", active_.ki) + fmt("Kd=%.4f", active_.kd),
               {0, 366, W, 20}, 0.45, TEXT);
    putCentred(img, fmt("Setpoint = (%.2f, ", active_.setX) + fmt("%.2f) cm", active_.setY),
               {0, 388, W, 20}, 0.45, TEXT);
    return img;
}

cv::Mat PidUi::renderPlot() const {
    const int PW = 820, PH = 600;
    const float POS_RANGE = 15.0f, TILT_RANGE = 10.0f;
    cv::Mat img(PH, PW, CV_8UC3, WHITE);
    const cv::Rect top(80, 50, PW - 110, 210), bot(80, 340, PW - 110, 200);

    if (log_.size() < 2) {
        putCentred(img, "No data yet", {0, 0, PW, PH}, 0.7, DIM);
        return img;
    }

    double t0 = log_.front().t, t1 = log_.back().t;
    double span = std::max(t1 - t0, 1e-3);
    double avg = span / (double)(log_.size() - 1);
    putCentred(img, fmt("%.1f s of data, ", span) + fmt("average loop time %.1f ms", avg * 1000.0),
               {0, 6, PW, 22}, 0.5, TEXT);

    double step = 1;
    for (double s : {1.0, 2.0, 5.0, 10.0, 20.0, 30.0, 60.0, 120.0, 300.0})
        if (span / s <= 8) { step = s; break; }

    auto toPx = [&](const cv::Rect& r, float range, double t, float v) {
        v = std::clamp(v, -range, range);
        int x = r.x + (int)std::lround((t - t0) / span * r.width);
        int y = r.y + r.height / 2 - (int)std::lround(v / range * (r.height / 2));
        return cv::Point(x, y);
    };

    auto axes = [&](const cv::Rect& r, float range, const std::string& label) {
        for (int k = -2; k <= 2; k++) {
            float v = range * k / 2.0f;
            cv::Point a = toPx(r, range, t0, v), b = toPx(r, range, t1, v);
            cv::line(img, a, b, GRID, 1);
            put(img, fmt("%g", v), {r.x - 40, a.y + 5}, 0.4, TEXT);
        }
        for (double s = 0; s <= span + 1e-9; s += step) {
            int x = r.x + (int)std::lround(s / span * r.width);
            cv::line(img, {x, r.y}, {x, r.y + r.height}, GRID, 1);
            put(img, fmt("%g", s), {x - 8, r.y + r.height + 18}, 0.4, TEXT);
        }
        cv::rectangle(img, r, TEXT, 1);
        put(img, label, {r.x, r.y - 10}, 0.48, TEXT);
    };

    auto series = [&](const cv::Rect& r, float range, float Sample::*f, cv::Scalar c, int th, bool dashed) {
        for (size_t i = 1; i < log_.size(); i++) {
            if (log_[i].t - log_[i - 1].t > 0.25) continue;
            if (dashed && (i / 4) % 2) continue;
            cv::line(img, toPx(r, range, log_[i - 1].t, log_[i - 1].*f),
                     toPx(r, range, log_[i].t, log_[i].*f), c, th, cv::LINE_AA);
        }
    };

    axes(top, POS_RANGE, "Ball position (cm)   blue = x, orange = y, dashed = setpoint");
    series(top, POS_RANGE, &Sample::spX, BLUE, 1, true);
    series(top, POS_RANGE, &Sample::spY, ORANGE, 1, true);
    series(top, POS_RANGE, &Sample::x, BLUE, 2, false);
    series(top, POS_RANGE, &Sample::y, ORANGE, 2, false);

    axes(bot, TILT_RANGE, "Control output (tilt, deg)   blue = x, orange = y");
    series(bot, TILT_RANGE, &Sample::tiltX, BLUE, 2, false);
    series(bot, TILT_RANGE, &Sample::tiltY, ORANGE, 2, false);
    putCentred(img, "Time (s)", {bot.x, bot.y + bot.height + 26, bot.width, 20}, 0.45, TEXT);
    return img;
}