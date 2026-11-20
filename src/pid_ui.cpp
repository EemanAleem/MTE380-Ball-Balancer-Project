#include "pid_ui.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {

const char* WIN = "Basic PID Controller";
const char* PLOT_WIN = "PID Results";

const double PLOT_SECONDS = 20.0;
const float PLOT_POS_RANGE_CM = 15.0f;
const float PLOT_TILT_RANGE_DEG = 10.0f;

const int W = 560, H = 580;
const int ROW_Y0 = 70, ROW_H = 52;
const int LABEL_X = 20, SLIDER_X0 = 190, SLIDER_X1 = 400;
const int BOX_X0 = 420, BOX_X1 = 540, BOX_H = 26;
const cv::Rect BTN_APPLY(180, 350, 200, 34);
const cv::Rect BTN_RESET(40, 404, 150, 34), BTN_PLOT(205, 404, 150, 34), BTN_RUN(370, 404, 150, 34);

const cv::Scalar BG(240, 240, 240), TEXT(0, 0, 0), DIM(110, 110, 110), DISABLED(160, 160, 160);
const cv::Scalar TRACK(205, 205, 205), TRACK_EDGE(160, 160, 160);
const cv::Scalar THUMB(225, 225, 225), THUMB_EDGE(120, 120, 120), THUMB_ACTIVE(250, 250, 250);
const cv::Scalar ENTRY(255, 255, 255), ENTRY_EDGE(160, 160, 160);
const cv::Scalar BTN(225, 225, 225), BTN_EDGE(173, 173, 173);
const cv::Scalar FOCUS(215, 120, 0);
const cv::Scalar MPL_BLUE(180, 119, 31), MPL_ORANGE(14, 127, 255), GRID(225, 225, 225), AXES(0, 0, 0);

const int FONT = cv::FONT_HERSHEY_SIMPLEX;

void put(cv::Mat& img, const std::string& s, cv::Point p, double scale, cv::Scalar c, int th = 1) {
    cv::putText(img, s, p, FONT, scale, c, th, cv::LINE_AA);
}

void putCentred(cv::Mat& img, const std::string& s, cv::Rect r, double scale, cv::Scalar c, int th = 1) {
    int base = 0;
    cv::Size sz = cv::getTextSize(s, FONT, scale, th, &base);
    put(img, s, {r.x + (r.width - sz.width) / 2, r.y + (r.height + sz.height) / 2}, scale, c, th);
}

void button(cv::Mat& img, cv::Rect r, const std::string& s, bool enabled = true, bool highlight = false) {
    cv::rectangle(img, r, BTN, cv::FILLED);
    cv::rectangle(img, r, highlight ? FOCUS : BTN_EDGE, highlight ? 2 : 1);
    putCentred(img, s, r, 0.5, enabled ? TEXT : DISABLED, 1);
}

std::string fmt(const char* f, float v) {
    char b[48];
    std::snprintf(b, sizeof(b), f, v);
    return b;
}

int rowY(int i) { return ROW_Y0 + i * ROW_H + ROW_H / 2; }
cv::Rect boxRect(int i) { return {BOX_X0, rowY(i) - BOX_H / 2, BOX_X1 - BOX_X0, BOX_H}; }
cv::Rect sliderHit(int i) { return {SLIDER_X0 - 10, rowY(i) - 15, SLIDER_X1 - SLIDER_X0 + 20, 30}; }

}

PidUi::PidUi(const PidSettings& initial, float lim) : active_(initial), pending_(initial) {
    const float BIG = 1e6f;
    rows_ = {
        {"Kp (Proportional)", &PidSettings::kp,   0.0f, 3.0f, 0.0f, BIG, "%.4f"},
        {"Ki (Integral)",     &PidSettings::ki,   0.0f, 2.0f, 0.0f, BIG, "%.4f"},
        {"Kd (Derivative)",   &PidSettings::kd,   0.0f, 2.0f, 0.0f, BIG, "%.4f"},
        {"Setpoint X (cm)",   &PidSettings::setX, -lim, lim,  -lim, lim, "%.4f"},
        {"Setpoint Y (cm)",   &PidSettings::setY, -lim, lim,  -lim, lim, "%.4f"},
    };
    start_ = Clock::now();
}

double PidUi::nowSec() const { return std::chrono::duration<double>(Clock::now() - start_).count(); }

bool PidUi::takeRestart() {
    bool r = restart_;
    restart_ = false;
    return r;
}

void PidUi::setCanRun(bool canRun) {
    canRun_ = canRun;
    if (!canRun_ && running_) {
        running_ = false;
        std::printf("[STOP] Motors off (camera not calibrated)\n");
    }
}

void PidUi::log(Clock::time_point t, float x, float y, float tiltX, float tiltY) {
    double ts = std::chrono::duration<double>(t - start_).count();
    log_.push_back({ts, x, y, active_.setX, active_.setY, tiltX, tiltY});
}

bool PidUi::hasPending() const {
    for (const Row& r : rows_)
        if (pending_.*r.field != active_.*r.field) return true;
    return false;
}

void PidUi::apply() {
    commitEdit();
    bool gainsChanged = pending_.kp != active_.kp || pending_.ki != active_.ki ||
                        pending_.kd != active_.kd || pending_.tau != active_.tau;
    active_ = pending_;
    if (gainsChanged) restart_ = true;
    std::printf("[APPLY] Kp=%.4f, Ki=%.4f, Kd=%.4f, Setpoint=(%.4f, %.4f)\n",
                active_.kp, active_.ki, active_.kd, active_.setX, active_.setY);
}

void PidUi::resetIntegral() {
    restart_ = true;
    std::printf("[RESET] Integral term reset\n");
}

void PidUi::toggleRunning() {
    if (!canRun_) {
        std::printf("[START] Calibrate the camera first\n");
        return;
    }
    running_ = !running_;
    std::printf(running_ ? "[START] Motors on\n" : "[STOP] Motors off\n");
}

void PidUi::togglePlot() {
    plotOpen_ = !plotOpen_;
    plotFrames_ = 0;
    if (!plotOpen_) {
        try {
            cv::destroyWindow(PLOT_WIN);
        } catch (const cv::Exception&) {
            // already closed
        }
    }
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
        else if (BTN_RESET.contains(p)) resetIntegral();
        else if (BTN_PLOT.contains(p)) togglePlot();
        else if (BTN_RUN.contains(p)) toggleRunning();
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
    if (editRow_ < 0) {
        if (key == 'm') {
            toggleRunning();
            return true;
        }
        return false;
    }
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

    if (plotOpen_) {
        if (plotFrames_ > 2) {
            bool closed;
            try {
                closed = cv::getWindowProperty(PLOT_WIN, cv::WND_PROP_AUTOSIZE) < 0;
            } catch (const cv::Exception&) {
                closed = true;
            }
            if (closed) {
                plotOpen_ = false;
                return;
            }
        }
        cv::imshow(PLOT_WIN, renderPlot());
        plotFrames_++;
    }
}

cv::Mat PidUi::renderPanel() const {
    cv::Mat img(H, W, CV_8UC3, BG);

    putCentred(img, "PID Gains", {0, 14, W, 40}, 0.9, TEXT, 2);

    for (int i = 0; i < (int)rows_.size(); i++) {
        const Row& r = rows_[i];
        int cy = rowY(i);
        float v = pending_.*r.field;
        bool changed = v != active_.*r.field;

        put(img, r.label, {LABEL_X, cy + 5}, 0.5, TEXT);

        cv::Rect trough(SLIDER_X0, cy - 3, SLIDER_X1 - SLIDER_X0, 6);
        cv::rectangle(img, trough, TRACK, cv::FILLED);
        cv::rectangle(img, trough, TRACK_EDGE, 1);
        float f = std::clamp((v - r.sliderMin) / (r.sliderMax - r.sliderMin), 0.0f, 1.0f);
        int kx = SLIDER_X0 + (int)std::lround(f * (SLIDER_X1 - SLIDER_X0));
        cv::Rect thumb(kx - 6, cy - 10, 12, 20);
        cv::rectangle(img, thumb, dragging_ == i ? THUMB_ACTIVE : THUMB, cv::FILLED);
        cv::rectangle(img, thumb, dragging_ == i ? FOCUS : THUMB_EDGE, 1);

        bool editing = editRow_ == i;
        cv::Rect box = boxRect(i);
        cv::rectangle(img, box, ENTRY, cv::FILLED);
        cv::rectangle(img, box, editing ? FOCUS : ENTRY_EDGE, 1);
        std::string s = editing ? editText_ + "|" : fmt(r.fmt, v);
        put(img, s, {box.x + 6, box.y + 18}, 0.5, (changed && !editing) ? FOCUS : TEXT);
    }

    button(img, BTN_APPLY, "Apply Parameters", true, hasPending());

    button(img, BTN_RESET, "Reset Integral");
    button(img, BTN_PLOT, plotOpen_ ? "Hide Plot" : "Plot Results");
    button(img, BTN_RUN, running_ ? "Stop" : "Start", canRun_ || running_);

    putCentred(img,
               fmt("Active parameters: Kp=%.4f, ", active_.kp) + fmt("Ki=%.4f, ", active_.ki) +
                   fmt("Kd=%.4f,", active_.kd),
               {0, 462, W, 20}, 0.45, TEXT);
    putCentred(img, fmt("Setpoint=(%.4f, ", active_.setX) + fmt("%.4f) cm", active_.setY),
               {0, 484, W, 20}, 0.45, TEXT);

    std::string motors = !canRun_ ? "Motors: off (calibrate the camera first)"
                       : running_ ? "Motors: running" : "Motors: stopped";
    putCentred(img, motors, {0, 520, W, 18}, 0.42, DIM);
    std::string live = "Ball: not found";
    if (!log_.empty() && nowSec() - log_.back().t < 0.3) {
        const Sample& s = log_.back();
        live = fmt("Ball: x=%+.1f cm, ", s.x) + fmt("y=%+.1f cm", s.y);
        if (s.controlling) live += fmt("   Tilt: x=%+.2f, ", s.tiltX) + fmt("y=%+.2f deg", s.tiltY);
    }
    putCentred(img, live, {0, 542, W, 18}, 0.42, DIM);
    return img;
}

cv::Mat PidUi::renderPlot() const {
    const int PW = 820, PH = 600;
    cv::Mat img(PH, PW, CV_8UC3, cv::Scalar(255, 255, 255));
    const cv::Rect top(80, 50, PW - 110, 210), bot(80, 340, PW - 110, 200);

    double tEnd = nowSec();
    double tStart = tEnd - PLOT_SECONDS;
    auto first = std::lower_bound(log_.begin(), log_.end(), tStart,
                                  [](const Sample& s, double t) { return s.t < t; });
    long n = (long)(log_.end() - first);

    // average loop time, like the Python version's printout
    std::string title = "Last 20 s";
    if (n >= 2) {
        double avg = (log_.back().t - first->t) / (double)(n - 1);
        if (avg > 0)
            title = fmt("Average loop time: %.1f ms", (float)(avg * 1000)) + fmt(" (%.1f Hz)", (float)(1.0 / avg));
    }
    putCentred(img, title, {0, 6, PW, 22}, 0.5, TEXT);

    auto toPx = [&](const cv::Rect& r, float range, double t, float v) {
        v = std::clamp(v, -range, range);
        int x = r.x + (int)std::lround((t - tStart) / PLOT_SECONDS * r.width);
        int y = r.y + r.height / 2 - (int)std::lround(v / range * (r.height / 2));
        return cv::Point(x, y);
    };

    auto axesBox = [&](const cv::Rect& r, float range, const std::string& ylabel) {
        for (int k = -2; k <= 2; k++) {
            float v = range * k / 2.0f;
            cv::Point a = toPx(r, range, tStart, v), b = toPx(r, range, tEnd, v);
            cv::line(img, a, b, GRID, 1);
            put(img, fmt("%g", v), {r.x - 40, a.y + 5}, 0.4, TEXT);
        }
        for (int s = 0; s <= (int)PLOT_SECONDS; s += 5) {
            int x = r.x + (int)std::lround(s / PLOT_SECONDS * r.width);
            cv::line(img, {x, r.y}, {x, r.y + r.height}, GRID, 1);
            std::string lbl = s == (int)PLOT_SECONDS ? "now" : fmt("-%.0f", (float)(PLOT_SECONDS - s));
            put(img, lbl, {x - 12, r.y + r.height + 18}, 0.4, TEXT);
        }
        cv::rectangle(img, r, AXES, 1);
        put(img, ylabel, {r.x, r.y - 10}, 0.48, TEXT);
    };

    auto series = [&](const cv::Rect& r, float range, float Sample::*f, cv::Scalar c, int th, bool dashed) {
        bool havePrev = false;
        cv::Point prev;
        double prevT = 0;
        int k = 0;
        for (auto it = first; it != log_.end(); ++it, ++k) {
            cv::Point p = toPx(r, range, it->t, (*it).*f);
            bool draw = havePrev && it->t - prevT < 0.25 && (!dashed || (k / 4) % 2 == 0);
            if (draw) cv::line(img, prev, p, c, th, cv::LINE_AA);
            prev = p;
            prevT = it->t;
            havePrev = true;
        }
    };

    auto legend = [&](const cv::Rect& r, const std::vector<std::pair<std::string, cv::Scalar>>& items,
                      const std::vector<bool>& dashed) {
        int x = r.x + r.width - 230, y = r.y + 18;
        cv::Rect box(x - 10, r.y + 6, 236, 20 * (int)items.size() + 8);
        cv::rectangle(img, box, cv::Scalar(255, 255, 255), cv::FILLED);
        cv::rectangle(img, box, cv::Scalar(204, 204, 204), 1);
        for (size_t i = 0; i < items.size(); i++) {
            int yy = y + 20 * (int)i;
            if (dashed[i]) {
                cv::line(img, {x, yy - 4}, {x + 8, yy - 4}, items[i].second, 2);
                cv::line(img, {x + 14, yy - 4}, {x + 22, yy - 4}, items[i].second, 2);
            } else {
                cv::line(img, {x, yy - 4}, {x + 22, yy - 4}, items[i].second, 2);
            }
            put(img, items[i].first, {x + 30, yy}, 0.42, TEXT);
        }
    };

    axesBox(top, PLOT_POS_RANGE_CM, "Ball position (cm)");
    series(top, PLOT_POS_RANGE_CM, &Sample::spX, MPL_BLUE, 1, true);
    series(top, PLOT_POS_RANGE_CM, &Sample::spY, MPL_ORANGE, 1, true);
    series(top, PLOT_POS_RANGE_CM, &Sample::x, MPL_BLUE, 2, false);
    series(top, PLOT_POS_RANGE_CM, &Sample::y, MPL_ORANGE, 2, false);
    legend(top, {{"x position", MPL_BLUE}, {"y position", MPL_ORANGE}, {"setpoints (dashed)", DIM}},
           {false, false, true});

    axesBox(bot, PLOT_TILT_RANGE_DEG, "Control output (tilt, deg)");
    series(bot, PLOT_TILT_RANGE_DEG, &Sample::tiltX, MPL_BLUE, 2, false);
    series(bot, PLOT_TILT_RANGE_DEG, &Sample::tiltY, MPL_ORANGE, 2, false);
    legend(bot, {{"x control", MPL_BLUE}, {"y control", MPL_ORANGE}}, {false, false});
    putCentred(img, "Time (s)", {bot.x, bot.y + bot.height + 26, bot.width, 20}, 0.45, TEXT);

    if (n == 0) putCentred(img, "No data yet - the ball has not been seen in the last 20 s", top, 0.5, DIM);
    return img;
}