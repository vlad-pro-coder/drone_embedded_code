#include "./MathHelpers.hpp"

Timer::Timer() { start(); }
void Timer::start() {
    start_time = std::chrono::steady_clock::now();
}
void Timer::reset(){
    start();
}

double Timer::seconds() const {
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> diff = now - start_time;
    return diff.count();
}

double Timer::milliseconds() const {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
}