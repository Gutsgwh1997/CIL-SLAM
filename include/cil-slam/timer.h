/**
 * @file timer.h
 * @brief 计时类
 * @author GWH
 * @version 0.1
 * @date 2021-03-18
 */
#pragma once

#include <chrono>
#include <stdexcept>

namespace CIL {
/**
 * @brief 默认单位是纳秒，乘以scale对应到想要的单位
 */
class Timer {
public:

    static constexpr double SECONDS = 1e-9;
    static constexpr double MILLISECONDS = 1e-6;
    static constexpr double NANOSECONDS = 1.0;

    Timer(double scale = MILLISECONDS) : started(false), scale(scale) { }
    ~Timer(){}

    void start(){
        started = true;
        start_t = std::chrono::high_resolution_clock::now();
    }

    double stop(){
        std::chrono::high_resolution_clock::time_point end_t = std::chrono::high_resolution_clock::now();

        if (!started) throw std::logic_error("[Timer] Stop called without previous start");

        started = false;
        std::chrono::duration<double, std::nano> elapsed_ns = end_t - start_t;
        return elapsed_ns.count() * scale;
    }

private:

    std::chrono::high_resolution_clock::time_point start_t;
    bool started;
    double scale;
};

// color cout
#ifndef GREEN
#define GREEN (std::cout << "\033[32m")
#endif
#ifndef BOLD_GREEN
#define BOLD_GREEN (std::cout << "\033[1m\033[32m")
#endif
#ifndef BOLD_YELLOW
#define BOLD_YELLOW (std::cout << "\033[1m\033[33m")
#endif
#ifndef BOLD_RED
#define BOLD_RED (std::cout << "\033[1m\033[31m")
#endif
#ifndef BOLD_BLUE
#define BOLD_BLUE (std::cout << "\033[1m\033[36m")
#endif
#ifndef BOLD_WHITE
#define BOLD_WHITE (std::cout << "\033[1m\033[37m")
#endif
#ifndef ENDL
#define ENDL "\033[0m" << std::endl
#endif
// color cout end

} // namespace CIL
