#include "PeriodicTimer.hpp"

namespace msp {
PeriodicTimer::PeriodicTimer(std::function<void()> funct, const double period_seconds) :
    funct(std::move(funct)),
    period_us(static_cast<size_t>(period_seconds * 1e6)) {}

bool PeriodicTimer::start() {
    std::lock_guard<std::mutex> lock(mutex_timer);
    if(!stopped || period_us.count() == 0) return false;
    stopped = false;
    thread_ptr = std::make_shared<std::thread>([this] {
        std::unique_lock<std::mutex> timer_lock(mutex_timer);
        while(!stopped) {
            const auto next = std::chrono::steady_clock::now() + period_us;
            timer_lock.unlock();
            funct();
            timer_lock.lock();
            wake.wait_until(timer_lock, next, [this] { return stopped; });
        }
    });
    return true;
}

bool PeriodicTimer::stop() {
    {
        std::lock_guard<std::mutex> lock(mutex_timer);
        if(stopped) return false;
        stopped = true;
    }
    wake.notify_all();
    if(thread_ptr && thread_ptr->joinable()) thread_ptr->join();
    return true;
}

void PeriodicTimer::setPeriod(const double& period_seconds) {
    stop();
    period_us = std::chrono::duration<size_t, std::micro>(
        static_cast<size_t>(period_seconds * 1e6));
    start();
}
}  // namespace msp
