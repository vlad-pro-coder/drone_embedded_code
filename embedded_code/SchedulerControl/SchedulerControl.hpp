#ifndef SCHEDULERCONTROL_H
#define SCHEDULERCONTROL_H

#include "../IncludesHeader.hpp"

class Task {
public:
    bool RanOnce = false;

    using ActionFunc = std::function<void()>;
    using ConditionFunc = std::function<bool()>;

    Task(ActionFunc action, ConditionFunc condition)
        : action_(std::move(action)), condition_(std::move(condition)) {}

    bool Run();

    // Factory to make creating tasks easier
    static std::shared_ptr<Task> make(ActionFunc action, ConditionFunc condition);

private:
    ActionFunc action_;
    ConditionFunc condition_;
};

class Scheduler {
public:
    Scheduler() = default;
    Scheduler& addTask(shared_ptr<Task> t);
    Scheduler& addScheduler(const Scheduler& other);
    bool isSchedulerDone() const;
    void clear();
    void update();

private:
    queue<shared_ptr<Task>> tasks;
};

class WaitSecondsTask : public Task {
public:
    explicit WaitSecondsTask(double sec)
        : seconds(sec)
        , Task(
            ActionFunc([this]() { startTime = std::chrono::steady_clock::now(); }),
            ConditionFunc([this, sec]() {
                auto now = std::chrono::steady_clock::now();
                double elapsed = std::chrono::duration<double>(now - startTime).count();
                return elapsed >= sec;
            })
        )
    {}

private:
    double seconds;
    std::chrono::steady_clock::time_point startTime;
};


#endif