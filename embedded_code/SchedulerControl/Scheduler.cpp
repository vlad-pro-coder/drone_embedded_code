#include "./SchedulerControl.hpp"

Scheduler& Scheduler::addTask(shared_ptr<Task> t) {
    this->tasks.push(t);
    return *this;
}

Scheduler& Scheduler::addScheduler(const Scheduler& other) {
    // Copy all tasks from 'other' into this scheduler
    std::queue<std::shared_ptr<Task>> copy = other.tasks;
    while (!copy.empty()) {
        tasks.push(copy.front());
        copy.pop();
    }
    return *this;
}

bool Scheduler::isSchedulerDone() const {
    return this->tasks.empty();
}

void Scheduler::clear() {
    if (!this->tasks.empty()) {
        this->tasks.front()->RanOnce = false;
    }
    queue<std::shared_ptr<Task>> empty;
    swap(this->tasks, empty);
}

void Scheduler::update() {
    if (this->isSchedulerDone())
        return;
    auto t = this->tasks.front();
    if (t->Run()) {
        this->tasks.pop();
    }
}