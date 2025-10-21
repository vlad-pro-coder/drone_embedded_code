#include "./SchedulerControl.hpp"

bool Task::Run()
{
    if (!RanOnce) {
            RanOnce = true;
            this->action_();
        }

        if (this->condition_()) {
            RanOnce = false;
            return true;
        }
        return false;
}

std::shared_ptr<Task> Task::make(ActionFunc action, ConditionFunc condition) {
    return std::make_shared<Task>(std::move(action), std::move(condition));
}