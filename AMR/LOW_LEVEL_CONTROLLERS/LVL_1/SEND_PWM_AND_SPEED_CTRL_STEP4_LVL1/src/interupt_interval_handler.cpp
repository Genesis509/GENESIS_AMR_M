#include "interupt_interval_handler.h"
#include <cstring> // For strcmp

InteruptIntervalHandler::InteruptIntervalHandler(int Interupt)
    : _INTERUPT_TIMER_INTERVAL_MS(Interupt)
{
    // Initialize intervals with default values
    for (int i = 0; i < MAX_INTERVALS; i++) {
        intervals[i].name = "";
        intervals[i].intervalMs = 0;
        intervals[i].reached_flag = false;
        intervals[i].total_count = 0;
        intervals[i].current_count = 0;
    }
}

void InteruptIntervalHandler::add_interval(const char* name, int intervalMs) {
    for (int i = 0; i < MAX_INTERVALS; i++) {
        if (intervals[i].name == "") { // Find an empty slot
            intervals[i].name = name;
            intervals[i].intervalMs = intervalMs;
            intervals[i].reached_flag = false;
            intervals[i].total_count = this->_total_count(intervalMs);
            intervals[i].current_count = 0;
            break;
        }
    }
}

void InteruptIntervalHandler::reset_interval(const char* name) {
    for (int i = 0; i < MAX_INTERVALS; i++) {
        if (strcmp(intervals[i].name, name) == 0)
        {
            intervals[i].reached_flag = false;
            intervals[i].current_count = 0;
            break;
        }
    }
}

void InteruptIntervalHandler::modify_interval(const char* name, int intervalMs) {
    for (int i = 0; i < MAX_INTERVALS; i++) {
        if (strcmp(intervals[i].name, name) == 0)
        {
            intervals[i].intervalMs = intervalMs;
            intervals[i].total_count =this->_total_count(intervalMs);
            intervals[i].current_count = 0;
            intervals[i].reached_flag = false; // Reset reached flag
            break;
        }
    }
}

int InteruptIntervalHandler::_total_count(int intervalMs) {

    if(intervalMs <= 0) {
        return 0; // Avoid division by zero
    }
    else if(intervalMs < _INTERUPT_TIMER_INTERVAL_MS) {
        return 1; // If the interval is smaller than the timer interval, count it as one
    }

    return (intervalMs / _INTERUPT_TIMER_INTERVAL_MS);
}

const interupt_interval_g InteruptIntervalHandler::get_intervals(int index) {
    if (index < 0 || index >= MAX_INTERVALS) {
        // Handle out-of-bounds access
        return {"", 0, false, 0, 0};
    }
    return intervals[index];
}

void InteruptIntervalHandler::update_intervals() {
    for (int i = 0; i < MAX_INTERVALS; i++) {
    { // Only update non-empty intervals
        if(intervals[i].name == "")
            break;
        
        intervals[i].current_count++;
        if (intervals[i].current_count >= intervals[i].total_count)
        {
                intervals[i].reached_flag = true; // Set reached flag
                intervals[i].current_count = 0; // Reset current count
        }
    }
        
    }
}

bool InteruptIntervalHandler::interval_reached(const char* name) {
    for (int i = 0; i < MAX_INTERVALS; i++) {
        if (strcmp(intervals[i].name, name) == 0)
        {   
            bool reached = intervals[i].reached_flag;
            intervals[i].reached_flag = false; // Reset the flag after checking
            return reached;
        }
    }
    return false; // Interval not found
}