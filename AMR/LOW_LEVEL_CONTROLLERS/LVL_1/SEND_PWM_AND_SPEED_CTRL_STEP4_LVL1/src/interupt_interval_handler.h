#ifndef INTERUPT_INTERVAL_HANDLER_H
#define INTERUPT_INTERVAL_HANDLER_H


#define INTERUPT_TIMER_INTERVAL_MS 1 // For smaller intervals, e.g., 100ms


typedef struct interupt_interval_g{
    const char* name; // Name of the interval
    int intervalMs; // Interval in milliseconds
    bool reached_flag; // Flag to indicate if the interval is active
    int total_count; // Count of how many times the samllest interval has been reached
    int current_count;
}interupt_interval_g;

class InteruptIntervalHandler {
public:

    InteruptIntervalHandler(int Interupt);
    void add_interval(const char* name, int intervalMs); // intervalMs is the desired interval in milliseconds
    void reset_interval(const char* name);
    void modify_interval(const char* name, int intervalMs);
    void update_intervals();
    const interupt_interval_g get_intervals(int index);
    bool interval_reached(const char* name);
private:
    static const int MAX_INTERVALS = 10; // Maximum number of intervals
    interupt_interval_g intervals[MAX_INTERVALS]; // Array to hold intervals
    const int _INTERUPT_TIMER_INTERVAL_MS; // Interval in milliseconds
    int _total_count(int intervalMs); 
    
};
#endif // INTERUPT_INTERVAL_HANDLER_H