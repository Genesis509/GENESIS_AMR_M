#ifndef WHEEL_SPEEL_CONTROL_H
#define WHEEL_SPEEL_CONTROL_H
#include <arduino.h> //for micros()
#define MY_PI 3.14159265358979323846 // PI 3.14159265358979323846
#define ROT_PER_SEC_TO_RAD_PER_SEC 6.28318530718 // 2 * PI

#define MAX_ANGULAR_VELOCITY 4.5 //rad/s
#define MAX_LINEAR_VELOCITY 0.8 //m/s


typedef enum {Forward, Backward, Stop} WheelState_t;

typedef struct Wheel_Characteristics_t {
  float wheel_dia_cm; 
  float wheel_cir_cm; 
  float wheel_rad_cm; 
  float Lx_cm; 
  float Ly_cm; 
  float encoder_resolution; // in counts per revolution
} Wheel_Characteristics_t;

typedef struct PID_Handler_t {
  float Kp;
  float Ki;
  float Kd;
  float previous_error;
  float integral;
  unsigned long previous_time;
  float max_integral;
  int prev_output;
  float output;
  float previous_derivative;
  int deadband; // Deadband for PWM output
  float MAX_ERROR = 2.0; //No longer using this
} PID_Handler_t;

typedef struct Wheel_speed_handler_t {
  volatile long *encoder_count;
  volatile long previous_encoder_count;
  unsigned long previous_time_wheel_speed;
  float wheel_speed;//in rad/s
  float previous_wheel_speed;
  static const int SPEED_FILTER_SIZE = 5; // Size of speed filter buffer
  float filtered_speed_buffer[SPEED_FILTER_SIZE]; // Buffer for speed filtering
  float filtered_speed;
  float MIN_TIME_INTERVAL = 0.005; // No longer using this
  int current_index = 0; // Current index for speed buffer
  int wheel_pwm_deadband = 40; // Deadband for wheel speed PWM output
  const float MAX_WHEEL_SPEED = 21.0; // Maximum wheel speed in cm/s
  const float MAX_POSSIBLE_SPEED = 25.0; //Guessing
  long encoder_diff;
  
}Wheel_speed_handler_t;



typedef struct PID_gains_t {
  float Kp;
  float Ki;
  float Kd;
  float max_integral;
}PID_gains_t;

typedef struct Gain_Scheduler_t {
  static const int MAX_GAINS = 20;
  static const int MIN_MID_MAX = 3;
  PID_gains_t Operation_Point_gains[MAX_GAINS];
  int NUM_OPERATION_POINTS ;
  float operation_point_speed_range [MAX_GAINS][2]; //min speed_excluded, mid speed use for tunning , max spped included , 
  int index_initialized = 0;
}Gain_Scheduler_t;

class WheelPIDSpeedControl 
{

  private:
    Wheel_Characteristics_t characteristics;
    PID_Handler_t pid_;
    WheelState_t state;
    Wheel_speed_handler_t speed_handler;
    Gain_Scheduler_t gain_scheduler;

    int pwm_output = 0; // Output PWM value after PID control
    
    void initCharacteristics();
    void initPID();
    void initGainScheduler();
    void initWheelSpeedHandler(volatile long *encoder_count);
    float get_filtered_speed();
    void set_PID_Gains_Based_on_tartget(float target_speed);  
     
  public:
    void set_pid_gains_operation_point(float target_speed, PID_gains_t pid_gains);
    void set_number_of_operation_points(int num_points);
    int PID_CONTROLLER(float target_speed);
    WheelPIDSpeedControl(volatile long *encoder_count);
    void Set_PID_Coef(float Kp, float Ki, float Kd);
    void update_current_speed();
    void update_filtered_speed();
    float get_speed() {
      return speed_handler.filtered_speed;
    }
    float (get_min_time_interval)() const{
      return speed_handler.MIN_TIME_INTERVAL;
    }

    void update_current_speed_interval();
    long get_encoder_diff() const{
      return speed_handler.encoder_diff;
    }
    Gain_Scheduler_t& get_gain_scheduler() { return gain_scheduler; }

};


class PID_TUNNER {
  private:
    Gain_Scheduler_t& gain_scheduler;
    int current_tuning_index;
    bool tuning_mode;
    unsigned long last_print_time;
    const unsigned long PRINT_INTERVAL = 2000; // Print gains every 2 seconds
    float current_tuning_speed;
    float target_speed = 0.0;
    // Helper methods
    int find_operation_point_index(float speed);
    void print_current_gains();
    void print_operation_point_info();
    void print_help();
  public:
    PID_TUNNER(Gain_Scheduler_t &gain_scheduler);
    
    // Main interface methods
    void handle_serial_input();
    void update(); // Call this in main loop for periodic printing
    float get_target_speed() const { return target_speed; }
    // Command processing methods
    bool parse_command(String command);
    void set_tuning_operation_point(float speed);
    void update_gain(char gain_type, float value);
    void exit_tuning_mode();
    
    // Status methods
    bool is_tuning_mode() const { return tuning_mode; }
    int get_current_tuning_index() const { return current_tuning_index; }
};


#endif // WHEEL_SPEEL_CONTROL_H