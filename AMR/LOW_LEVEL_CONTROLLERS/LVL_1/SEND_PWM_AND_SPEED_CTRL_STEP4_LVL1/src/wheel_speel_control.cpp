#include "wheel_speel_control.h"

WheelPIDSpeedControl::WheelPIDSpeedControl(volatile long *encoder_count)
{
  initCharacteristics();
  initPID();
  initWheelSpeedHandler(encoder_count);
  initGainScheduler();
  state = WheelState_t::Stop;
}

void WheelPIDSpeedControl::initGainScheduler() {

  for(int i = 0; i < gain_scheduler.NUM_OPERATION_POINTS; i++) {
    gain_scheduler.Operation_Point_gains[i].Kp = 0.0;
    gain_scheduler.Operation_Point_gains[i].Ki = 0.0;
    gain_scheduler.Operation_Point_gains[i].Kd = 0.0;
    gain_scheduler.operation_point_speed_range[i][0] = 0.0;
    gain_scheduler.operation_point_speed_range[i][1] = 0.0;
    gain_scheduler.operation_point_speed_range[i][2] = 0.0;
  }
  int num_points = 7;
  set_number_of_operation_points(num_points);

  float target_speed_2 = 2;
  PID_gains_t pid_gains_2 = {8, 2, 0.8, 40}; // Kp, Ki, Kd , Max Integral

  float target_speed_5 = 5;
  PID_gains_t pid_gains_5 = {6, 1.8, 2, 50};
  
  float target_speed_8 = 8;
  PID_gains_t pid_gains_10 = {7, 1, 2, 60};

  float target_speed_11 = 11;
  PID_gains_t pid_gains_11 = {5, 1, 4, 70}; // 8 1

  float target_speed_14 = 14;
  PID_gains_t pid_gains_14 = {6, 1.5, 4, 80};

  float target_speed_17 = 17;
  PID_gains_t pid_gains_20 = {6, 1.5, 4, 120};

  float target_speed_20 = 20;
  PID_gains_t pid_gains_21 = {6, 1.8 , 4, 150};

  set_pid_gains_operation_point(target_speed_2, pid_gains_2);
  set_pid_gains_operation_point(target_speed_5, pid_gains_5);
  set_pid_gains_operation_point(target_speed_8, pid_gains_10);
  set_pid_gains_operation_point(target_speed_11, pid_gains_11);
  set_pid_gains_operation_point(target_speed_14, pid_gains_14);
  set_pid_gains_operation_point(target_speed_17, pid_gains_20);
  set_pid_gains_operation_point(target_speed_20, pid_gains_21);

}

void WheelPIDSpeedControl::set_number_of_operation_points(int num_points) {
  gain_scheduler.NUM_OPERATION_POINTS = num_points;

  float step = this->speed_handler.MAX_WHEEL_SPEED / num_points;

  for(int i = 0; i < num_points; i++) {
    gain_scheduler.operation_point_speed_range[i][0] = i * step;
    gain_scheduler.operation_point_speed_range[i][1] = (i + 1) * step;
  }

}

void WheelPIDSpeedControl::set_pid_gains_operation_point(float target_speed, PID_gains_t pid_gains) {
  
  for(int i = 0; i < gain_scheduler.NUM_OPERATION_POINTS; i++) {
    if(target_speed >= gain_scheduler.operation_point_speed_range[i][0] && target_speed <= gain_scheduler.operation_point_speed_range[i][1]) {
      gain_scheduler.Operation_Point_gains[i].Kp = pid_gains.Kp;
      gain_scheduler.Operation_Point_gains[i].Ki = pid_gains.Ki;
      gain_scheduler.Operation_Point_gains[i].Kd = pid_gains.Kd;
      gain_scheduler.Operation_Point_gains[i].max_integral = pid_gains.max_integral;
    }
  }

}



void WheelPIDSpeedControl::initCharacteristics() {
  characteristics.wheel_dia_cm = 8; //  value in cm
  characteristics.wheel_cir_cm = characteristics.wheel_dia_cm * M_PI; // Calculate circumference
  characteristics.wheel_rad_cm = characteristics.wheel_dia_cm / 2.0; // Calculate radius
  characteristics.Lx_cm = 7.75; //  value in cm
  characteristics.Ly_cm = 10; //  value in cm
  characteristics.encoder_resolution = 1495; //  value in counts per revolution
}

void WheelPIDSpeedControl::initPID() {
  pid_.Kp = 10.0; // Proportional coefficient
  pid_.Ki = 1.8; // Integral coefficient 5
  pid_.Kd = 0.5; // Derivative coefficient 2 wasn't bad
  pid_.previous_error = 0;
  pid_.integral = 0;
  pid_.previous_time = micros();
  pid_.max_integral = 40; // Limit for integral to prevent windup (slightly big enough to overcome friction)
  pid_.prev_output = 0;
  pid_.output = 0;
}

void WheelPIDSpeedControl::initWheelSpeedHandler(volatile long *encoder_count) {
  speed_handler.encoder_count = encoder_count; // Use a processed encoder count (by another class)
  speed_handler.previous_encoder_count = 0;
  speed_handler.previous_time_wheel_speed = micros();
  speed_handler.wheel_speed = 0.0;
  speed_handler.previous_wheel_speed = 0.0;
  speed_handler.MIN_TIME_INTERVAL = 0.01; // Not using this anymore , a new class is responsible for the time interval and "real time" stuff
  
  // Initialize speed filter buffer
  for (int i = 0; i < speed_handler.SPEED_FILTER_SIZE; i++) {
    speed_handler.filtered_speed_buffer[i] = 0.0;
  }
}

void WheelPIDSpeedControl::update_current_speed() {
  unsigned long current_time = micros();
  float time_interval = (current_time - this->speed_handler.previous_time_wheel_speed) / 1000000.0;
  
  long count_diff = *(speed_handler.encoder_count) - this->speed_handler.previous_encoder_count;
  speed_handler.encoder_diff = count_diff;
  
  // Detect unreasonable encoder jumps (could probably remove this line, the encoder handler does some' similar)
  if (abs(count_diff) > this->characteristics.encoder_resolution) {
     this->speed_handler.wheel_speed = this->speed_handler.previous_wheel_speed;
     this->speed_handler.previous_encoder_count = *(speed_handler.encoder_count);
     return ;
  }
  
  this->speed_handler.previous_encoder_count = *(speed_handler.encoder_count);
  this->speed_handler.previous_wheel_speed = this->speed_handler.wheel_speed;
  
  float revolutions = (float)count_diff / this->characteristics.encoder_resolution;
  float radians = revolutions * 2.0 * MY_PI;
  
  
  float raw_speed = radians / time_interval;
  
  /* static float prev_raw_speed = 0.0;
  static float prev_filtered_speed = 0.0; */
  // Apply low-pass filter to smooth speed changes (I know magic numbers I feel like they are obvious enough)
  //y[n] = a₁·y[n-1] + a₂·x[n-1] + a₃·x[n] blablabla math -> z trasnform boom cut off frequency = 8.5 Hz

  //float filtered_speed = prev_filtered_speed * 0.7 + 0.15 * prev_raw_speed  + 0.15 * raw_speed;
  float filtered_speed = 0.4 * this->speed_handler.previous_wheel_speed + 0.6 * raw_speed;

  if(abs(raw_speed) >= speed_handler.MAX_POSSIBLE_SPEED) {
     return; // Okay I need to figure out what to do if/when this happens (definitely a bug if it happens)
  }

  
  this->speed_handler.wheel_speed = filtered_speed;
  // Update speed filter buffer
  this->speed_handler.filtered_speed_buffer[this->speed_handler.current_index] = this->speed_handler.wheel_speed;
  
  this->speed_handler.previous_time_wheel_speed = current_time;

  this->speed_handler.current_index = (this->speed_handler.current_index + 1) % speed_handler.SPEED_FILTER_SIZE; // Wrap aroundcurrent_index + 1) % speed_handler.SPEED_FILTER_SIZE - 1;
  
  if(this->speed_handler.current_index == 0) {
    update_filtered_speed();
  }
  //return ;
  
}

void WheelPIDSpeedControl::update_filtered_speed() {
  float sum = 0.0;
  int validCount = 0;
  
  for(int i = 0; i < speed_handler.SPEED_FILTER_SIZE; i++) {
    sum += speed_handler.filtered_speed_buffer[i];
    validCount++;
  }
  
  speed_handler.filtered_speed = (validCount > 0) ? sum / validCount : 0.0;
}

void WheelPIDSpeedControl::set_PID_Gains_Based_on_tartget(float target_speed)
{
  static float prev_target_speed = -999.0;
  
  if(prev_target_speed == target_speed || abs(target_speed - prev_target_speed) < 0.5) {
    return;
  }
  
  prev_target_speed = target_speed;
  float abs_target_speed = abs(target_speed);
  
  for(int i = 0; i < gain_scheduler.NUM_OPERATION_POINTS; i++) {
    if(abs_target_speed >= gain_scheduler.operation_point_speed_range[i][0] && abs_target_speed <= gain_scheduler.operation_point_speed_range[i][1]) {
      pid_.Kp = gain_scheduler.Operation_Point_gains[i].Kp;
      pid_.Ki = gain_scheduler.Operation_Point_gains[i].Ki;
      pid_.Kd = gain_scheduler.Operation_Point_gains[i].Kd;
      pid_.max_integral = gain_scheduler.Operation_Point_gains[i].max_integral;
      return;
    }
  }
  
}


int WheelPIDSpeedControl::PID_CONTROLLER(float target_speed)
{

  set_PID_Gains_Based_on_tartget(target_speed);
  if (abs(target_speed) < 0.1) { 
    this->pid_.integral = 0;   
    this->pid_.previous_error = 0;
    this->pid_.previous_derivative = 0;
    return 0; 
  }
  
  if(abs(target_speed) > this->speed_handler.MAX_WHEEL_SPEED) {
    target_speed = this->speed_handler.MAX_WHEEL_SPEED * (target_speed > 0 ? 1 : -1); // Limit target speed
  }
  // Get current wheel speed
  float current_speed = this->speed_handler.filtered_speed;//get_filtered_speed();// get_speed();

  float error = target_speed - current_speed;

  unsigned long current_time = micros();
  
  // Calculate time delta in seconds
  float dt = (current_time - this->pid_.previous_time) / 1000000.0;
  
  // Calculate integral term with anti-windup
  if (abs(this->pid_.prev_output) < 255) {
    this->pid_.integral += error * dt;
  }
  
  // Limit integral to prevent windup
  if (this->pid_.integral > this->pid_.max_integral) this->pid_.integral = this->pid_.max_integral;
  if (this->pid_.integral < -this->pid_.max_integral) this->pid_.integral = -this->pid_.max_integral;
  
  float derivative = (error - this->pid_.previous_error) / dt;
  
  //blabla magic numbers (obvious numbers) 
  float filtered_derivative = 0.4 * this->pid_.previous_derivative + 0.6 * derivative;
  this->pid_.previous_derivative = filtered_derivative;
  
  float output = (this->pid_.Kp * error) + 
                (this->pid_.Ki * this->pid_.integral)  + 
                (this->pid_.Kd * filtered_derivative);   
                
  
  // Store current error and time for next iteration
  this->pid_.previous_error = error;
  this->pid_.previous_time = current_time;
  
  // Convert output to PWM value
  int pwm_output = (int)output;
  
  if (pwm_output > 255) pwm_output = 255;
  if (pwm_output < -255) pwm_output = -255;
  
  float maxOutputChange = 20.0; 
  if(abs(pwm_output - this->pid_.prev_output) > maxOutputChange) {
      pwm_output = this->pid_.prev_output + (pwm_output > this->pid_.prev_output ? maxOutputChange : -maxOutputChange);
  }

  this->pid_.prev_output = pwm_output;

  
  return pwm_output;
}

void WheelPIDSpeedControl::Set_PID_Coef(float Kp, float Ki, float Kd) {
  pid_.Kp = Kp;
  pid_.Ki = Ki;
  pid_.Kd = Kd;
  pid_.previous_time = micros();
  pid_.previous_error = 0;
  pid_.integral = 0;
  pid_.max_integral = 40; //default, they have no buisness changing this
  pid_.prev_output = 0;
  pid_.output = 0;
  pid_.previous_derivative = 0;
  
}

float WheelPIDSpeedControl::get_filtered_speed() {
  float sum = 0.0;
    int validCount = 0;
    
    for(int i = 0; i < speed_handler.SPEED_FILTER_SIZE; i++) {
        sum += speed_handler.filtered_speed_buffer[i];
        validCount++;
    }
    
    return (validCount > 0) ? sum / validCount : 0.0;
}

void WheelPIDSpeedControl::update_current_speed_interval() { //no longer using this , another class is responsibl for intervals
  if(micros() - speed_handler.previous_time_wheel_speed < speed_handler.MIN_TIME_INTERVAL * 1000000) {
    return; // Skip if time interval is too small
  }
  update_current_speed(); // Call the update function to calculate speed
}

// -------------------------------------- PID_TUNNER ------------------------------------------------------
//---------------------------------------------------------------------------------------------------------

PID_TUNNER::PID_TUNNER(Gain_Scheduler_t &gain_scheduler) 
    : gain_scheduler(gain_scheduler), 
      current_tuning_index(-1), 
      tuning_mode(false),
      last_print_time(0),
      current_tuning_speed(0.0) {
}

void PID_TUNNER::handle_serial_input() {
    static String inputBuffer = "";
    
    while (Serial.available()) {
        char c = Serial.read();
        
        if (c == '\n' || c == '\r') {
            if (inputBuffer.length() > 0) {
                if (!parse_command(inputBuffer)) {
                    // If not a tuning command, treat as target speed
                    float target = inputBuffer.toFloat();
                    if (target >= -20.0 && target <= 20.0) {
                        // Update your existing target_speed variable here
                        Serial.print("Target speed set to: ");
                        Serial.println(target);
                        this->target_speed = target;
                    } else {
                        Serial.println("Invalid target speed. Range: -20 to 20 rad/s");
                    }
                }
                inputBuffer = "";
            }
        } else {
            inputBuffer += c;
        }
    }
}

bool PID_TUNNER::parse_command(String command) {
    command.trim();
    command.toLowerCase();
    
    // Check for operation point selection: "op:speed"
    if (command.startsWith("op:")) {
        float speed = command.substring(3).toFloat();
        set_tuning_operation_point(speed);
        return true;
    }
    
    // Check for gain updates: "kp:value", "ki:value", "kd:value", "max:value"
    if (command.indexOf(':') > 0) {
        int colonIndex = command.indexOf(':');
        String gainType = command.substring(0, colonIndex);
        float value = command.substring(colonIndex + 1).toFloat();
        
        if (gainType == "kp" || gainType == "ki" || gainType == "kd" || gainType == "max") {
            if (tuning_mode) {
                update_gain(gainType.charAt(1), value);
                return true;
            } else {
                Serial.println("Not in tuning mode. Use 'op:speed' first.");
                return true;
            }
        }
    }
    
    // Check for exit command
    if (command == "exit" || command == "quit") {
        exit_tuning_mode();
        return true;
    }
    
    // Check for help command
    if (command == "help" || command == "?") {
        print_help();
        return true;
    }
    
    return false; // Not a recognized tuning command
}

void PID_TUNNER::set_tuning_operation_point(float speed) {
    current_tuning_speed = abs(speed);
    current_tuning_index = find_operation_point_index(current_tuning_speed);
    
    if (current_tuning_index >= 0) {
        tuning_mode = true;
        last_print_time = 0; // Force immediate print
        Serial.println("\n=== PID TUNING MODE ACTIVATED ===");
        print_operation_point_info();
        print_current_gains();
        Serial.println("Commands: kp:val, ki:val, kd:val, max:val, exit");
        Serial.println("=====================================\n");
    } else {
        Serial.print("Error: No operation point found for speed ");
        Serial.println(speed);
        tuning_mode = false;
    }
}

int PID_TUNNER::find_operation_point_index(float speed) {
    for (int i = 0; i < gain_scheduler.NUM_OPERATION_POINTS; i++) {
        if (speed >= gain_scheduler.operation_point_speed_range[i][0] && 
            speed <= gain_scheduler.operation_point_speed_range[i][1]) {
            return i;
        }
    }
    return -1; // Not found
}

void PID_TUNNER::update_gain(char gain_type, float value) {
    if (current_tuning_index < 0) return;
    
    PID_gains_t& gains = gain_scheduler.Operation_Point_gains[current_tuning_index];
    
    switch (gain_type) {
        case 'p': // kp
            gains.Kp = value;
            Serial.print("Kp updated to: ");
            Serial.println(value);
            break;
        case 'i': // ki  
            gains.Ki = value;
            Serial.print("Ki updated to: ");
            Serial.println(value);
            break;
        case 'd': // kd
            gains.Kd = value;
            Serial.print("Kd updated to: ");
            Serial.println(value);
            break;
        case 'a': // max_integral
            gains.max_integral = value;
            Serial.print("Max integral updated to: ");
            Serial.println(value);
            break;
        default:
            Serial.println("Invalid gain type");
            return;
    }
    
    print_current_gains();
}

void PID_TUNNER::print_current_gains() {
    if (current_tuning_index < 0) return;
    
    PID_gains_t& gains = gain_scheduler.Operation_Point_gains[current_tuning_index];
    
    Serial.println("--- Current PID Gains ---");
    Serial.print("Kp: "); Serial.println(gains.Kp, 3);
    Serial.print("Ki: "); Serial.println(gains.Ki, 3);
    Serial.print("Kd: "); Serial.println(gains.Kd, 3);
    Serial.print("Max Integral: "); Serial.println(gains.max_integral, 1);
    Serial.println("------------------------");
}

void PID_TUNNER::print_operation_point_info() {
    if (current_tuning_index < 0) return;
    
    Serial.print("Tuning Operation Point #");
    Serial.print(current_tuning_index);
    Serial.print(" for speed ");
    Serial.print(current_tuning_speed);
    Serial.println(" rad/s");
    Serial.print("Speed range: [");
    Serial.print(gain_scheduler.operation_point_speed_range[current_tuning_index][0]);
    Serial.print(" - ");
    Serial.print(gain_scheduler.operation_point_speed_range[current_tuning_index][1]);
    Serial.println("]");
}

void PID_TUNNER::update() {
    if (tuning_mode && (millis() - last_print_time > PRINT_INTERVAL)) {
        print_current_gains();
        last_print_time = millis();
    }
}

void PID_TUNNER::exit_tuning_mode() {
    tuning_mode = false;
    current_tuning_index = -1;
    current_tuning_speed = 0.0;
    Serial.println("Exited PID tuning mode");
}

void PID_TUNNER::print_help() {
    Serial.println("\n=== PID TUNER HELP ===");
    Serial.println("op:speed    - Enter tuning mode for speed");
    Serial.println("kp:value    - Set Kp gain");
    Serial.println("ki:value    - Set Ki gain"); 
    Serial.println("kd:value    - Set Kd gain");
    Serial.println("max:value   - Set max integral");
    Serial.println("exit        - Exit tuning mode");
    Serial.println("help or ?   - Show this help");
    Serial.println("number      - Set target speed");
    Serial.println("=====================\n");
}