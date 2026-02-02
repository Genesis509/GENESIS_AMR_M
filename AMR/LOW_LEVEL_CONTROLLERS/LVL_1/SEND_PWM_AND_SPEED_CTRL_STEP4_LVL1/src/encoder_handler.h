#ifndef ENCODER_HANDLER_H
#define ENCODER_HANDLER_H

#include <Arduino.h>
#include "wheel_speel_control.h"
#define NUM_ENCODERS 4
#define OVERFLOW_FACTOR 0.9

typedef struct EncoderCount_t {
  volatile long& frontLeftCount;
  volatile long& frontRightCount ;
  volatile long& rearLeftCount ;
  volatile long& rearRightCount;
} EncoderCount_t;

typedef enum EncoderName_t {
  FRONT_LEFT=0,
  FRONT_RIGHT,
  REAR_LEFT,
  REAR_RIGHT
} EncoderName_t;

typedef struct EncoderCount_Handler_t {
  static const short COUNT_SAMPLE_SIZE = 5;
  volatile long Count_samples[COUNT_SAMPLE_SIZE] ;
  long processedCount = 0;
  int current_index = 0; 
} EncoderCount_Handler_t;

/* //Yes using an array would make simpler code ,
// but I prefer to use structs for better readability and maintainability
typedef struct EncodersCount_Handler_t {
  EncoderCount_Handler_t frontLeft;
  EncoderCount_Handler_t frontRight;
  EncoderCount_Handler_t rearLeft;
  EncoderCount_Handler_t rearRight;
}EncodersCount_Handler_t;  */

class EncoderHandler {
  private:
  
    volatile long* EncoderCounts_raw_ptrs[NUM_ENCODERS] = { NULL }; // Pointer to encoderCount variable

    EncoderCount_Handler_t encoderCounstHandler [NUM_ENCODERS];

    static const long MAX_ENCODER_COUNT = LONG_MAX * OVERFLOW_FACTOR;
    static const long MIN_ENCODER_COUNT = LONG_MIN * OVERFLOW_FACTOR;
    void reset_sample_buffer(EncoderCount_Handler_t& handler);

  public:
   EncoderHandler(volatile long* frontLeft, volatile long* frontRight,
   volatile long* rearLeft, volatile long* rearRight);
   void update_EncoderCount_Samples_and_raw_average();
   void average_encoderCount();
   void check_raw_encoders_count_and_reset();
   long* get_EncoderCount_ptr(EncoderName_t encoderName) {
     return &(encoderCounstHandler[encoderName].processedCount);
   }
};



#endif // ENCODER_HANDLER_H