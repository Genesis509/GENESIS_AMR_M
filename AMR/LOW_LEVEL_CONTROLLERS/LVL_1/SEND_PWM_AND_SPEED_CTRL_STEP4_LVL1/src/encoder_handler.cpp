#include "encoder_handler.h"

EncoderHandler::EncoderHandler(volatile long* frontLeft, volatile long* frontRight,
  volatile long* rearLeft, volatile long* rearRight) {
  
  EncoderCounts_raw_ptrs[FRONT_LEFT] = frontLeft;
  EncoderCounts_raw_ptrs[FRONT_RIGHT] = frontRight;
  EncoderCounts_raw_ptrs[REAR_LEFT] = rearLeft;
  EncoderCounts_raw_ptrs[REAR_RIGHT] = rearRight;

  for(int i = 0; i < NUM_ENCODERS; i++) {
    encoderCounstHandler[i].current_index = 0;
    encoderCounstHandler[i].processedCount = 0;
    for(int j = 0; j < EncoderCount_Handler_t::COUNT_SAMPLE_SIZE; j++) {
      encoderCounstHandler[i].Count_samples[j] = 0;
    }
  }
}

void EncoderHandler::check_raw_encoders_count_and_reset()
{

  for(int i = 0; i < NUM_ENCODERS; i++) {
    if(*EncoderCounts_raw_ptrs[i] > MAX_ENCODER_COUNT || 
       *EncoderCounts_raw_ptrs[i] < MIN_ENCODER_COUNT)
      {
        *EncoderCounts_raw_ptrs[i] = 0; // Reset the count if it exceeds limits
        reset_sample_buffer(encoderCounstHandler[i]);

      }
  }
} 

void EncoderHandler::update_EncoderCount_Samples_and_raw_average()
{
  
  for(int i = 0; i < NUM_ENCODERS; i++) {
    
    encoderCounstHandler[i].Count_samples[encoderCounstHandler[i].current_index] = *EncoderCounts_raw_ptrs[i];
    
    
    if(encoderCounstHandler[i].current_index == EncoderCount_Handler_t::COUNT_SAMPLE_SIZE - 1)
    { 
      // Calculate average
      long sum = 0;
      for(int j = 0; j < EncoderCount_Handler_t::COUNT_SAMPLE_SIZE; j++) {
        sum += encoderCounstHandler[i].Count_samples[j];
      }
      encoderCounstHandler[i].processedCount = sum / EncoderCount_Handler_t::COUNT_SAMPLE_SIZE;
    }

    encoderCounstHandler[i].current_index = (encoderCounstHandler[i].current_index + 1) % EncoderCount_Handler_t::COUNT_SAMPLE_SIZE;
  }
  
}

void EncoderHandler::reset_sample_buffer(EncoderCount_Handler_t& handler) {
  for(int i = 0; i < EncoderCount_Handler_t::COUNT_SAMPLE_SIZE; i++) {
    handler.Count_samples[i] = 0;
  }
  handler.processedCount = 0;
  handler.current_index = 0;
}