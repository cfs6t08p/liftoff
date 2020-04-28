#include <stdlib.h>
#include <string.h>

#include "xc.h"

#include "ign_api.h"

#define LIFTOFF_START 0
#define LIFTOFF_GET 1
#define LIFTOFF_DATA 2
#define LIFTOFF_END 3

int16_t instrumentation[360 * 9];
uint16_t instrumentation_index;
uint16_t instrumentation_countdown;

static void instrumentation_start(uint16_t start) {
  instrumentation_index = 0;
  instrumentation_countdown = start;
}

static void instrumentation_put(int16_t value) {
  if(instrumentation_countdown == 0 && instrumentation_index < sizeof(instrumentation) / sizeof(*instrumentation)) {
    instrumentation[instrumentation_index++] = value;
  }
}

static void instrumentation_get() {
  static uint8_t packet_buffer[19];
  
  uint16_t read_index = 0;
  
  while(read_index < instrumentation_index) {
    packet_buffer[0] = LIFTOFF_DATA;
    
    memcpy(&packet_buffer[1], &instrumentation[read_index], 18);
    
    read_index += 9;
    
    IGN->send_packet(packet_buffer, sizeof(packet_buffer));
    
    uint16_t delay = 5000;
    
    while(delay--) IGN->idle();
  }
  
  packet_buffer[0] = LIFTOFF_END;
  
  IGN->send_packet(packet_buffer, 1);
}

#define CTL_STATE_IDLE 0
#define CTL_STATE_CW 1
#define CTL_STATE_CCW 2

int16_t control_integral;

uint8_t control_state;
uint8_t target_speed;
int16_t target_position;

void ign_handler(uint16_t type, void *data, uint16_t len) {
  uint8_t *packet = data;
  
  if(type == IGN_EVENT_FL13) {
    uint8_t position = packet[0];
    uint8_t speed = packet[1];
    
    if(instrumentation_countdown) {
      instrumentation_countdown--;
    }
    
    target_position = 10 + (position * 58) / 10;
    target_speed = (speed * 8) / 10;
    
    control_integral = 0;
    
    int16_t current_position = IGN->get_encoder_count();
    
    int16_t distance = abs(target_position - current_position);
    int16_t max_speed = distance / 5;
    
    if(target_speed > max_speed) {
      target_speed = max_speed;
    }
    
    if(target_position > current_position) {
      IGN->set_led_constant(IGN_PLED_G, 0);
      IGN->set_led_constant(IGN_PLED_R, 250);
      control_state = CTL_STATE_CW;
      IGN->motor_cw();
    } else if(target_position < current_position) {
      IGN->set_led_constant(IGN_PLED_G, 250);
      IGN->set_led_constant(IGN_PLED_R, 0);
      control_state = CTL_STATE_CCW;
      IGN->motor_ccw();
    } else {
      IGN->set_led_constant(IGN_PLED_G, 0);
      IGN->set_led_constant(IGN_PLED_R, 0);
      control_state = CTL_STATE_IDLE;
      IGN->motor_stop();
    }
  } else if(type == IGN_EVENT_DATA) {
    uint8_t cmd = ((uint8_t *)data)[0];
    
    if(cmd == LIFTOFF_START) {
      instrumentation_start(*((uint16_t *)&packet[1]));
    } else if(cmd == LIFTOFF_GET) {
      instrumentation_get();
    }
  }
}

uint32_t last_ctl_tick;

int main(void) {
  CORCONbits.PSV = 1;
  PSVPAG = 0;
  
  IGN->set_handler(ign_handler);
  
  while(1) {
    uint32_t current_tick = IGN->get_tick_count();
    
    if(current_tick - last_ctl_tick >= 2) {
      static int16_t filtered_speed;
      static int16_t last_position;
      int16_t current_position = IGN->get_encoder_count();
      int16_t measured_speed = abs(current_position - last_position) * 10;
      
      filtered_speed = (filtered_speed * 4 + measured_speed) / 5;
      int16_t error = target_speed - filtered_speed;
      
      if(control_state == CTL_STATE_CW && current_position + filtered_speed > target_position) {
        IGN->set_led_constant(IGN_PLED_G, 0);
        IGN->set_led_constant(IGN_PLED_R, 0);
        control_state = CTL_STATE_IDLE;
        IGN->motor_brake();
      } else if(control_state == CTL_STATE_CCW && current_position - filtered_speed < target_position) {
        IGN->set_led_constant(IGN_PLED_G, 0);
        IGN->set_led_constant(IGN_PLED_R, 0);
        control_state = CTL_STATE_IDLE;
        IGN->motor_brake();
      } else if(control_state != CTL_STATE_IDLE) {
        uint16_t control_p = (error * 6) / 10;
        
        control_integral += (error * 6) / 10;
        
        if(control_integral > 1000) {
          control_integral = 1000;
        }
        
        if(control_integral < -1000) {
          control_integral = -1000;
        }
        
        int16_t motor_speed = control_p + control_integral / 10;
        
        if(motor_speed < 0) {
          motor_speed = 0;
        }
        
        if(motor_speed > 99) {
          motor_speed = 99;
        }
        
        instrumentation_put(current_position);
        instrumentation_put(error);
        instrumentation_put(control_integral);
        
        IGN->motor_set_speed((motor_speed * 256) / 25);
      }
      
      last_position = current_position;
      last_ctl_tick = current_tick;
      
      // show current position on red LED
      uint8_t position_led = (current_position * 5) / 12; // 250 / 600
      
      if(current_position < 0) {
        IGN->set_led_constant(IGN_BLED_R, 0);
      } else if(current_position > 600) {
        IGN->set_led_constant(IGN_BLED_R, 250);
      } else {
        IGN->set_led_constant(IGN_BLED_R, position_led);
      }
    }
    
    IGN->idle();
  }
}
