#ifndef _IGN_API_H_
#define _IGN_API_H_

#include "xc.h"

#define IGN_VERSION 0x1671

#define IGN_BLED_R 0
#define IGN_BLED_B 1
#define IGN_PLED_G 2
#define IGN_PLED_R 3

#define IGN_EVENT_FL13 0
#define IGN_EVENT_DATA 1

typedef void (*ign_handler_t)(uint16_t type, void *data, uint16_t len);

struct ign_call_table {
  uint16_t version;
  uint32_t (*get_tick_count)(); // get number of ticks since start (1 tick == 1ms)
  int16_t (*get_encoder_count)(); // get encoder count (normal range: 10 - 540)
  void (*motor_stop)(); // stop driving the motor (coast to stop)
  void (*motor_brake)(); // stop driving the motor (active braking) 
  void (*motor_set_speed)(uint16_t speed); // set motor drive current (0-1023)
  void (*motor_cw)(); // start driving the motor in a clockwise direction
  void (*motor_ccw)(); // start driving the motor in a counter-clockwise direction
  void (*pwr_off)(); // turn off the device, does not return
  void (*set_led_constant)(uint16_t led, uint8_t intensity); // set LED to a constant intensity
  void (*set_led_pulsing)(uint16_t led, uint8_t rate); // pulse LED, higher rate == slower pulsing
  uint8_t (*is_connected)(); // returns true if a bluetooth connection is active, false otherwise
  void (*send_packet)(void *data, uint16_t len); // send a custom data packet (max len 19)
  uint8_t (*get_battery_level)(); // get battery level 0-99%
  uint16_t (*get_api_errors)(); // get number of reported API errors (invalid state or parameter)
  uint16_t (*get_ctl_errors)(); // get number of reported control errors (emergency brake)
  uint16_t (*get_motor_current)();
  void (*idle)(); // process events & other IGN tasks, must be called regularly
  void (*set_handler)(ign_handler_t handler); // set IGN event handler
};

extern const struct ign_call_table *IGN;

#endif
