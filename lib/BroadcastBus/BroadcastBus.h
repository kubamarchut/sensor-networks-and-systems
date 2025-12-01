#ifndef BROADCASTBUS_H
#define BROADCASTBUS_H

#include <Arduino.h>

#define BB_MAX_REGISTERS (8)
#define BB_MAX_SENSORS (88)

#define BB_MASK_START  (0x90)
#define BB_MASK_STOP   (0xA0)
#define BB_MASK_REQ    (0x03)
#define BB_MASK_RES    (0x0A)
#define BB_MASK_FIN    (0x0D)

#define BB_RECV_TIMEOUT (400) // ms
#define BB_ECHO_TIMEOUT (50) // ms
#define BB_DELAY_MIN (500) //ms
#define BB_DELAY_MAX (1000) //ms

typedef struct {
    uint8_t addr;
    uint8_t data;
} bb_sensor_register;

typedef struct {
    uint8_t node_addr;
    uint8_t sensor_addr;
    uint8_t seq;
    uint8_t flags;
    uint8_t regs_len;
    bb_sensor_register regs[BB_MAX_REGISTERS];
} bb_sensor_frame;


#endif // BROADCASTBUS_H