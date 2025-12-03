#ifndef BROADCAST_CONSTS
#define BROADCAST_CONSTS

#define BB_MAX_REGISTERS (8)
#define BB_MAX_SENSORS (8)

#define BB_MASK_START  (0x90)
#define BB_MASK_STOP   (0xA0)
#define BB_MASK_REQ    (0x03)
#define BB_MASK_SEN    (0x0A)
#define BB_MASK_FIN    (0x0D)

#define BB_BAUD (9600)
#define BB_RECV_TIMEOUT (400) // ms
#define BB_ECHO_TIMEOUT (50) // ms
#define BB_DELAY_MIN (500) //ms
#define BB_DELAY_MAX (1000) //ms
#define BB_DEBUG

typedef struct __attribute__((packed)) {
    uint8_t addr;
    uint8_t data;
} bb_sensor_register;

typedef struct __attribute__((packed)) {
    uint8_t node_addr;
    uint8_t sensor_addr;
    uint8_t seq;
    uint8_t flags;
    uint8_t regs_len;
    bb_sensor_register regs[BB_MAX_REGISTERS];
} bb_sensor_frame;

typedef struct __attribute__((packed)) {
    uint8_t cmd;
    size_t length;
    uint8_t data[128];
    bool hasCrc = false;
    uint8_t crc;
} bb_buffer;

#endif // BROADCAST_CONSTS