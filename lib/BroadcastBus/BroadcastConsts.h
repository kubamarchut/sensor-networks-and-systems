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


void bb_print_frame(bb_sensor_frame& frame) {
    Serial.print("\tNode address = 0x");
    Serial.println(frame.node_addr, HEX);
    Serial.print("\tSensor address = 0x");
    Serial.println(frame.sensor_addr, HEX);
    Serial.print("\tSeq = ");
    Serial.println(frame.seq);
    Serial.print("\tFlags = 0b");
    Serial.println(frame.flags, BIN);
    Serial.print("\tRegisters length =");
    Serial.println(frame.regs_len, DEC);
    for (size_t i = 0; i < BB_MAX_REGISTERS; i++) {
        Serial.print("\tRegister[");
        Serial.print(i, DEC);
        Serial.print("] = 0x");
        Serial.print(frame.regs[i].addr, HEX);
        Serial.print(" - ");
        Serial.println(frame.regs[i].data);
    }

}

#endif // BROADCAST_CONSTS