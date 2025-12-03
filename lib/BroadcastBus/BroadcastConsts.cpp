#include "BroadcastConsts.h"

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