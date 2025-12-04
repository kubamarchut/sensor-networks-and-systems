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

void bb_print_frame_compact(const bb_sensor_frame& frame) {
    Serial.print("Node=0x");
    Serial.print(frame.node_addr, HEX);

    Serial.print(" Sensor=0x");
    Serial.print(frame.sensor_addr, HEX);

    Serial.print(" Seq=");
    Serial.print(frame.seq);

    Serial.print(" Flags=0b");
    Serial.print(frame.flags, BIN);

    Serial.print(" Regs(");
    Serial.print(frame.regs_len);
    Serial.print(")=");

    // Print only real registers on a single line
    for (size_t i = 0; i < frame.regs_len; i++) {
        Serial.print("[0x");
        Serial.print(frame.regs[i].addr, HEX);
        Serial.print(":");
        Serial.print(frame.regs[i].data);
        Serial.print("]");  
        if (i < frame.regs_len - 1) Serial.print(","); // comma-separate
    }

    Serial.println(); // end the line
}