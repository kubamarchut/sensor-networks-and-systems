
#include <Arduino.h>
#include "morslib.h"
#include "WirelessCommunication.h"
#include "Stopwatch.h"

#ifndef NODE_ADDR
#define NODE_ADDR 0x02            // ID tego węzła master
#endif

WirelessCommunication radio;
Stopwatch requestStopwatch(5000);
Stopwatch responseDebounce(0xFFFFFFFF);

uint16_t seq;

struct WirelessNode {
    uint8_t address;
    uint32_t ttl;
};
WirelessNode nodes[MAX_NODES];

morslib mymors(LED_BUILTIN, 200);

void receiveResponse() {
    WirelessPacket pkt;

    while (radio.receive(pkt)) {
        responseDebounce.reset(LORA_ROUND);
        if (pkt.type == PKT_RES) {
            for (int i = 0; i < MAX_NODES; i++) {
                if (nodes[i].address == pkt.trace[0]) {
                    nodes[i].ttl = millis() + 10000;
                    break;
                }

                if (nodes[i].address == 0) {
                    nodes[i].address = pkt.trace[0];
                    nodes[i].ttl = millis() + 10000;
                    break;
                }
            }
        }
    }
}

void sendRequest() {
    if (requestStopwatch.isTimeout()) {
        WirelessPacket pkt;
        pkt.trace[0] = NODE_ADDR;
        pkt.hopCount = 1;
        pkt.type = PKT_REQ;
        pkt.to = 0x07;
        pkt.seq = seq++;
        pkt.length = 0;
        memset(pkt.trace, 0, MAX_NODES);

        radio.send(pkt);
        requestStopwatch.reset();
    }
}

void setup() {
    mymors.begin();
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(9600);
    while (!Serial) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(1000);
        digitalWrite(LED_BUILTIN, LOW);
        delay(500);
    }
    Serial.print("W");
    Serial.print(NODE_ADDR - 1);
    Serial.println(" uruchomiony");

    if (!radio.begin(NODE_ADDR, ROLE_MASTER)) {
        Serial.println("Inicjalizacja radio nieudana");
        while (1);
    }

    Serial.println("Inicjalizacja radio udana");
    randomSeed(analogRead(A0));
    seq = random();
    mymors.queue('s');
}

void loop() {
    mymors.handle();
    radio.poll();
    receiveResponse();
    sendRequest();

    if (responseDebounce.isTimeout()) {
        uint32_t now = millis();

        Serial.println("Received PKT_RES");
        for (int i = 0; i < MAX_NODES; i++) {
            if (nodes[i].address == 0)
                break;

            uint8_t index = nodes[i].address-1;
            if (index <= 5) {
                Serial.print("\tW");
                Serial.print(index);
            } else {
                Serial.print("\tS");
                Serial.print(index);
            }

            Serial.print(" ");

            if (nodes[i].ttl <= now) {
                Serial.println(" online");
            } else {
                Serial.println(" offline !!");
            }
        }
        responseDebounce.reset(0xFFFFFFFF);
    }
}
