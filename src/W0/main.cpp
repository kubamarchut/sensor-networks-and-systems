
#include <Arduino.h>
#include "morslib.h"
#include "WirelessCommunication.h"
#include "Stopwatch.h"

#ifndef NODE_ADDR
#define NODE_ADDR 0x01            // ID tego węzła master
#endif

Indicator indicator(9, 10, 11);
WirelessCommunication radio;
Stopwatch requestStopwatch(5000);
Stopwatch onlineStopwatch(10000);

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
        if (pkt.type == PKT_RES) {
            for (int i = 0; i < MAX_NODES; i++) {
                uint8_t address = pkt.trace[i];

                if (address == 0)
                    break;

                for (int j = 0; j < MAX_NODES; j++) {
                    if (nodes[j].address == address) {
                        nodes[j].ttl = millis() + 10000;
                        break;
                    }

                    if (nodes[j].address == 0) {
                        nodes[j].address = address;
                        nodes[j].ttl = millis() + 10000;
                        break;
                    }
                }
            }
        }

        WirelessCommunication::dumpPacket(pkt);
    }
}

void sendRequest() {
    if (requestStopwatch.isTimeout()) {
        WirelessPacket pkt;
        memset(pkt.trace, 0, MAX_NODES);
        memset(pkt.initialTrace, 0, MAX_NODES);
        memset(pkt.payload, 0, 8);
        pkt.trace[0] = NODE_ADDR;
        pkt.hopCount = 1;
        pkt.type = PKT_REQ;
        pkt.to = 0x07;
        pkt.seq = seq++;
        pkt.length = 0;

        radio.send(pkt);
        requestStopwatch.reset();
    }
}

void setup() {
    mymors.begin();
    randomSeed(analogRead(A0));
    indicator.begin();
    indicator.setColor(Indicator::RED);
    delay(1000);
    indicator.setColor(Indicator::GREEN);
    delay(1000);
    indicator.setColor(Indicator::BLUE);
    delay(1000);
    indicator.setColor(Indicator::RED);

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

    if (!radio.begin(NODE_ADDR, ROLE_MASTER, &indicator)) {
        Serial.println("Inicjalizacja radio nieudana");
        while (1);
    }

    Serial.println("Inicjalizacja radio udana");
    seq = random();
    mymors.queue('s');
}

void loop() {
    mymors.handle();
    radio.poll();
    receiveResponse();
    sendRequest();

    if (onlineStopwatch.isTimeout()) {
        uint32_t now = millis();

        Serial.println("Received PKT_RES");
        for (int i = 0; i < MAX_NODES; i++) {
            if (nodes[i].address == 0)
                break;

            uint8_t index = nodes[i].address-1;
            if (index <= 5) {
                Serial.print(" W");
                Serial.print(index);
            } else {
                Serial.print(" S");
                Serial.print(index-5);
            }

            Serial.print("-");

            if (nodes[i].ttl >= now) {
                Serial.print("on");
            } else {
                Serial.print("off");
            }
        }
        Serial.println();
        onlineStopwatch.reset();
    }


}
