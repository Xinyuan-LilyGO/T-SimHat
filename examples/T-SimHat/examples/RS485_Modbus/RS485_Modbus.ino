#define PIN_5V_EN    0
#define CAN_TX_PIN   22
#define CAN_RX_PIN   21
#define RS485_TX_PIN 19
#define RS485_RX_PIN 18
#define RS485_EN_PIN 23


#include <Arduino.h>
#include <HardwareSerial.h>
#include <ModbusMaster.h>
// instantiate ModbusMaster object
ModbusMaster node;
HardwareSerial Serial485(2);

void CANTask(void *prarm);

void setup()
{
    pinMode(RS485_EN_PIN, OUTPUT);
    digitalWrite(RS485_EN_PIN, HIGH);

    pinMode(PIN_5V_EN, OUTPUT);
    digitalWrite(PIN_5V_EN, HIGH);

    Serial.begin(9600);
    Serial485.begin(9600, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
    delay(5);
    node.begin(2, Serial485);

}

void loop()
{
    static uint32_t i;
    uint8_t j, result;
    uint16_t data[6];

    i++;
    // set word 0 of TX buffer to least-significant word of counter (bits 15..0)
    node.setTransmitBuffer(0, lowWord(i));

    // set word 1 of TX buffer to most-significant word of counter (bits 31..16)
    node.setTransmitBuffer(1, highWord(i));

    // slave: write TX buffer to (2) 16-bit registers starting at register 0
    result = node.writeMultipleRegisters(0, 2);

    // slave: read (6) 16-bit registers starting at register 2 to RX buffer
    result = node.readHoldingRegisters(2, 6);

    // do something with data if read is successful
    if (result == node.ku8MBSuccess) {
        for (j = 0; j < 6; j++) {
            data[j] = node.getResponseBuffer(j);
        }
    }
}
