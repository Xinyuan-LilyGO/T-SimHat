#include <Arduino.h>
#include "config.h"
#include <HardwareSerial.h>
#include <ESP32CAN.h>
#include <SPI.h>

HardwareSerial Serial485(2);

CAN_device_t CAN_cfg;             // CAN Config
unsigned long previousMillis = 0; // will store last time a CAN Message was send
const int interval = 1000;        // interval at which send CAN Messages (milliseconds)
const int rx_queue_size = 10;     // Receive Queue size

void CANTask(void *prarm);

void setup()
{
    pinMode(RS485_EN_PIN, OUTPUT);
    digitalWrite(RS485_EN_PIN, HIGH);

    pinMode(PIN_5V_EN, OUTPUT);
    digitalWrite(PIN_5V_EN, HIGH);

    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);


    Serial.begin(115200);
    Serial485.begin(115200, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
    delay(5);
    Serial.println("factory test");
    // put your setup code here, to run once:

    xTaskCreatePinnedToCore(CANTask, "CANTask", 1024 * 10, NULL, 3, NULL, 1);
}

void loop()
{
    while (Serial.available())
        Serial485.write(Serial.read());
    while (Serial485.available())
        Serial.write(Serial485.read());
    static uint32_t Millis;
    if (millis() - Millis > 1000) {
        digitalWrite(RELAY_PIN, !digitalRead(RELAY_PIN));
        Millis = millis();
    }
}

void CANTask(void *prarm)
{
    CAN_cfg.speed = CAN_SPEED_1000KBPS;
    CAN_cfg.tx_pin_id = (gpio_num_t)CAN_TX_PIN;
    CAN_cfg.rx_pin_id = (gpio_num_t)CAN_RX_PIN;
    CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
    // Init CAN Module
    ESP32Can.CANInit();

    Serial.print("CAN SPEED :");
    Serial.println(CAN_cfg.speed);

    while (1) {
        CAN_frame_t rx_frame;
        unsigned long currentMillis = millis();

        // Receive next CAN frame from queue
        if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) {

            if (rx_frame.FIR.B.FF == CAN_frame_std) {
                printf("New standard frame");
            } else {
                printf("New extended frame");
            }

            if (rx_frame.FIR.B.RTR == CAN_RTR) {
                printf(" RTR from 0x%08X, DLC %d\r\n", rx_frame.MsgID, rx_frame.FIR.B.DLC);
            } else {
                printf(" from 0x%08X, DLC %d, Data ", rx_frame.MsgID, rx_frame.FIR.B.DLC);
                for (int i = 0; i < rx_frame.FIR.B.DLC; i++) {
                    printf("0x%02X ", rx_frame.data.u8[i]);
                }
                printf("\n");
            }
        }
        // Send CAN Message
        if (currentMillis - previousMillis >= interval) {
            previousMillis = currentMillis;
            CAN_frame_t tx_frame;
            tx_frame.FIR.B.FF = CAN_frame_std;
            tx_frame.MsgID = 0x001;
            tx_frame.FIR.B.DLC = 8;
            tx_frame.data.u8[0] = 0x00;
            tx_frame.data.u8[1] = 0x01;
            tx_frame.data.u8[2] = 0x02;
            tx_frame.data.u8[3] = 0x03;
            tx_frame.data.u8[4] = 0x04;
            tx_frame.data.u8[5] = 0x05;
            tx_frame.data.u8[6] = 0x06;
            tx_frame.data.u8[7] = 0x07;

            ESP32Can.CANWriteFrame(&tx_frame);
        }
    }
}
