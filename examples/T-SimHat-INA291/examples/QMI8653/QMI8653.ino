
#define SerialAT Serial1
#define PIN_DTR     25
#define PIN_TX      27
#define PIN_RX      26
#define PWR_PIN     4
#define RELAY_1     18
#define RELAY_2     19
#define RELAY_3     32
#define IIC_SDA 21
#define IIC_SCL 22

#include <Arduino.h>
#include <Wire.h>
#include <QMI8658.h>

QMI8653 qmi8653;

void setup()
{
    Serial.begin(115200); // Set console baud rate
    SerialAT.begin(115200, SERIAL_8N1, PIN_RX, PIN_TX);
    delay(100);

    pinMode(RELAY_1, OUTPUT);
    pinMode(RELAY_2, OUTPUT);
    pinMode(RELAY_3, OUTPUT);
    Serial.println(F("DHTxx test!"));

    Wire.begin(IIC_SDA, IIC_SCL);

    if (qmi8653.begin() == false) {
        Serial.println(F("qmi8653 false! 没有找到qmi8653芯片 "));
        while (1);
    }
    Serial.println("Measuring voltage and current with INA219 ...");

    digitalWrite(RELAY_1, LOW);
    digitalWrite(RELAY_2, LOW);
    digitalWrite(RELAY_3, LOW);

    digitalWrite(RELAY_1, HIGH);
    delay(500);
    digitalWrite(RELAY_2, HIGH);
    delay(500);
    digitalWrite(RELAY_3, HIGH);
    delay(500);
    digitalWrite(RELAY_1, LOW);
    delay(500);
    digitalWrite(RELAY_2, LOW);
    delay(500);
    digitalWrite(RELAY_3, LOW);
    delay(500);


}

void loop()
{
    uint16_t  a_x, a_y, a_z, g_x, g_y, g_z;
    qmi8653.readAcceleration(&a_x, &a_y, &a_z);
    qmi8653.readAngular(&g_x, &g_y, &g_z);
    Serial.print(" a_x : ");
    Serial.print(a_x);
    Serial.print(" a_y : ");
    Serial.print(a_y);
    Serial.print(" a_z : ");
    Serial.println(a_z );
    Serial.print(" g_x : ");
    Serial.print(g_x);
    Serial.print(" g_y : ");
    Serial.print(g_y);
    Serial.print(" g_z : ");
    Serial.println(g_z );
    vTaskDelay(500);

}
