
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
#include <Adafruit_INA219.h>
Adafruit_INA219 ina219_1(0x40);
Adafruit_INA219 ina219_2(0x41);

static uint32_t  i = 0 ;
float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
float power_mW = 0;


void setup()
{
    Serial.begin(115200); // Set console baud rate
    SerialAT.begin(115200, SERIAL_8N1, PIN_RX, PIN_TX);
    delay(100);

    pinMode(RELAY_1, OUTPUT);
    pinMode(RELAY_2, OUTPUT);
    pinMode(RELAY_3, OUTPUT);

    Wire.begin(IIC_SDA, IIC_SCL);

    // Initialize the INA219.
    // By default the initialization will use the largest range (32V, 2A).  However
    // you can call a setCalibration function to change this range (see comments).
    if (!ina219_1.begin()) {
        Serial.println("Failed to find ina219_1 chip    没有找到ina219_1芯片");
        while (1) {
            delay(10);
        }
    }
    delay(1000);
    if (!ina219_2.begin()) {
        Serial.println("Failed to find ina219_2 chip  没有找到ina219_2芯片");
        while (1) {
            delay(10);
        }
    }
    // To use a slightly lower 32V, 1A range (higher precision on amps):
    // ina2191.setCalibration_32V_1A();
    // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
    //ina219.setCalibrat/ion_16V_400mA();

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
    shuntvoltage = ina219_1.getShuntVoltage_mV();
    busvoltage = ina219_1.getBusVoltage_V();
    current_mA = ina219_1.getCurrent_mA();
    power_mW = ina219_1.getPower_mW();
    loadvoltage = busvoltage + (shuntvoltage / 1000);

    Serial.println("/******************************ina219_1******************************/");
    Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
    Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
    Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
    Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
    Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
    Serial.println("");


    shuntvoltage = ina219_2.getShuntVoltage_mV();
    busvoltage = ina219_2.getBusVoltage_V();
    current_mA = ina219_2.getCurrent_mA();
    power_mW = ina219_2.getPower_mW();
    loadvoltage = busvoltage + (shuntvoltage / 1000);
    Serial.println("/******************************ina219_2******************************/");
    Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
    Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
    Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
    Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
    Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
    Serial.println("");

    delay(2000);

}
