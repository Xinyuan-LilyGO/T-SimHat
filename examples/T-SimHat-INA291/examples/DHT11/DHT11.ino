

#define SerialAT Serial1
#define PIN_DTR     25
#define PIN_TX      27
#define PIN_RX      26
#define PWR_PIN     4
#define RELAY_1     18
#define RELAY_2     19
#define RELAY_3     32
#define DHTPIN      23
#define DHTTYPE DHT11   // DHT 22  (AM2302)

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BusIO_Register.h>
#include "DHT.h"


DHT dht(DHTPIN, DHTTYPE);
float h;
float t;

void setup()
{
    Serial.begin(115200); // Set console baud rate
    SerialAT.begin(115200, SERIAL_8N1, PIN_RX, PIN_TX);
    delay(100);

    pinMode(RELAY_1, OUTPUT);
    pinMode(RELAY_2, OUTPUT);
    pinMode(RELAY_3, OUTPUT);
    pinMode(DHTPIN, OUTPUT);
    dht.begin();
    Serial.println(F("DHTxx test!"));
    h = dht.readHumidity();
    t = dht.readTemperature();

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
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    float h = dht.readHumidity();
    // Read temperature as Celsius
    float t = dht.readTemperature();

    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t)) {
        Serial.println("Failed to read from DHT sensor!");
        return;
    }
    Serial.print("Humidity: ");
    Serial.print(h);
    Serial.print(" %\t");
    Serial.print("Temperature: ");
    Serial.print(t);
    Serial.println(" *C ");
    delay(500);


}
