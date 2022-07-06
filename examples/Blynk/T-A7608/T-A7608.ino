#define BLYNK_TEMPLATE_ID ""
#define BLYNK_DEVICE_NAME ""
#define BLYNK_AUTH_TOKEN ""
#define TINY_GSM_MODEM_SIM7600  //The AT instruction of A7670 is compatible with SIM7600 
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb
#define SerialAT Serial1

// See all AT commands, if wanted
// #define DUMP_AT_COMMANDS

#define A7608_UART_BAUD    115200
#define RS485_UART_BAUD    115200
#define A7608_TX_PIN       26
#define A7608_RX_PIN       27
#define A7608_PWR_PIN      4
#define A7608_RI_PIN       33
#define A7608_PIN_DTR      25
#define A7608_RESET_PIN    5

#define BAT_ADC      35
#define BAT_EN       12
#define SD_MISO      2
#define SD_MOSI      15
#define SD_SCLK      14
#define SD_CS        13

#define PIN_5V_EN    0
#define RELAY_PIN    32

#define RS485_EN_PIN 23
#define RS485_TX_PIN 19
#define RS485_RX_PIN 18

#define CAN_TX_PIN   22
#define CAN_RX_PIN   21

#include <Arduino.h>
#include <HardwareSerial.h>
#include <ESP32CAN.h>
#include <SPI.h>
#include <TinyGsmClient.h>
#include <BlynkSimpleTinyGSM.h>

BlynkTimer timer;

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = BLYNK_AUTH_TOKEN;

// Your GPRS credentials
// Leave empty, if missing user or pass
char apn[]  = "YourAPN";
char user[] = "";
char pass[] = "";


HardwareSerial Serial485(2);

CAN_device_t CAN_cfg;             // CAN Config
unsigned long previousMillis = 0; // will store last time a CAN Message was send
const int interval = 1000;        // interval at which send CAN Messages (milliseconds)
const int rx_queue_size = 10;     // Receive Queue size


#ifdef DUMP_AT_COMMANDS  // if enabled it requires the streamDebugger lib
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, Serial);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

static TaskHandle_t AppTaskCreate_Handle = NULL;
static TaskHandle_t RS485_Task_Handle = NULL;
static TaskHandle_t CAN_Task_Handle = NULL;
bool can_send_flag = false;
String can_send_str;
bool reply = false;
int rs485_rx_str = 0;

void A7608modem_on();
void CAN_Task(void *prarm);
void RS485_Task( void *pvParameters );

BLYNK_WRITE(V0)
{
    if (param.asInt() == 1) {
        Serial.println("OFF");
        digitalWrite(RELAY_PIN, HIGH);
        Blynk.logEvent("RELAY STATE", "OFF");//Sending Events
    } else {
        Serial.println("ON");
        digitalWrite(RELAY_PIN, LOW);
        Blynk.logEvent("RELAY STATE", "ON");//Sending Events
        //to send the value to terminal
//max length for the 1 terminal message is 255 chars

    }
}

//Syncing the output state with the app at startup
BLYNK_CONNECTED()
{
    Blynk.syncVirtual(V0);  // will cause BLYNK_WRITE(V0) to be executed
}

//to get the value from the terminal text input

BLYNK_WRITE(V1)
{
    String string = param.asStr();
    Serial.print("RS485_Send:");
    Serial.println(string);
    char RS485_str[255];
    strcpy(RS485_str, string.c_str());
    Serial485.write(RS485_str);
}

//to get the value from the terminal text input
BLYNK_WRITE(V2)
{
    can_send_str = param.asStr();
    can_send_flag = true;
}

void setup()
{
    Serial.begin(115200);
    Serial485.begin(RS485_UART_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
    delay(5);
    A7608modem_on();

    pinMode(RS485_EN_PIN, OUTPUT);
    digitalWrite(RS485_EN_PIN, HIGH);

    pinMode(PIN_5V_EN, OUTPUT);
    digitalWrite(PIN_5V_EN, HIGH);

    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);


    // Restart takes quite some time
    // To skip it, call init() instead of restart()
    Serial.println("Initializing modem...");
    if (!modem.init()) {
        Serial.println("Failed to restart modem, delaying 10s and retrying");

    }

    Serial.println("...");
    Blynk.begin(auth, modem, apn, user, pass);
    // Setup a function to be called every second
    // timer.setInterval(2000L, sendSensor);
    Serial.println("Blynk.begin done");

    xTaskCreatePinnedToCore(
        Task_Create
        ,  "Task_Create"   // A name just for humans
        , 4096   // This stack size can be checked & adjusted by reading the Stack Highwater
        ,  NULL
        ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,  &AppTaskCreate_Handle
        ,  ARDUINO_RUNNING_CORE);
}

void loop()
{
    Blynk.run();
    timer.run();
    vTaskDelay(10);

}


void A7608modem_on()
{
    SerialAT.begin(A7608_UART_BAUD, SERIAL_8N1, A7608_RX_PIN, A7608_TX_PIN);
    delay(100);

    pinMode(BAT_EN, OUTPUT);
    digitalWrite(BAT_EN, HIGH);

    //A7608 Reset
    pinMode(A7608_RESET_PIN, OUTPUT);
    digitalWrite(A7608_RESET_PIN, LOW);
    delay(100);
    digitalWrite(A7608_RESET_PIN, HIGH);
    delay(1000);
    digitalWrite(A7608_RESET_PIN, LOW);

    pinMode(A7608_PWR_PIN, OUTPUT);
    digitalWrite(A7608_PWR_PIN, LOW);
    delay(100);
    digitalWrite(A7608_PWR_PIN, HIGH);
    delay(1000);
    digitalWrite(A7608_PWR_PIN, LOW);


    int i = 10;
    Serial.println("\nTesting Modem Response...\n");
    Serial.println("****");
    while (i) {
        SerialAT.println("AT");
        delay(500);
        if (SerialAT.available()) {
            String r = SerialAT.readString();
            Serial.println(r);
            if ( r.indexOf("OK") >= 0 ) {
                reply = true;
                break;;
            }
        }
        delay(500);
        i--;
    }
    Serial.println("****\n");
    if (reply) {
        Serial.println("A7608_Mode normal");
    } else {
        Serial.println("A7608_Mode abnormal");
    }

}

//Task_Create
static void Task_Create(void *pvParameters)
{
    (void) pvParameters;

    BaseType_t xReturn = pdPASS;


    xReturn = xTaskCreatePinnedToCore(
                  RS485_Task
                  ,  "RS485_Task"
                  ,  4096  // Stack size
                  ,  NULL
                  ,  1  // Priority
                  ,  &RS485_Task_Handle
                  ,  ARDUINO_RUNNING_CORE);
    if (xReturn == pdPASS) {
        Serial.println("Create RS485_Task  succeeded!");
    }



    xReturn = xTaskCreatePinnedToCore(
                  CAN_Task
                  ,  "CAN_Task"
                  ,  4096  // Stack size
                  ,  NULL
                  ,  1  // Priority
                  ,  &CAN_Task_Handle
                  ,  ARDUINO_RUNNING_CORE);
    if (xReturn == pdPASS) {
        Serial.println("Create CAN_Task  succeeded!");
    }



    vTaskDelete(AppTaskCreate_Handle); //Delete AppTaskCreate

    for (;;) {


    }

}


void RS485_Task(void *pvParameters)
{

    for (; ; ) {

        while (Serial.available()) {
            Serial485.write(Serial.read());
        }
        while ( Serial485.available()) {
            Serial.write(Serial485.read());
            // rs485_rx_str = Serial485.read();
            // Serial.print((char)rs485_rx_str);
        }
        vTaskDelay(10);
    }


}

void CAN_Task(void *pvParameters)
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
        if (can_send_flag) {
            CAN_frame_t tx_frame;
            tx_frame.FIR.B.FF = CAN_frame_std;
            tx_frame.MsgID = 0x001;
            tx_frame.FIR.B.DLC = 8;

            for (size_t i = 0; i < 8; i++) {
                tx_frame.data.u8[i] = can_send_str[i];
            }
            ESP32Can.CANWriteFrame(&tx_frame);
            can_send_flag = false;
            can_send_str = "";
        }

        // Send CAN Message
        // if (currentMillis - previousMillis >= interval) {
        //     previousMillis = currentMillis;
        //     CAN_frame_t tx_frame;
        //     tx_frame.FIR.B.FF = CAN_frame_std;
        //     tx_frame.MsgID = 0x001;
        //     tx_frame.FIR.B.DLC = 8;
        //     tx_frame.data.u8[0] = 0x00;
        //     tx_frame.data.u8[1] = 0x01;
        //     tx_frame.data.u8[2] = 0x02;
        //     tx_frame.data.u8[3] = 0x03;
        //     tx_frame.data.u8[4] = 0x04;
        //     tx_frame.data.u8[5] = 0x05;
        //     tx_frame.data.u8[6] = 0x06;
        //     tx_frame.data.u8[7] = 0x07;

        //     ESP32Can.CANWriteFrame(&tx_frame);
        // }
        vTaskDelay(10);
    }
}
