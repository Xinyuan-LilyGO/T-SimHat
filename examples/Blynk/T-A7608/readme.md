<h1 align = "center">T-A7670 Blynk_Console</h1>

## **[English](./README.MD) | [中文](./README_CN.MD)**


# Steps:
1. Need to install the following dependencies
     - [blynk](https://github.com/blynkkk/blynk-library)
     - [TinyGSM](https://github.com/vshymanskyy/TinyGSM)
     - [StreamDebugger](https://github.com/vshymanskyy/StreamDebugger)
     - [ArduinoHttpClient](https://github.com/ricemices/ArduinoHttpClient)
     - [ESP32-Arduino-CAN](https://github.com/miwagner/ESP32-Arduino-CAN)

2. Register and login[blynk.cloud](https://blynk.cloud/dashboard/login) 

3. Create a new template or use an existing template (The name can be customized)

![](../../../image/Blynk/1-Create_Template-T-SimHat.png)

4.   When the Template is ready, go to Search -> Devices - Create New Device

![](../../../image/Blynk/2-1-Create_device-T-SimHat.png)
![](../../../image/Blynk/2-2-Create_device-T-SimHat.png)

5. Configuration module

![](../../../image/Blynk/3-1-Configuration_module_T-SimHat.png)
![](../../../image/Blynk/3-2-Configuration_module_T-SimHat.png)
![](../../../image/Blynk/3-3-Configuration_module_T-SimHat.png)
![](../../../image/Blynk/3-4-Configuration_module_T-SimHat.png)
![](../../../image/Blynk/3-5-Configuration_module_T-SimHat.png)

6. Set up the Web dashboard

![](../../../image/Blynk/4-Dash_board.png)
![](../../../image/Blynk/4-1-Dash_board.png)
![](../../../image/Blynk/4-2-Dash_board.png)
![](../../../image/Blynk/4-3-Dash_board.png)
![](../../../image/Blynk/4-4-Dash_board.png)
![](../../../image/Blynk/4-5-Dash_board.png)

7. Replace the macro definition in the code

    #define BLYNK_TEMPLATE_ID ""
    #define BLYNK_DEVICE_NAME ""
    #define BLYNK_AUTH_TOKEN "";

 ![](../../../image/Blynk/5-Ready_code.png)

8. Upload the code and now you control the Relay and send data in the cloud
   
   ![](../../../image/Blynk/6.png)

9. Reference[Documentation](https://docs.blynk.io/en/)

