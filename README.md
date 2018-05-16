# ESP32 M5Stack BLE client for OWON B35T meter

Uses Bluetooth Low Energy (BLE) client to allow the M5Stack to remotely display the screen output and to control the functions of an Owon B35T multi-meter.

**Please note:**

Current arduino-esp32 git (14.05.2018) allows you to choose partiton scheme from Arduino IDE->Tools->Partition Scheme,
it is important to change this from default to minimal_spiffs or no_ota,
because the size of the binary file exceeds the allowable value in the default partition.

![remote meter 1](https://github.com/reaper7/M5Stack_BLE_client_Owon_B35T/blob/master/docs/m5stack.jpg)

![remote meter 2](https://github.com/reaper7/M5Stack_BLE_client_Owon_B35T/blob/master/docs/m5stack_meter.jpg)

**2018 Reaper7**

I publish this sketch as it is. I do not have free time to clean it up and polish.

If you like it, buy me a beer: (https://www.paypal.me/reaper7md)
