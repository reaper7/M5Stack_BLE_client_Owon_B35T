# ESP32 M5Stack BLE client for OWON B35T meter

Uses Bluetooth Low Energy (BLE) client to allow the M5Stack to remotely display the screen output and to control the functions of an Owon B35T multi-meter.

**Please note:**

Current changes in arduino-esp32 git (after 04.03.2018) cause the final sketch size to increase beyond the permissible size ! (https://github.com/espressif/arduino-esp32/issues/1194),

One possible solution is to increase the application partition size, as discussed in this excellent blog:
http://desire.giesecke.tk/index.php/2018/01/30/change-partition-size/

![remote meter 1](https://github.com/reaper7/M5Stack_BLE_client_Owon_B35T/blob/master/docs/m5stack.jpg)

![remote meter 2](https://github.com/reaper7/M5Stack_BLE_client_Owon_B35T/blob/master/docs/m5stack_meter.jpg)

**2018 Reaper7**

I publish this sketch as it is. I do not have free time to clean it up and polish.

If you like it, buy me a beer: (https://www.paypal.me/reaper7md)
