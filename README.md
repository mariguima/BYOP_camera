# BYOP Camera

## How to use

- To run this sketch, download all files, especially the `DFRobot_AXP313A` library folder to power the camera 
- Make sure the project folder is named `BYOP_Camera`, exactly like the .ino file
- As of now, the code is only guaranteed to work with `esp32` library in version 1.3.1 (change it in Library Manager)
- In the root directory, create a `wifi_config.h` file, and copy-paste the following code (this file will hold your SSID and password configurations):
```
#define STASSID               "REPLACE_WITH_SSID"                   // WiFi SSID
#define STAPSK                "REPLACE_WITH_PASSWORD"               // WiFi password
```
- Before running the project, replace the macros above with the actual SSID and password of your network. 
- Note: when accessing the IP address for the live camera server from external devices (your PC, phone, etc.), ensure they are connected to the WiFi network configured above. 
