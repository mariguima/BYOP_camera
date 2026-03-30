# BYOP Camera

To run this sketch, download all files, especially the `DFRobot_AXP313A` library folder to power the camera. In the root directory, create a `wifi_config.h` file, and copy-paste the following code:
```
#define STASSID               "REPLACE_WITH_SSID"                   // WiFi SSID
#define STAPSK                "REPLACE_WITH_PASSWORD"               // WiFi password
```
Before running the project, replace the macros above with the actual SSID and password of your network. Note: when accessing the IP address for the live camera server from external devices (your PC, phone, etc.), ensure they are connected to the WiFi network configured above. 