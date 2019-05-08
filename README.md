# BLE mesh demo with EVK-NINA-Bx and EVK-NINA-W10x

### Overview
This is a demo that combines BLE light_switch mesh demo and Wifi gateway. 

The light_switch client is connected to wifi module through UART, i.e. NINA-W10x. Provisioning of the BLE nodes are done via nRF mesh application on mobile phone. Every light_switch server will periodically update its on/off status. The status message will be received by the light_switch client and forwarded to NINA-W10 through UART. Then the message will be forwarded to cloud over MQTT publishing. The status then will be shown in an IoT dashboard. 

The IoT dashboard also allows controlling the on/off status of the light_switch servers, either by individual or group. The message flow is the in reverse order of status publishing.

The dashboard is available at http://ubx.ddns.net:1880/ui. 

Note that the dashboard only accept messages from WiFi gateway with certain device keys. The device key is LED-GW-0001000XXXXXX where XXXXXX refers to the last 3 bytes of MAC address in upper case. 

### Requirements
- EVK-NINA-B11x or EVK-NINA-B30x (minimally 3 is required. EVK-NINA-B11x and EVK-NINA-B30x can be used in the same demo)
- EVK-NINA-W10x x 1
- Jump wire x 3
- Segger J-Flash Lite
- Nordic nRF5 SDK v15.2.0
- Nordic mesh SDK 3.1.0
- Segger Embedded Studio 
- ESP32 Flash Download Tool
- Android phone

### Running the demo
(Using EVK-NINA-B11x as example)
- Flash s132_nrf52_6.1.0_softdevice.hex + light_switch_server_nrf52832_xxAA_s132_6.1.0.hex to one ore more EVK-NINA-B11x as server. 
- Flash s132_nrf52_6.1.0_softdevice.hex + light_switch_client_nrf52832_xxAA_s132_6.1.0.hex to EVK-NINA-B11x as client.
- Use nRF mesh mobile app to provision the servers and client. After provisionin is done, configure the server generic on/off server to bind the key, add publication address 0xCAFE, and subscrition address 0xC001 or 0xC002 based on whether the node ID is odd or even number. Configure the client generic on/off client to bind the app key, add subscrition address 0xCAFE.
- Use ESP32 flash download tool to flash the NINA-W10x image to EVK-NINA-W10x. A screenshot of the download tool with flashing parameter can be found in the light_switch_demo_hex/NINA-W10 directory. After pressing the START button, press RESET button on EVK-NINA-W10x while holding BOOT button. Alternatively, use jump wire to connect RESET to IO19 and IO0 to IO26 on EVK-NINA-W10x, updating will start automatically without pressing BOOT and RESET button.
- Use jump wire to connect the following pins between EVK-NINA-W10x and EVK-NINA-B11x:
  IO5  (J4 EVK-NINA-W10x) - 23  (J3 EVK-NINA-B11x)
  IO18 (J4 EVK-NINA-W10x) - 22  (J3 EVK-NINA-B11x)
  GND  (J4 EVK-NINA-W10x) - GND (J4 EVK-NINA-B11x)
- Install the app-wifi-sec1-debug-28-9-2018.apk in the repository (or download from https://github.com/espressif/esp-idf-provisioning-android/releases ) on Android phone. Use it to provision EVK-NINA-W10x with the SSID and password of the router.
- Use a browser to open url http://ubx.ddns.net:1880/ui . The LED status should be displayed. Control the LED by pressing the individual LED switch or group switch.

Note that the provisioning info can be reset by pressing SW1 button during reset.

### Building the demo

#### EVK-NINA-B1x:

To compile the NINA-Bx demo firmware, please follow the steps below:
1. Download and extract Nordic SDK v15.2.0. Rename it to nRF5_SDK_15.2.0_9412b96.
2. Download and extract nRF5 Mesh SDK v3.1.0. Rename it to nrf5SDKforMeshv310src and put it in the same directory as nRF5_SDK_15.2.0_9412b96
3. Replace the files in nRF5_SDK_15.2.0_9412b96 and nrf5SDKforMeshv310src by the files in nRF5_SDK-meshSDK-patches. 
4. Copy client and server folders to nrf5SDKforMeshv310src/examples/light_switch directory and replace the original ones. Open the SES projects and compile for light_switch_client_nrf52832_xxAA_s132_6.1.0.hex and light_switch_server_nrf52832_xxAA_s132_6.1.0.hex.

#### EVK-NINA-W10x:

To compile the NINA-W10x demo firmware, please follow the steps below:
1. Install the toolchain, get ESP-IDF and set up the patch to ESP-IDF according to instructions at https://docs.espressif.com/projects/esp-idf/en/stable/get-started/ 
2. Go to the mqtt-tcp directory, run "make app" to build the image.

Note that ESP-IDF compilation in Windows may be slow due to Windows secruity with many files and folders being created. Either exclude the mqtt-tcp directory from Windows security or run in Linux distribution, e.g. Ubuntu.

 





