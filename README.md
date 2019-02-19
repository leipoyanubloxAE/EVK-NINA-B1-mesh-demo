# BLE mesh demo with EVK-NINA-Bx

### Overview
This is a demo that combines BLE light_switch mesh demo and MQTT subscriber example. The MQTT subsciber is merged with the client node in the light_switch mesh example.

Due to the limitation of softdevice only supporting one advertising set, GATT provisioning for mesh demo is removed. Provisioning has to be done via a provisioning device. 

MQTT subscriber test need to work together with a Raspberry Pi supporting Bluetooth 6LowPAN. Refer to Nordic SDK documentation on how to set up Raspberry Pi for Bluetooth 6LowPAN support. If the IPV6 tunning is also set up on Raspberry Pi, the MQTT subscriber is able to subscribe to cloud MQTT server, e.g. test.mosquitto.org. For simplicity, mosquitto server running on Raspberry Pi is used.

### Requirements
- EVK-NINA-B11x or EVK-NINA-B30x (minimally 3 is required. EVK-NINA-B11x and EVK-NINA-B30x can be used in the same demo)
- Segger J-Flash Lite
- Nordic nRF5 SDK v15.2.0
- Nordic mesh SDK 3.1.0
- Segger Embedded Studio 

### Running the demo
(Using EVK-NINA-B11x as example)
- Flash s132_nrf52_6.1.0_softdevice.hex + light_switch_provisioner_nrf52832_xxAA_s132_6.1.0.hex to EVK-NINA-B11x as provisioner.
- Flash s132_nrf52_6.1.0_softdevice.hex + subscriber_light_switch_client_pca10040_s132.hex to EVK-NINA-B11x as client.
- Keep both provisioner and client powered. Press SW1 button on Provisioner to start provisioning the client. The client should be provisioned automatically. Before provisioning, LED on client should be RED and BLUE. After provisioning is done, LED on client should be off. LED on provisioner should be RED if provisioning is successful. (Note that if LED on provisioner is BLUE, it means the provisioner has provisioned the client but is unable to set the model for the client. In that case, it is better erase the provisioner and client and start over.)
- Flash s132_nrf52_6.1.0_softdevice.hex + light_switch_server_nrf52832_xxAA_s132_6.1.0.hex to one ore more EVK-NINA-B11x as server. The server should be automatically provisioned if Provisioner is powered on and SW1 is pressed. The server LED will blink several times to signify the completion of provisioning.
- Note that the provisioning status and keys are stored in flash. They will be preserved between power cycles. 
- After all devices are provisioned, connect to the Client from the Pi through Bluetooth 6LowPAN and restart radvd. The Client will set an IP address. Refer to Nordic SDK documentation on how to pin EVK-NINA-B1 from PI. Once it's connected, Client LED will turn RED.
- Press SW1 button on the Client to connect the the MQTT server. MQTT server IP address is in the main.c. Once connection is successful, Client LED will turn BLUE and subscription will be done. Subscried topic is "led/state".
- Use MQTT app on a mobile phone to connect the the same MQTT server and publish 1 or 0 to the same topic "led/state". The message will be received by the Client and LEDs on Servers will be switched on and off based on the value.

Note that the provisioning info can be reset by pressing SW1 button during reset.

### Building the demo
To run the demo, you can use the precompiled firmware, or use Segger Embedded Studio to compile the firmware.

To compile the demo firmware and run the demo, please follow the steps:
1. Download and extract Nordic SDK v15.2.0. Rename it to nRF5_SDK_15.2.0_9412b96.
2. Download and extract nRF5 Mesh SDK v3.1.0. Rename it to nrf5SDKforMeshv310src and put it in the same directory as nRF5_SDK_15.2.0_9412b96
3. Replace the files in nRF5_SDK_15.2.0_9412b96 and nrf5SDKforMeshv310src by the files in nRF5_SDK-meshSDK-patches. 
4. Copy subscriber_light_switch_client folder to nRF5_SDK_15.2.0_9412b96/examples/iot/mqtt/lwip/ directory. Open the SES project and compile for subscriber_light_switch_client_pca10040_s132.hex.
5. Go to folder nrf5SDKforMeshv310src/examples/light_switch/provisioner/, open the SES project and compile for light_switch_provisioner_nrf52832_xxAA_s132_6.1.0.hex.
6. Go to folder nrf5SDKforMeshv310src/examples/light_switch/server, open the SES project and compile for light_switch_server_nrf52832_xxAA_s132_6.1.0.hex.


 





