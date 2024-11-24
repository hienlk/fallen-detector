# Fall Detection Device and Mobile Application


The Fall Detection Device is a wearable device designed to detect falls using the MPU6050 accelerometer and gyroscope sensor. It leverages the jerk in acceleration data to identify fall events accurately.
The device connects to a mobile app via Bluetooth Low Energy (BLE) and incorporates a BLE server built using the NimBLE stack, designed to support pairing and bonding with passkey authentication for enhanced security.
In the event of a detected fall, the device activates a buzzer and LED to alert people nearby. If the device is already connected to the mobile app, the phone will send an SOS SMS to a pre-entered phone number.
The device features an event cancellation button, which generates an interrupt to deactivate the buzzer when it is active, allowing users to stop the alert manually. Additionally, it includes an emergency button that can be pressed if the MPU fails to detect a fall, enabling users to manually trigger a fall alert and notify others in critical situations.



## Features

- [x] **Fall Detection:** The device continuously monitors the user's movements using the MPU6050 sensor. A significant jerk in acceleration is detected to trigger a fall event.

- [x] **Buzzer and LED Alert:** When a fall is detected, a buzzer and LED are activated to alert nearby individuals about the incident.

- [x] **Event Cancellation Button:** The device is equipped with an event cancellation button. Pressing this button generates an interrupt, stopping the buzzer and preventing false alarms.

- [x] **Emergency Button:** If the user falls and the device fails to detect the event, they can press the emergency button to manually trigger a fall alert.

- [x] **BLE server:** The BLE server, built using NimBLE, facilitates data transmission between the device and the mobile app. It supports pairing, bonding, and passkey authentication for secure connections.

- [x] **Send SOS SMS:** In the event of a fall, the device sends a notification to the app via NimBLE. The app then sends an SOS SMS to pre-added emergency contacts, including the user’s location (Google Maps) to assist with immediate help.

- [x] **Mobile Application:** An Android mobile application connects to the fall detection device, sending SOS SMS messages and providing other functionalities for managing the device.

- [ ] **Find Device:** This feature allows users to locate the device by activating the buzzer through the mobile app.

- [ ] **Enable/Disable BLE Advertising Button:** This button lets users enable or disable BLE advertising, allowing them to connect the device to a new phone or prevent others from pairing with it.

## Hardware Components Used:
<p align="center">
<img src="https://github.com/hienlk/fallen-detector/blob/sp/res/falldetector_bb.png" height="500" width="500">
</p>
<br>

- **ESP32 C3 Core Board** 

- **MPU6050 Accelerometer and Gyroscope Sensor** 

- **Buzzer** 

- **LED** 

- **Event Cancellation Button** 

- **Emergency Button**

## Workflow
<p align="center">
<img src="https://github.com/hienlk/fallen-detector/blob/sp/res/uml.png" height="700" width="450">
</p>
<br>

The MPU6050 continuously collects motion and temperature data, which is processed by the ESP32.
If the calculated jerk exceeds a threshold:
is_fallen() returns True.
The Buzzer and LED are activated to alert nearby individuals.
If connected, the app sends an SOS SMS to emergency contacts.

If no fall is detected, is_fallen() returns False, no alerts are triggered, and the system continues its normal operation by processing data to monitor the user's movements in real-time.
The Event Cancellation Button can deactivate the buzzer and LED when pressed.
In case the MPU6050 fails to detect a fall, the Emergency Button allows the user to manually trigger a fall event, ensuring the alert is sent promptly.
The BLE GATT server, implemented using NimBLE, enables secure communication between the ESP32 and the mobile app. It supports pairing, bonding, and passkey authentication for data exchange.


## Getting Started

To use the Fall Detection Device, follow these steps:

1. Assemble the hardware components according to the schematic diagram provided.

2. Config NimBLE: In ESP-IDF ```python idf.py menuconfig → Component Config → Bluetooth → Bluetooth Host → NimBLE - BLE only```

3. Upload the provided firmware to the ESP32 using ESP-IDF.

4. Install mobile applicaion by apk.

5. Connect to BLE server of the device using app. Pairing and bonding it. The passkey is "123456".

6. Add emergency contacts in app.

7. Wear the device, and it will continuously monitor your movements.

8. In the event of a fall, the buzzer and LED will active, and SOS SMS messages will be sent to the emergency contacts.

