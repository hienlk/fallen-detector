# Fall Detection Device and Mobile Application


The Fall Detection Device is a wearable device designed to detect falls using the MPU6050 accelerometer and gyroscope sensor. 
It utilizes the jerk in accelerations to determine if a fall event has occurred. 
The device is connect with mobile app via BLE. 
In case of a fall, the device activates a buzzer and LED to alert people in the vicinity, if it is already connected with mobile app the phone will send SOS SMS to phone number which has been entered. 
Additionally, it includes an event cancellation button that generates an interrupt to stop the buzzer and an emergency button in case the fall event is detected wrong. 



## Features

- [x] **Fall Detection:** The device continuously monitors the user's movements using the MPU6050 sensor. A significant jerk in accelerations is detected to trigger a fall event.

- [x] **Buzzer and LED Alert:** Once a fall is detected, a buzzer and LED are activated to alert nearby individuals about the fall.

- [x] **Event Cancellation Button:** The device is equipped with an event cancellation button. Pressing this button generates an interrupt, stopping the buzzer and preventing false alarms.

- [x] **Emergency Button:** When the elder falls and the device can't catch this event, they can use this button to change to fall event.

- [x] **BLE server:** The server is built to send data from device to phone. It can paring, bonding and using passkey.

- [x] **Send SOS SMS:** In the event of a fall, the device send notification to app through NimBLE and that app send SOS SMS messages to pre-added emergency contacts to request help. The user's location (Google Map) where they fell includes in messages.

- [x] **Mobile Application:** An Android Mobile Application use to connect to the fall detection device and send SOS SMS when the elder falls.

- [ ] **Find Device:** The buzzer of device will be active when user use this function on mobile app.

- [ ] **Enable/Disable BLE Advertising Button:** Using to enable/disable advertising of BLE whenever user want to connect device with other phone or prevent another person connect their phone with it.

## Hardware Components Used:
<p align="center">
<img src="" height="500" width="500">
</p>
<br>

- **ESP32 C3 Core Board** 

- **MPU6050 Accelerometer and Gyroscope Sensor** 

- **Buzzer** 

- **LED** 

- **Event Cancellation Button** 

- **Emergency Button**

## How it Works
<p align="center">
<img src="" height="700" width="450">
</p>
<br>

Device need connect to phone via mobile app.
The MPU6050 sensor measures the user's accelerations and jerk in real-time. If the jerk exceeds a certain threshold, it indicates a fall event.
When falling dectection, the buzzer will sound an alarms. Mobile app will send SOS SMS to phone number which has been added.
The event cancellation button is used to stop the buzzer.
BLE gatt server is used to send data from device to user phone.


## Getting Started

To use the Fall Detection Device, follow these steps:

1. Assemble the hardware components according to the schematic diagram provided.

2. Upload the provided firmware to the ESP32 using ESP-IDF.

3. Install mobile applicaion by apk.

4. Connect to gatt server of the device using app. Pairing and bonding it. The passkey is "123456".

5. Add emergency contacts in app.

6. Wear the device, and it will continuously monitor your movements.

7. In the event of a fall, the buzzer and LED will active, and SOS SMS messages will be sent to the emergency contacts.

