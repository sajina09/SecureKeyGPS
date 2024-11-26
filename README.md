# GPS and MCU Simulation in Proteus

This project simulates a **GPS-based car access control system** and **anomaly detection system** using an LCD display, AES encryption, and predefined GPS coordinates. The simulation is designed to run in a Proteus environment to demonstrate secure car unlocking based on proximity.

---

## Features
1. **GPS Simulation**:
   - Predefined locations simulate a GPS receiver.
   - Tracks the car's location and a key's location.
   - Measures the distance between the car and the key.

2. **AES Encryption**:
   - Commands (e.g., "open") are encrypted using AES-128 before transmission.
   - Ensures secure communication between devices.

3. **Anomaly Detection**:
   - Detects unusual key movements (e.g., an eavesdropper carrying a cloned key away).
   - Alerts and disables car access if an anomaly is detected.

4. **LCD Output**:
   - Displays the system status, distance between car and key, and alerts.

---

## Hardware Simulation Requirements
To simulate this project in **Proteus**, ensure the following components are included:
1. **Microcontroller (MCU)**: Arduino or similar.
2. **16x2 LCD Display**.
3. **GPS Module (simulated using predefined coordinates)**.
4. **AES Encryption Library**: Simulated in the code using the `AESLib` library.
5. **Virtual Terminal (optional)**: To log encrypted and decrypted messages for debugging.

---

## Software Dependencies
- **Arduino IDE**:
  - Include the following libraries:
    - `LiquidCrystal.h` for LCD control.
    - `AESLib.h` for AES encryption.
- **Proteus Simulation Software**.

---

## How It Works
1. **Initialization**:
   - The LCD and GPS log buffer are initialized.
   - AES encryption key and initialization vector are predefined.

2. **Main Loop**:
   - The key's GPS location updates every 25 seconds.
   - The system calculates the distance between the car and the key.
   - If the key is within 70 meters of the car:
     - The system sends an "open" command encrypted with AES-128.
     - The signal is decrypted and verified.
     - The car unlocks if verification succeeds.
   - If an anomaly (e.g., key movement > 1 km) is detected:
     - The system locks down and raises an alert.
   - If the key is out of range, the system displays a "Fake Key" message.

---

## Messages on LCD
- **Open the Car**:
  - The key is in proximity, and the command is authenticated.
- **Fake Key**:
  - The key is too far from the car.
- **Eavesdropper**:
  - Suspicious key movement detected; the system locks down.
- **Dist: X.XXm**:
  - Displays the calculated distance between the car and the key.

---

## Code Overview
### Key Components:
1. **GPS Simulation**:
   - Predefined locations simulate GPS updates.
   - Uses a circular buffer to log the last 5 minutes of key movements.

2. **Distance Calculation**:
   - Haversine formula calculates the distance between two coordinates.

3. **Encryption and Decryption**:
   - AES-128 encrypts the "open" command.
   - Decryption verifies the authenticity of the signal.

4. **Anomaly Detection**:
   - Compares current key location with the log to detect suspicious movements.

---

## Customization
- **Locations**:
  - Update the `locations` array to simulate new GPS points.
- **Distance Threshold**:
  - Adjust the `70.0` meter range for unlocking.
- **Anomaly Detection Threshold**:
  - Change the `1000.0` meter limit to fine-tune eavesdropper detection.

---

## Simulation Steps in Proteus
1. Set up the circuit with an Arduino, 16x2 LCD, and a GPS module.
2. Upload the code to the simulated MCU.
3. Observe the LCD output as the system processes GPS locations and reacts to various scenarios.

---

## Future Enhancements
- **Real GPS Integration**: Replace simulated coordinates with real GPS module data.
- **Bluetooth/WiFi Communication**: Enable secure wireless communication.
- **Additional Alerts**: Integrate alarms or notifications for eavesdropping detection.

---

Enjoy building and simulating this secure car access system in Proteus!
