# GPS and MCU Simulation in Proteus

This project simulates a **GPS-based car access control system** with **anomaly detection** using a microcontroller and LCD in Proteus. The system ensures secure car unlocking based on proximity using GPS and AES encryption.

---

## Features
- **GPS Simulation**:
  - Tracks the car and key locations.
  - Calculates the distance between them.
- **AES Encryption**:
  - Secures commands like "open" with AES-128 encryption.
- **Anomaly Detection**:
  - Identifies suspicious key movements and raises alerts.
- **LCD Display**:
  - Shows system status, distance, and alerts.

---

## Requirements
- **Hardware in Proteus**:
  - Microcontroller (e.g., Arduino)
  - 16x2 LCD
  - GPS module (simulated coordinates)
- **Software**:
  - Arduino IDE with `LiquidCrystal.h` and `AESLib.h`
  - Proteus simulation software

---

## How It Works
1. **Distance Calculation**:
   - Uses GPS data to calculate the distance between car and key.
2. **Unlocking**:
   - If the key is within 70 meters, the system sends an encrypted "open" command.
3. **Anomaly Detection**:
   - Detects unusual key movements and locks the system.
4. **LCD Messages**:
   - Displays distance, status, or alerts like "Fake Key" or "Eavesdropper."

---

## Customization
- **Change GPS Points**:
  - Update predefined coordinates for new locations.
- **Adjust Distance**:
  - Modify the proximity threshold for unlocking.

---

## Simulation Steps
1. Build the circuit in Proteus with Arduino, GPS, and LCD.
2. Upload the code to the MCU.
3. Test the system by simulating different GPS locations.

---

## Additional Information
The detailed paper explaining this project has been attached. You can go through it for an in-depth understanding.

---

## Future Improvements
- Use a real GPS module for live data.
- Add Bluetooth or WiFi for wireless communication.

---

This project demonstrates a secure and interactive car access system in a simulated environment.
