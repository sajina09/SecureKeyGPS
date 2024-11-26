#include <LiquidCrystal.h>
#include <AESLib.h> // AES encryption library

LiquidCrystal lcd(7, 6, 5, 4, 3, 2); // LCD pins

// Predefined GPS coordinates for simulation
float locations[12][2] = {
  {28.0658, -80.6244}, // Panther Dining Hall (Car's location)
  {28.0659, -80.6238}, // Evans Library
  {28.0662, -80.6250}, // Clemente Center for Sports and Recreation
  {28.0665, -80.6235}, // Olin Engineering Complex
  {28.0657, -80.6232}, // Gleason Performing Arts Center
  {28.0660, -80.6225}, // Skurla Hall
  {28.0654, -80.6240}, // Roberts Hall
  {28.0656, -80.6236},  // Denius Student Center

  // Places far away.
  {28.0836, -80.6081}, // Melbourne Beach
  {28.1173, -80.6257}, // Downtown Melbourne
  {28.5721, -80.6480}  // Kennedy Space Center
};

// Car's stationary location (latitude, longitude)
float carLocation[2] = {28.0658, -80.6244};

// Encryption Key (16 bytes for AES-128)
byte aes_key[16] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10};

// Initialization Vector (IV)
byte aes_iv[16] = {0x10, 0x0F, 0x0E, 0x0D, 0x0C, 0x0B, 0x0A, 0x09,
                   0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01};

// Circular buffer for logging last 5 minutes of key's GPS locations
#define LOG_SIZE 12 // 12 logs for 5 minutes (1 log every 25 seconds)
float keyLocationLog[LOG_SIZE][2];
int logIndex = 0;

// Last update timestamp
unsigned long lastUpdate = 0;

// Function to calculate distance between two GPS coordinates
float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
  float R = 6371.0; // Earth's radius in kilometers
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float a = sin(dLat / 2) * sin(dLat / 2) +
            cos(radians(lat1)) * cos(radians(lat2)) *
            sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c * 1000; // Distance in meters
}

// Encrypt message using AES-128
String encryptMessage(const char *msg) {
  char encrypted[32];
  aes128_cbc_encrypt((byte *)msg, (byte *)encrypted, strlen(msg), aes_key, aes_iv);
  return String(encrypted);
}

// Decrypt message using AES-128
String decryptMessage(const char *encryptedMsg) {
  char decrypted[32];
  aes128_cbc_decrypt((byte *)encryptedMsg, (byte *)decrypted, strlen(encryptedMsg), aes_key, aes_iv);
  return String(decrypted);
}

// Check for anomalies in the key's GPS location log
bool detectEavesdropper() {
  // Compare the current key location with the logged locations
  for (int i = 0; i < LOG_SIZE; i++) {
    if (keyLocationLog[i][0] != 0.0 && keyLocationLog[i][1] != 0.0) { // If log entry exists
      float distance = calculateDistance(
          keyLocationLog[i][0], keyLocationLog[i][1], 
          keyLocationLog[logIndex][0], keyLocationLog[logIndex][1]);
      if (distance > 1000.0) { // Eavesdropper threshold: Key moved > 1km
        return true; // Potential eavesdropper detected
      }
    }
  }
  return false;
}

void setup() {
  lcd.begin(16, 2); // Initialize LCD
  // Initialize the GPS log with zero values
  for (int i = 0; i < LOG_SIZE; i++) {
    keyLocationLog[i][0] = 0.0;
    keyLocationLog[i][1] = 0.0;
  }
}

void loop() {
  if (millis() - lastUpdate >= 25000) { // Update every 25 seconds
    lastUpdate = millis();

    // Simulate getting the key's GPS location
    static int locationIndex = 0;
    locationIndex = (locationIndex + 1) % 4; // Cycle through predefined locations list
    float keyLat = locations[locationIndex][0];
    float keyLon = locations[locationIndex][1];

    // Add the current key location to the GPS log
    keyLocationLog[logIndex][0] = keyLat;
    keyLocationLog[logIndex][1] = keyLon;
    logIndex = (logIndex + 1) % LOG_SIZE; // Move to the next log index (circular buffer)

    // Calculate the distance between the car and the key
    float distance = calculateDistance(carLocation[0], carLocation[1], keyLat, keyLon);

    // Clear the LCD screen for new messages
    lcd.clear();

    // If the key is near the car
    if (distance <= 70.0) { // Within 70 meters
      const char *message = "open";
      String encryptedSignal = encryptMessage(message); // Encrypt "open" command
      String decryptedSignal = decryptMessage(encryptedSignal.c_str()); // Decrypt for verification

      if (decryptedSignal == "open") { // If decrypted successfully
        lcd.print("Open the car");
      } else {
        lcd.print("Auth Failed");
      }
    } 
    // If an eavesdropper is detected
    else if (detectEavesdropper()) {
      lcd.print("Eavesdropper!");
      lcd.setCursor(0, 1);
      lcd.print("Lock Disabled");
      // Disable the car's locking system or raise an alert
    } 
    // Key is too far but no eavesdropper detected
    else {
      lcd.print("Fake Key");
    }

    // Display the distance on the second line
    lcd.setCursor(0, 1);
    lcd.print("Dist:");
    lcd.print(distance, 1);
    lcd.print("m");
  }

  delay(100); // Small delay for smoother operation
}
