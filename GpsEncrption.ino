#include <LiquidCrystal.h>
#include <AESLib.h> // AES encryption library

LiquidCrystal lcd(7, 6, 5, 4, 3, 2); // LCD pins

// A structure to map the lcoation cordinates with the name of the place.
struct Location {
  String name;   
  float latitude;  
  float longitude; 
};

Location locations[] = {
  {"Panther Dining Hall", 28.0658, -80.6244}, // Car's location
  {"Evans Library", 28.0659, -80.6238}, // Should open the car
  {"Melbourne Beach", 28.0836, -80.6081}, //Should be the Eavesdropper
  {"Skurla Hall", 28.0660, -80.6225}, // Should be out of range.

  {"Clemente Center for Sports and Recreation", 28.0662, -80.6250},
  {"Olin Engineering Complex", 28.0665, -80.6235},
  {"Gleason Performing Arts Center", 28.0657, -80.6232},
  {"Roberts Hall", 28.0654, -80.6240},
  {"Denius Student Center", 28.0656, -80.6236},
  {"Downtown Melbourne", 28.1173, -80.6257},
  {"Kennedy Space Center", 28.5721, -80.6480}
};
const int numLocations = sizeof(locations) / sizeof(locations[0]); // Calculate the size of the array

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
float calculateDistance(const Location& loc1, const Location& loc2) {
  float R = 6371.0; // Earth's radius in kilometers
  float dLat = radians(loc2.latitude - loc1.latitude);
  float dLon = radians(loc2.longitude - loc1.longitude);
  float a = sin(dLat / 2) * sin(dLat / 2) +
            cos(radians(loc1.latitude)) * cos(radians(loc2.latitude)) *
            sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c * 1000; // Distance in meters
}
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
  Serial.begin(9600); // Start serial communication at 9600 baud for the serial input

  // Initialize the GPS log with zero values
  for (int i = 0; i < LOG_SIZE; i++) {
    keyLocationLog[i][0] = 0.0;
    keyLocationLog[i][1] = 0.0;
  }
}

void loop() {
  if (millis() - lastUpdate >= 25000) { // Update every 25 seconds
    astUpdate = millis();

    // Simulate getting the key's GPS location
    static int locationIndex = 0;
    locationIndex = (locationIndex + 1) % numLocations; // Cycle through all predefined locations
    Location keyLocation = locations[locationIndex]; // Get the current key location as a Location struct

    Location carLocation = locations[0]; // Panther Dining Hall (Car's location)

    // Add the current key location to the GPS log
    keyLocationLog[logIndex][0] = keyLocation.latitude;
    keyLocationLog[logIndex][1] = keyLocation.longitude;
    logIndex = (logIndex + 1) % LOG_SIZE; // Move to the next log index (circular buffer)

    // Calculate the distance between the car and the key
    float distance = calculateDistance(carLocation, keyLocation);

    // Clear the LCD screen for new messages
    lcd.clear();

     // Output the key location and distance and info of the status of the key.
      Serial.println(" -------------- \n ");
      Serial.print("Key Location: ");
      Serial.println(keyLocation.name);
      Serial.print("Distance to Car: ");
      Serial.print(distance, 1);
      Serial.println(" meters");
     
    // If the key is near the car
    if (distance <= 70.0) { // Within 70 meters
      const char *message = "open";
      String encryptedSignal = encryptMessage(message); // Encrypt "open" command
      String decryptedSignal = decryptMessage(encryptedSignal.c_str()); // Decrypt for verification

      if (decryptedSignal == "open") { // If decrypted successfully
        lcd.print("Open the car");
        Serial.println("Car unlocked: Key is in range.");

      } else {
        lcd.print("Auth Failed");
        Serial.println("Car Auth Failed");
      }
    } 
    // If an eavesdropper is detected
    else if (detectEavesdropper()) {
      lcd.print("Eavesdropper!");
      lcd.setCursor(0, 1);
      lcd.print("Lock Disabled");       // Disable the car's locking system or raise an alert
        Serial.println("Alert: Eavesdropper detected! Lock disabled.");

    } 
    // Key is too far but no eavesdropper detected
    else {
      lcd.print("Fake Key");
            Serial.println("Car locked: Key is out of range or fake.");
    }

    // Display the distance on the second line
    lcd.setCursor(0, 1);
    lcd.print("Dist:");
    lcd.print(distance, 1);
    lcd.print("m");
  }

  delay(100); // Small delay for smoother operation
}
