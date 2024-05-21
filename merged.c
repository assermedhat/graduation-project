#include <SPI.h>
#include <MFRC522.h>

// RFID setup
#define SS_PIN 10
#define RST_PIN 9
MFRC522 rfid(SS_PIN, RST_PIN);

// Ultrasonic sensor pins
const int trigPins[6] = {2, 4, 6, 8, A0, A2};
const int echoPins[6] = {3, 5, 7, 9, A1, A3};

// Timing variables
unsigned long previousMillisRFID = 0;
unsigned long previousMillisUltrasonic = 0;
const long intervalRFID = 1000; // Time interval for RFID reading (in ms)
const long intervalUltrasonic = 500; // Time interval for ultrasonic sensor reading (in ms)

// Function to measure distance with ultrasonic sensor
long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  long distance = duration * 0.034 / 2;
  return distance;
}

// Function to read RFID
void readRFID() {
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    Serial.print("Card UID: ");
    for (byte i = 0; i < rfid.uid.size; i++) {
      Serial.print(rfid.uid.uidByte[i] < 0x10 ? " 0" : " ");
      Serial.print(rfid.uid.uidByte[i], HEX);
    }
    Serial.println();
    rfid.PICC_HaltA();
  }
}

// Function to read all ultrasonic sensors
void readUltrasonicSensors() {
  for (int i = 0; i < 6; i++) {
    long distance = readUltrasonic(trigPins[i], echoPins[i]);
    Serial.print("Distance from sensor ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(distance);
    Serial.println(" cm");
    delay(100);  // Small delay to prevent interference
  }
}

void setup() {
  Serial.begin(9600);
  SPI.begin();
  rfid.PCD_Init();

  // Initialize ultrasonic sensor pins
  for (int i = 0; i < 6; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }

  // Ensure RFID reset pin is set to a known state
  pinMode(RST_PIN, OUTPUT);
  digitalWrite(RST_PIN, HIGH);
}

void loop() {
  unsigned long currentMillis = millis();

  // Check if it's time to read RFID
  if (currentMillis - previousMillisRFID >= intervalRFID) {
    previousMillisRFID = currentMillis;
    readRFID();
  }

  // Check if it's time to read ultrasonic sensors
  if (currentMillis - previousMillisUltrasonic >= intervalUltrasonic) {
    previousMillisUltrasonic = currentMillis;
    readUltrasonicSensors();
  }
}
