#include <Adafruit_Fingerprint.h>
#include <SoftwareSerial.h>

#if (defined(__AVR__) || defined(ESP8266)) && !defined(__AVR_ATmega2560__)
// For UNO and others without hardware serial, we must use software serial...
// pin #2 is IN from sensor (GREEN wire)
// pin #3 is OUT from arduino  (WHITE wire)
// Set up the serial port to use softwareserial..
SoftwareSerial mySerial(12, 13);

#else
// On Leonardo/M0/etc, others with hardware serial, use hardware serial!
// #0 is green wire, #1 is white
#define mySerial Serial1

#endif

Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);

#define RELAY_PIN_LOCK          3
#define RELAY_PIN_BULB          6
#define ACCESS_DELAY            3000      // Keep lock unlocked for 3 seconds 
#define LED_DELAY               1000      // Keep lock unlocked for 3 seconds 

#define SENSOR_PIN              2
#define BUZZ_PIN                7
#define BUTTON_PIN              5

#define MAX_FAILED_ATTEMPTS     5
#define RESET_PERIOD            60000     // 1 minute in milliseconds

unsigned long last_Failed_Attempt_Time = 0;
int failed_Attempts = 0;
bool buzzer_Active = false;

unsigned long currentTime1 = 0;
unsigned long currentTime2 = 0;
unsigned long currentTime3 = 0;
unsigned long currentTime4 = 0;
unsigned long currentTime5 = 0;
unsigned long lastActivationTime = 0;
unsigned long lastDebounceTime = 0;
unsigned long unlockEndTime = 0;
unsigned long ledcorrect = 0;
unsigned long ledfail = 0;

void setup()
{
  Serial.begin(9600);        // Set the baud rate to match the ESP32
  
  finger.begin(57600);       // set the data rate for the sensor serial port
  delay(5);
  if (finger.verifyPassword()) {
  } 
  else {
    while (1) { delay(1); }
  }
  
  pinMode(RELAY_PIN_LOCK, OUTPUT);
  pinMode(RELAY_PIN_BULB, OUTPUT);
  
  pinMode(BUZZ_PIN, OUTPUT);
  pinMode(SENSOR_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);
  
  digitalWrite(RELAY_PIN_LOCK, LOW);
  digitalWrite(RELAY_PIN_BULB, LOW);
  digitalWrite(BUZZ_PIN, HIGH);
}

void loop()
{
  getFingerprintID();       // Perform fingerprint authentication
  Sensor_Lightbulb();       // Control the lightbulb based on sensor input
  Button();                 // Check if the button is pressed for manual unlocking
  handleFailed_Attempts();  // Handle failed attempts and lockout period
  handle_Buzzer();           // Handle buzzer activation and deactivation
  handle_ESP32();           // Handle the commands sent from the blynk app of the ESP32
}

uint8_t getFingerprintID() {
  uint8_t p = finger.getImage();
  switch (p) {
    case FINGERPRINT_OK:
      break;
    case FINGERPRINT_NOFINGER:
      finger.LEDcontrol(FINGERPRINT_LED_OFF, 0, FINGERPRINT_LED_PURPLE);
      finger.LEDcontrol(FINGERPRINT_LED_OFF, 0, FINGERPRINT_LED_RED);
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      return p;
    case FINGERPRINT_IMAGEFAIL:
      return p;
    default:
      return p;
  }

  // OK success!

  p = finger.image2Tz();
  switch (p) {
    case FINGERPRINT_OK:
      break;
    case FINGERPRINT_IMAGEMESS:
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      return p;
    case FINGERPRINT_FEATUREFAIL:
      return p;
    case FINGERPRINT_INVALIDIMAGE:
      return p;
    default:
      return p;
  }

  // OK converted!
  p = finger.fingerSearch();
  if (p == FINGERPRINT_OK) {
    finger.LEDcontrol(FINGERPRINT_LED_FLASHING, 25, FINGERPRINT_LED_PURPLE, 10);
    currentTime4 = millis();
    ledcorrect = currentTime4 + LED_DELAY;
    while (currentTime4 < ledcorrect) {
      currentTime4 = millis();
    }
    buzzer_Active = false;         
    digitalWrite(BUZZ_PIN, HIGH); 
    Serial.write('3'); 
    Unlock_Lock();
    failed_Attempts = 0;
  } 
  else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    return p;
  } 
  else if (p == FINGERPRINT_NOTFOUND) {
    finger.LEDcontrol(FINGERPRINT_LED_FLASHING, 25, FINGERPRINT_LED_RED, 10);
    currentTime5 = millis();
    ledfail = currentTime5 + LED_DELAY;
    while (currentTime5 < ledfail) {
      currentTime5 = millis();
    }
    increment_Failed_Attempts();
    return p;
  } 
  else {
    return p;
  }

  // found a match!
  return finger.fingerID;
}

void Unlock_Lock() {
  currentTime3 = millis();

  if (currentTime3 - lastDebounceTime > 50) {
    lastDebounceTime = millis();

    digitalWrite(RELAY_PIN_LOCK, HIGH);
    unlockEndTime = currentTime3 + ACCESS_DELAY;
    while (currentTime3 < unlockEndTime) {
      currentTime3 = millis();
    }
    digitalWrite(RELAY_PIN_LOCK, LOW);
  }
}

void increment_Failed_Attempts()
{
  failed_Attempts++;                       // Increment the failed attempts counter
  
  if (failed_Attempts == 1)
  {
    last_Failed_Attempt_Time = millis();   // Record the time of the first failed attempt
  }
}

void Sensor_Lightbulb()
{
  int sensorValue = digitalRead(SENSOR_PIN);
  
  if (sensorValue == HIGH)
  {
    digitalWrite(RELAY_PIN_BULB, HIGH);  // Turn on the lightbulb if the sensor detects a high value
  }
  else
  {
    digitalWrite(RELAY_PIN_BULB, LOW);   // Turn off the lightbulb if the sensor detects a low value
  }
}

void handleFailed_Attempts()
{
  currentTime2 = millis();
  
  if (failed_Attempts >= MAX_FAILED_ATTEMPTS && (currentTime2 - last_Failed_Attempt_Time) < RESET_PERIOD)
  {
    activate_Buzzer();              // Activate the buzzer if maximum failed attempts reached within lockout period
  }
  
  if ((currentTime2 - last_Failed_Attempt_Time) >= RESET_PERIOD && last_Failed_Attempt_Time != 0)
  {
    failed_Attempts = 0;            // Reset failed attempts counter after lockout period
    last_Failed_Attempt_Time = 0;   // Reset last failed attempt time after lockout period
  }
}

void activate_Buzzer(){
  if (!buzzer_Active){
    digitalWrite(BUZZ_PIN, LOW);    // Activate the lock mechanism
    Serial.write('1'); 
    failed_Attempts = 0; 
    buzzer_Active = true;
    lastActivationTime = millis();
  }
}

void handle_Buzzer()
{ 
  currentTime1 = millis();          // Get the current time

  if (buzzer_Active && (currentTime1 - lastActivationTime >= 6000))
  {
    buzzer_Active = false;          // Deactivate the light
    digitalWrite(BUZZ_PIN, HIGH);
    Serial.write('3'); 
  }
}

void Button()
{
  if (digitalRead(BUTTON_PIN) == HIGH)
  {
    buzzer_Active = false;          // Deactivate the light
    digitalWrite(BUZZ_PIN, HIGH);
    Serial.write('3'); 
    Unlock_Lock();                  // Unlock the lock on button press
    failed_Attempts = 0;            // Reset failed attempts counter on successful button press
  }
}

void handle_ESP32() {
  if (Serial.available()) {
    char command = Serial.read();      // read command from ESP32
    if (command == '1') 
    {
      activate_Buzzer();
    } 
    else if (command == '3') 
    {
      buzzer_Active = false;              
      digitalWrite(BUZZ_PIN, HIGH);
    } 

    else if (command == '5') 
    {
      Unlock_Lock();         // Unlock the lock on button press
      failed_Attempts = 0;   // Reset failed attempts counter on successful button press
      Serial.write('7'); 
    }
  }
}
