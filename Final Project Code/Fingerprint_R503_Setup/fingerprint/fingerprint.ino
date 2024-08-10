#include <Adafruit_Fingerprint.h>
#define RELAY_PIN_LOCK          3
#define RELAY_PIN_BULB          6
#define ACCESS_DELAY            3000      // Keep lock unlocked for 3 seconds 

#define BUTTON_PIN              5

#define MAX_FAILED_ATTEMPTS     3
#define RESET_PERIOD            30000     // 1 minute in milliseconds

unsigned long last_Failed_Attempt_Time = 0;
int failed_Attempts = 0;
bool light_Active = false;
unsigned long currentTime1 = 0;
unsigned long currentTime2 = 0;
unsigned long currentTime3 = 0;
unsigned long lastActivationTime = 0;
unsigned long lastDebounceTime = 0;

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

void setup()
{
  Serial.begin(9600);  // Set the baud rate to match the ESP32
  
  pinMode(RELAY_PIN_LOCK, OUTPUT);
  pinMode(RELAY_PIN_BULB, OUTPUT);

  pinMode(BUTTON_PIN, INPUT);

  digitalWrite(RELAY_PIN_LOCK, LOW);
  digitalWrite(RELAY_PIN_BULB, LOW);

  // set the data rate for the sensor serial port
  finger.begin(57600);
  delay(5);
  if (finger.verifyPassword()) {
  } 
  else {
    while (1) { delay(1); }
  }
}

void loop()                     // run over and over again
{
  getFingerprintID();
  handle_light();
  handleFailed_Attempts();  // Handle failed attempts and lockout period
  Button();
  handle_ESP32();
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
    delay(1000);
    light_Active = false;          // Deactivate the light
    digitalWrite(RELAY_PIN_BULB, LOW); 
    Serial.write('3'); 
    Unlock_Lock();
    failed_Attempts = 0;
  } 
  else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    return p;
  } 
  else if (p == FINGERPRINT_NOTFOUND) {
    finger.LEDcontrol(FINGERPRINT_LED_FLASHING, 25, FINGERPRINT_LED_RED, 10);
    delay(1000);
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
    delay(ACCESS_DELAY);
    digitalWrite(RELAY_PIN_LOCK, LOW);
    delay(50);
  }
}

void increment_Failed_Attempts()
{
  failed_Attempts++;           // Increment the failed attempts counter

  if (failed_Attempts == 1)
  {
    last_Failed_Attempt_Time = millis();   // Record the time of the first failed attempt
  }
}

void handleFailed_Attempts()
{
  currentTime2 = millis();
  
  if (failed_Attempts >= MAX_FAILED_ATTEMPTS && (currentTime2 - last_Failed_Attempt_Time) < RESET_PERIOD)
  {
    lightning();            // Activate the buzzer if maximum failed attempts reached within lockout period
  }
  
  if ((currentTime2 - last_Failed_Attempt_Time) >= RESET_PERIOD && last_Failed_Attempt_Time != 0)
  {
    failed_Attempts = 0;            // Reset failed attempts counter after lockout period
    last_Failed_Attempt_Time = 0;   // Reset last failed attempt time after lockout period
  }
}

void lightning(){
  if (!light_Active){
    digitalWrite(RELAY_PIN_BULB, HIGH);    // Activate the lock mechanism
    Serial.write('1'); 
    failed_Attempts = 0; 
    light_Active = true;
    lastActivationTime = millis();
  }
}

void handle_light()
{ 
  currentTime1 = millis();   // Get the current time

  if (light_Active && (currentTime1 - lastActivationTime >= 6000))
  {
    light_Active = false;          // Deactivate the light
    digitalWrite(RELAY_PIN_BULB, LOW);
    Serial.write('3'); 
  }
}

void Button()
{
  if (digitalRead(BUTTON_PIN) == HIGH)
  {
    light_Active = false;           // Deactivate the light
    digitalWrite(RELAY_PIN_BULB, LOW);
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
      lightning();
    } 
    else if (command == '3') 
    {
      light_Active = false;              
      digitalWrite(RELAY_PIN_BULB, LOW);
    } 

    else if (command == '5') 
    {
      Unlock_Lock();         // Unlock the lock on button press
      failed_Attempts = 0;   // Reset failed attempts counter on successful button press
      Serial.write('7'); 
    }
  }
}
