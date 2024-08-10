#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

SemaphoreHandle_t mutex;

#define BLYNK_TEMPLATE_ID "TMPL6_7_5om3t"
#define BLYNK_TEMPLATE_NAME "ESP32 Automation"
#define BLYNK_AUTH_TOKEN "FuYjDr9781vj-PUeJZgalaEe8L3UCvD8"

// Comment this out to disable prints and save space
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "HOANG";
char pass[] = "tttttttt7";

unsigned long lastButton1Time = 0;
unsigned long lastButton2Time = 0;
unsigned long lastButton3Time = 0;
unsigned long currentTime = 0;

BlynkTimer timer;

#define button1_pin 26
#define button2_pin 25
#define button3_pin 33

#define led1_pin   16
#define led2_pin   15
#define relay_pin  14

int buzz_state = 0;
int lock_state = 0;
int led1_state = 0;
int led2_state = 0;
int relay_state = 0;

// Change the virtual pins according to the rooms
#define buzz_vpin V0
#define lock_vpin V1
#define button1_vpin V2
#define button2_vpin V3
#define button3_vpin V4

//------------------------------------------------------------------------------
// This function is called every time the device is connected to the Blynk.Cloud
// Request the latest state from the server
BLYNK_CONNECTED() {
  Blynk.syncVirtual(buzz_vpin);
  Blynk.syncVirtual(lock_vpin);
  Blynk.syncVirtual(button1_vpin);
  Blynk.syncVirtual(button2_vpin);
  Blynk.syncVirtual(button3_vpin);
}
//--------------------------------------------------------------------------

BLYNK_WRITE(buzz_vpin) {
  buzz_state = param.asInt();
  if (buzz_state == 1) {
    xSemaphoreTake(mutex, portMAX_DELAY);          // Acquire the semaphore to ensure exclusive access
    Serial.write('1');                             // Send command to Arduino to BUZZ_PIN to turn on
    xSemaphoreGive(mutex);                         // Release the semaphore
  }
  else {
    xSemaphoreTake(mutex, portMAX_DELAY);          // Acquire the semaphore to ensure exclusive access
    Serial.write('3');                             // Send command to Arduino to BUZZ_PIN to turn off
    xSemaphoreGive(mutex);                         // Release the semaphore
  }
}

BLYNK_WRITE(lock_vpin) {
  lock_state = param.asInt();
  if (lock_state == 1) {
    xSemaphoreTake(mutex, portMAX_DELAY);          // Acquire the semaphore to ensure exclusive access
    Serial.write('5');                             // Send command to Arduino to RELAY_PIN_LOCK to do the [void Unlock_Lock(); function]
    xSemaphoreGive(mutex);                         // Release the semaphore
  }
  else {
    xSemaphoreTake(mutex, portMAX_DELAY);
    xSemaphoreGive(mutex);
  }
}
//---------------------------------------------------------------------------

BLYNK_WRITE(button1_vpin) {
  led1_state = param.asInt();
  digitalWrite(led1_pin, led1_state);
}

BLYNK_WRITE(button2_vpin) {
  led2_state = param.asInt();
  digitalWrite(led2_pin, led2_state);
}

BLYNK_WRITE(button3_vpin) {
  relay_state = param.asInt();
  digitalWrite(relay_pin, relay_state);
}
//---------------------------------------------------------------------------

void setup() {
  // Communicate with Arduino
  Serial.begin(9600);            // Set the baud rate to match the Arduino
  // Create the mutex
  mutex = xSemaphoreCreateMutex();
  
  // Debug console [Computer]
  // Serial.begin(115200);       // Set the baud rate to match the Upload Speed Default of the Computer       
  //--------------------------------------------------------------------
  pinMode(button1_pin, INPUT);
  pinMode(button2_pin, INPUT);
  pinMode(button3_pin, INPUT);
  //--------------------------------------------------------------------
  pinMode(led1_pin, OUTPUT);
  pinMode(led2_pin, OUTPUT);
  pinMode(relay_pin, OUTPUT);
  //--------------------------------------------------------------------
  // During Starting all LEDs should be turned OFF
  digitalWrite(led1_pin, LOW);
  digitalWrite(led2_pin, LOW);
  digitalWrite(relay_pin, LOW);
  //--------------------------------------------------------------------  
  Blynk.begin(auth, ssid, pass, "blynk.cloud", 80);
}

void loop() {
  Blynk.run();
  timer.run();
  
  listen_push_buttons();
  read_serial_commands();
}

void listen_push_buttons() {
  
  currentTime = millis();

  if (digitalRead(button1_pin) == HIGH && currentTime - lastButton1Time > 500) {
    lastButton1Time = millis();
    led1_state = !led1_state;
    digitalWrite(led1_pin, led1_state);
    Blynk.virtualWrite(button1_vpin, led1_state); // Update button state
  }

  if (digitalRead(button2_pin) == HIGH && currentTime - lastButton2Time > 500) {
    lastButton2Time = millis();
    led2_state = !led2_state;
    digitalWrite(led2_pin, led2_state);
    Blynk.virtualWrite(button2_vpin, led2_state); // Update button state
  }

  if (digitalRead(button3_pin) == HIGH && currentTime - lastButton3Time > 500) {
    lastButton3Time = millis();
    relay_state = !relay_state;
    digitalWrite(relay_pin, relay_state);
    Blynk.virtualWrite(button3_vpin, relay_state); //update button state
  }
}

void read_serial_commands() {
  if (Serial.available()) {
    char command = Serial.read();                // read command from Arduino
    if (command == '1') {
        buzz_state = 1;
        Blynk.virtualWrite(buzz_vpin, buzz_state);
    }
    else if (command == '3') {
        buzz_state = 0;
        Blynk.virtualWrite(buzz_vpin, buzz_state);
    }
    else if (command == '7' && lock_state == 1) {
        lock_state = 0;
        Blynk.virtualWrite(lock_vpin, lock_state);
    }
  }
}
