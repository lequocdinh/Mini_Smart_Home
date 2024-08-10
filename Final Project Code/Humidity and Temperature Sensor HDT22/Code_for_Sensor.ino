#include <Adafruit_Sensor.h>
#include <DHT_U.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define DHT_PIN                9         // DHT22 sensor data pin
#define DHTTYPE                DHT22     // DHT sensor type

#define MOTOR_PIN              10        // Enable pin for motor driver (PWM pin)
#define LED_WARNING_PIN        8         // Lightup LED 

const float TEMPERATURE_THRESHOLD = 35.0;      // Temperature threshold in degrees Celsius
const float HUMIDITY_THRESHOLD = 20.0;         // Humidity threshold in percentage

LiquidCrystal_I2C lcd(0x27, 16, 2);            // Adjust the I2C address if necessary

DHT Sensor(DHT_PIN, DHTTYPE);

byte degree_symbol[8] = 
{
  0b00111,
  0b00101,
  0b00111,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};

void setup() {
  pinMode(MOTOR_PIN, OUTPUT);      // Set the fan pin as an output
  
  Sensor.begin();
  
  lcd.init();                       // initialize the lcd
  lcd.backlight(); 
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Temp = ");
  lcd.setCursor(0,1);
  lcd.print("Humidity = ");
  lcd.createChar(1, degree_symbol);
  lcd.setCursor(11,0);
  lcd.write(1);
  lcd.print("C");
  lcd.setCursor(15,1);
  lcd.print("%");
}

void loop() {
  // Read temperature and humidity values from the sensor
  float temperature = Sensor.readTemperature();
  float humidity = Sensor.readHumidity();

  // Check temperature and humidity conditions
  if (isnan(temperature) || isnan(humidity)) {
    lcd.setCursor(7, 0);
    lcd.print("Error    "); // Clear the previous value
    lcd.setCursor(11, 1);
    lcd.print("Error    "); // Clear the previous value
    digitalWrite(MOTOR_PIN, LOW); // Turn off the fan
  } 
  else {
    lcd.setCursor(7, 0);
    lcd.print(temperature, 1);
    lcd.write(1);
    lcd.print("C");
    lcd.setCursor(11, 1);
    lcd.print(humidity, 1);
    lcd.print("%");
  } 
    
  if (temperature > TEMPERATURE_THRESHOLD || humidity < HUMIDITY_THRESHOLD) {
      digitalWrite(MOTOR_PIN, HIGH);         // Turn on the fan
      digitalWrite(LED_WARNING_PIN, HIGH);   // Turn on the LED
  } 
  else {
      digitalWrite(MOTOR_PIN, LOW);          // Turn off the fan
      digitalWrite(LED_WARNING_PIN, LOW);    // Turn off the LED
  }

  // Delay between readings
  delay(2000); // Adjust the delay duration as needed
}
