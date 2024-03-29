// Copyright reserved 2024 Xorbite

#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Servo.h>
#include <SPI.h>
#include <DHT11.h>
#include <WiFiS3.h>
#include "arduino_secrets.h"
#include <ArduinoMqttClient.h>
#include <Adafruit_MCP3008.h>
#include <string>
#include <Bounce2.h>

// Defining our display's resolution
#define OLED_WIDTH 128
#define OLED_HEIGHT 64

// Defining the display's memory address
#define OLED_ADDR 0x3C

// Instantiate the display with it's correspondent resolution
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT);

// Instantiating Debouncer
Bounce debouncer = Bounce(); 

// Instantiate the MCP3008 chip
Adafruit_MCP3008 mcp;

// MCP3008 chip/slave select pin
const int csPin = 10;

// Defining the servos
Servo horizontalServo;
Servo verticalServo;

// LEDs pins
const int RedLed_pin = 4;
const int YellowLed_pin = 0;

// Servos pins
const int horizontalServoPin = 9;
const int verticalServoPin = 5;

// Caliberating the servos
int hServoPos = 10;
int hServoLimitLow = 10;
int hServoLimitHigh = 180;

int vServoPos = 10;
int vServoLimitLow = 10;
int vServoLimitHigh = 180;


// ---- Begining joystick code

// Joystick pins
const int joyXPin = 7;
const int joyYPin = 6;
const int jsSwitchPin = 2;
bool toggleState = false; // Current toggled state


// Neutral position of joystick (usually the midpoint)
const int neutralX = 511;
const int neutralY = 524;

// Deadzone to determine if the joystick is in neutral position
const int deadzone = 10;

// END of joystick code ----


// Ultrasonic sensor pins
const int trigPin = 8;
const int echoPin = 7;


// Defining duration and distance
long duration;
int distance;
String isObstacleDetected = "false";

// Humidity + Temp sensor code
// Defining the dht DHT sensor;
const int tempPin = 6;
float temperature;
int humidity;
DHT11 dht11(tempPin);

// LDR pins
const int ldr1Pin = A0; // Bottom-Left
const int ldr2Pin = A2; // Top-Left
const int ldr3Pin = A3; // Top-Right 
const int ldr4Pin = A1; // Bottom-Right
int LDR1Value = 0;
int LDR2Value = 0;
int LDR3Value = 0;
int LDR4Value = 0;

// Manual mode check variable
bool manualSolarMode = false;

// Threshold for LDR difference
const int LDRThreshold = 50;

// Active buzzer pins
const int aBuzzerPin = 3;

// Voltage sensor pin
const int VSensorPin = 5;
float voltage;

// Current sensor pin
const int ASensorPin = 4;
float current;

// Normal current voltage with nothing plugged in
const float zeroCurrentVoltage = 2.5;

// MQTT & WIFI initialization 
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;

// Calling WIFI object
WiFiClient wifiClient;

// Calling MQTT Object
MqttClient mqttClient(wifiClient);

// Further intial instructions
const char broker[] = "192.168.144.1";
const int port = 1883;

// Defining the topics to be published
const char tempTopic[] = "firstlastname/temp";
const char humidityTopic[] = "firstlastname/humidity";
const char objectDetectionTopic[] = "firstlastname/objectDetection";
const char solarVoltageTopic[] = "firstlastname/solarVoltage";
const char solarCurrentTopic[] = "firstlastname/solarCurrent";
const char manualModeCheckTopic[] = "firstlastname/manualModeCheck";
// const char subscribeTopic[] = "firstlastname/nothing";
// long count = 0;
// const long interval = 1000; //analog read interval

// Declaring the initial variables
unsigned long previousMillisTemp = 0;
unsigned long previousMillisHumidity = 0;
unsigned long previousMillisCurrent = 0;
unsigned long previousMillisVoltage = 0;
unsigned long previousMillisObjDetection = 0;
unsigned long previousMillisManualModeCheck = 0;

// Setting an interval for data transfer for different topics
const long intervalTemp = 2000;    // Interval at which to send temperature (milliseconds)
const long intervalHumidity = 2000;    // Interval at which to send temperature (milliseconds)
const long intervalCurrent = 5000; // Interval for solar current
const long intervalVoltage = 5000; // Interval for solar voltag
const long intervalObjDetection = 1000; // Interval for object detection system
const long intervalManualModeCheck = 1000; // Interval for object detection system


void setup() 
{  
  // Switch to channel (Baud rate) 9600 (To be able to see the messages, it needs to be on the same baud rate as the output system)
  Serial.begin(9600);
  
  // Initiate the SPI protocol for communication and setup the necessary pins accordingly
  mcp.begin(csPin);

  delay(500); // Delay for booting up the system

  // WIFI code
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    Serial.println("Wifi connection failed, making another attempt; please wait!");
    delay(5000);
  } 

  Serial.println("Wifi connected successfully!");


  // MQTT code
  bool MQTTconnected = false;

  while (!MQTTconnected) {
    if (!mqttClient.connect(broker, port)) {
      delay(1000);  
      Serial.println("Connection with MQTT broker has failed, trying again; please wait!");
    } else{
      MQTTconnected = true;
      Serial.println("Connected successfully to the MQTT broker!");
    }
  }

  mqttClient.onMessage(onMqttMessage);
  // mqttClient.subscribe(subscribeTopic);

  

  // OLED code
  // Begin sending signals to the display and link with the defined address on memory to get the data sent
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();

  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(5, 20);
  display.println("SolarSonIO");
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(8, 50);
  display.println("Made by Yazan Akkad");
  display.display();
  delay(4000);

  // LED code
  pinMode(RedLed_pin, OUTPUT);
  pinMode(YellowLed_pin, OUTPUT);


  // Servo & petentiometer
  // Linking the servo to the declared pin
  horizontalServo.attach(horizontalServoPin);
  verticalServo.attach(verticalServoPin);
  
  // Initialize the servos (90 degrees)
  horizontalServo.write(vServoPos);
  verticalServo.write(hServoPos);


  // Ultrasonic sensor code
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // LDR sensors code
  // Initialize the LDR pins as input
  pinMode(ldr1Pin, INPUT); // Bottom-Left
  pinMode(ldr2Pin, INPUT); // Top-Left
  pinMode(ldr3Pin, INPUT); // Top-Right 
  pinMode(ldr4Pin, INPUT); // Bottom-Right


  // Joystick related code 
  // Initialize the joystick switch pin as an input with internal pull-up resistor
  pinMode(jsSwitchPin, INPUT_PULLUP);

  // Attaching the debounce function to the switch
  debouncer.attach(jsSwitchPin);
  debouncer.interval(50); // interval in milliseconds

  // Active buzzer code
  pinMode(aBuzzerPin, OUTPUT);

  
  // Voltage sensor code
  pinMode(VSensorPin, INPUT);

  // Current sensor code
  pinMode(ASensorPin, INPUT);



}

void loop() 
{

  // Joystick switch vars
  int buttonReading = digitalRead(jsSwitchPin);
  // int switchState = debouncer.read();
  

  display.clearDisplay();
  
  // Updating the debouncing value
  debouncer.update();


  // Check if the button state changed to PRESSED
  if (debouncer.fell()) {
    toggleState = !toggleState; // Toggle the state
    manualSolarMode = !manualSolarMode; // Toggle the mode
  } 


  // Serial.println(manualSolarMode);
  // Serial.println(toggleState);


  // LDR with automatic servos movements
  autoSolarCtrlMode();
  // manualSolarCtrlMode();

  // Ultrasonic sensor function
  distanceFromObjectReader();


  //Temp + Humidity sensor function
  measureTemp();


  // Voltage sensor function
  measureVoltage();

  
  // Current sensor function
  measureCurrent();


  // Publish the info read within the system to the MQTT broker
  mqttPublishData();


  // Just a random red blinking LED function, you never know if there are airplanes in the area xD
  // blinkRedLed();


  display.display();

  // delay(500);

}

void mqttPublishData(){

  mqttClient.poll();

  unsigned long currentMillis = millis();

  // Publish temperature data
  if (currentMillis - previousMillisTemp >= intervalTemp) {
    previousMillisTemp = currentMillis;

    int temperatureValue = temperature; // Get the temperature measured

    Serial.print("Sending temperature to topic: ");
    Serial.println(tempTopic);
    Serial.println(temperatureValue);

    mqttClient.beginMessage(tempTopic, true, 0);
    mqttClient.print(temperatureValue);
    mqttClient.endMessage();
  }

  // Publish humidity data
  if (currentMillis - previousMillisHumidity >= intervalHumidity) {
    previousMillisHumidity = currentMillis;

    float humidityValue = humidity; // Get the humidity measured

    Serial.print("Sending data to topic: ");
    Serial.println(humidityTopic);
    Serial.println(humidityValue);

    mqttClient.beginMessage(humidityTopic, true, 0);
    mqttClient.print(humidityValue);
    mqttClient.endMessage();
  }

  // Publish if object is being detected
  if (currentMillis - previousMillisObjDetection >= intervalObjDetection) {
    previousMillisObjDetection = currentMillis;

    String objDetectionValue = isObstacleDetected; // Check for any collision possibilities 

    Serial.print("Sending data to topic: ");
    Serial.println(objectDetectionTopic);
    Serial.println(objDetectionValue);

    mqttClient.beginMessage(objectDetectionTopic, true, 0);
    mqttClient.print(objDetectionValue);
    mqttClient.endMessage();
  }

  // Publish solar current data
  if (currentMillis - previousMillisCurrent >= intervalCurrent) {
    previousMillisCurrent = currentMillis;

    float solarCurrentValue = current; // Get the humidity measured

    Serial.print("Sending data to topic: ");
    Serial.println(solarCurrentTopic);
    Serial.println(solarCurrentValue);

    mqttClient.beginMessage(solarCurrentTopic, true, 0);
    mqttClient.print(solarCurrentValue);
    mqttClient.endMessage();
  }

  // Publish solar voltage data
  if (currentMillis - previousMillisVoltage >= intervalVoltage) {
    previousMillisVoltage = currentMillis;

    float solarVoltageValue = voltage; // Actual voltage reading function

    Serial.print("Sending data to topic: ");
    Serial.println(solarVoltageTopic);
    Serial.println(solarVoltageValue);

    mqttClient.beginMessage(solarVoltageTopic, true, 0);
    mqttClient.print(solarVoltageValue);
    mqttClient.endMessage();
  }


  // Publish manual mode data
  if (currentMillis - previousMillisManualModeCheck >= intervalManualModeCheck) {
    previousMillisManualModeCheck = currentMillis;

    bool manualModeCheckValue = manualSolarMode; // Solar mode check function

    Serial.print("Sending data to topic: ");
    Serial.println(manualModeCheckTopic);
    Serial.println(manualModeCheckValue);

    mqttClient.beginMessage(manualModeCheckTopic, true, 0);
    mqttClient.print(manualModeCheckValue);
    mqttClient.endMessage();
  }
  
  delay(1);

}


void onMqttMessage(int messageSize) {
  
  Serial.print("Received a message with topic '");
  Serial.println(
    // mqttClient.messageTopic();
  );

  String message = "";
  
  while (mqttClient.available()) {
    message.concat((char)mqttClient.read());
  }
  
  Serial.println(message);

  // if (message == "on") {
  //   digitalWrite(lightPin, HIGH);
  // } else {
  //   digitalWrite(lightPin, LOW);
  // }

}   


void blinkRedLed() {

  digitalWrite(RedLed_pin, HIGH);
  // verticalServo.write(0);   // turn servo to 0 degrees
  // horizontalServo.write(0);   // turn servo to 0 degrees
  delay(50);        // wait a second
  digitalWrite(RedLed_pin, LOW);
  // verticalServo.write(190); // turn servo to 180 degrees
  // horizontalServo.write(190); // turn servo to 180 degrees
  delay(1500);        // wait a second

}

void measureTemp() {

  temperature = dht11.readTemperature();
  humidity = dht11.readHumidity();

  // Checking if the sensor has any issues or errors before proceeding with the measurement
  if (temperature != DHT11::ERROR_CHECKSUM && temperature != DHT11::ERROR_TIMEOUT &&
        humidity != DHT11::ERROR_CHECKSUM && humidity != DHT11::ERROR_TIMEOUT)
    {
        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.println(" °C");

        Serial.print("Humidity: ");
        Serial.print(humidity);
        Serial.println(" %");


        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(0, 20);
        display.print("Temp: ");
        display.print(temperature);
        display.println("C");

        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(0, 30);
        display.print("Humidity: ");
        display.print(humidity);
        display.println("%");

    }
    else
    {
        if (temperature == DHT11::ERROR_TIMEOUT || temperature == DHT11::ERROR_CHECKSUM)
        {
            Serial.print("Temperature Reading Error: ");
            Serial.println(DHT11::getErrorString(temperature));
        }
        if (humidity == DHT11::ERROR_TIMEOUT || humidity == DHT11::ERROR_CHECKSUM)
        {
            Serial.print("Humidity Reading Error: ");
            Serial.println(DHT11::getErrorString(humidity));
        }
    }

}


void measureVoltage() {

  int rawVoltage = mcp.readADC(VSensorPin);
  int offset = 100;
  // Arduino analog only supports 10 bits which is in total 1024 values from 0 up to 1024. Below we are modifying that value and 
  // mapping it to the voltage value we are going to measure up to which is 25v (2500mv), the sensors maximum and add correction offset. 
  double voltage = map(rawVoltage, 0, 1023, 0, 2500) - offset;

  voltage /=100; // Divide the value measured by 100 to get the exact decimal voltage value

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 40);
  display.print("Voltage: ");
  display.print(voltage);
  display.println("V");

}

void measureCurrent() {

  int rawCurrent = mcp.readADC(ASensorPin);
  // ACS712 30A: Output is 66mV/A. 2.5V (centered) when no current flows
  float sensorVoltage = (rawCurrent / 1023.0) * 5.0;
  float current = (sensorVoltage - zeroCurrentVoltage) / 0.066; // Convert to current in Amps according to a 30 amps sensor. The 5 amps vrsion should be 0.185 according to the documentation.
  

  if (current < 0.00) {
    current = 0.00;
  }

  // Display the measured values accordingly
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 50);
  display.print("Current: ");
  display.print(current);
  display.println("A");
  Serial.println(current);


}


void distanceFromObjectReader() {

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);


  // The pulseIn function will make sure to process the bounce of the sound wave received back
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2; 

  Serial.print("Distance: ");
  Serial.println(distance);


  // display.setTextSize(1);
  // display.setTextColor(WHITE);
  // display.setCursor(0, 10);
  // display.print("Obj distance: ");
  // display.print(distance);
  // display.println(" cm");




  // Active buzzer code
  // If the distance detected on the ultrasonic sensor is less than 5 cm then the buzzer will be activated to sound an alarm
  if (distance < 6) {
    int buzzerDelay = 100; // Set this to the desired delay for the buzzer
    digitalWrite(aBuzzerPin, HIGH);
    digitalWrite(YellowLed_pin, HIGH);
    delay(buzzerDelay);
    digitalWrite(aBuzzerPin, LOW);
    digitalWrite(YellowLed_pin, LOW);
    delay(buzzerDelay);

    isObstacleDetected = "true";
  } else {
    isObstacleDetected = "false";
    digitalWrite(aBuzzerPin, LOW);
    digitalWrite(YellowLed_pin, LOW);
  }
}

void autoSolarCtrlMode() {

  if (isObstacleDetected != "true" && toggleState != true) {
    manualSolarMode = false;

    int LDR1Value = analogRead(ldr1Pin); // Bottom-Left
    Serial.print("LDR 1: ");
    Serial.println(LDR1Value);
    int LDR2Value = analogRead(ldr2Pin); // Top-Left
    Serial.print("LDR 2: ");
    Serial.println(LDR2Value);
    int LDR3Value = analogRead(ldr3Pin); // Top-Right 
    Serial.print("LDR 3: ");
    Serial.println(LDR3Value);
    int LDR4Value = analogRead(ldr4Pin); // Bottom-Right
    Serial.print("LDR 4: ");
    Serial.println(LDR4Value);



    // Analog method
    // Calculate the average values for horizontal and vertical directions

    int rightAvg = (LDR3Value + LDR4Value) / 2; // Top-Right + Bottom-Right / Right Average
    Serial.print("Average Right: ");
    Serial.println(rightAvg);
    int leftAvg = (LDR2Value + LDR1Value) / 2; // Top-Left + Bottom-Left / Left Average
    Serial.print("Average Left: ");
    Serial.println(leftAvg);
    int topAvg = (LDR2Value + LDR3Value) / 2; // Top-Left + Top-Right / Top Average
    Serial.print("Average Top: ");
    Serial.println(topAvg);
    int bottomAvg = (LDR1Value + LDR4Value) / 2; // Bottom-Left + Bottom-Right / Bottom Average
    Serial.print("Average Bottom: ");
    Serial.println(bottomAvg);



    // Get the difference  
    int diff_top_bottom = topAvg - bottomAvg;   //Get the difference average betwen LDRs top and LDRs bot
    int diff_right_left = rightAvg - leftAvg;    //Get the difference average betwen LDRs right and LDRs left


    // V3.0

    if (abs(diff_right_left) >= LDRThreshold){        //Change position only if light difference is bigger then the LDRThreshold
      if  (diff_right_left > 0) {
        if (hServoPos < 180) {
          horizontalServo.write(hServoPos  + 2);
        }
      }
      if (diff_right_left <  0) {
        if (hServoPos > 0) {
          horizontalServo.write(hServoPos - 2);
        }
      }
    }
              
        //up-down movement of solar tracker

    // if (abs(diff_top_bottom) >= LDRThreshold){    //Change position only if light difference is bigger then LDRThreshold
    //   if (diff_top_bottom > 0) {
    //     if  (verticalServo.read() < 180) {
    //       verticalServo.write((verticalServo.read()  - 2));
    //     }
    //   }
    //   if (diff_top_bottom <  0) {
    //     if (verticalServo.read()  > 0) {
    //       verticalServo.write((verticalServo.read() + 2));
    //     }
    //   }
    // }       


  // if (topAvg < bottomAvg) {

  //   verticalServo.write(vServoPos - 1);
  //   if (vServoPos > vServoLimitHigh) {
  //     vServoPos = vServoLimitHigh;
  //   }
  //   delay(8);
  // } else if (bottomAvg < topAvg) {

  //   verticalServo.write(vServoPos + 1);
  //   if (vServoPos > vServoLimitLow) {
  //     vServoPos = vServoLimitLow;
  //   }
  //   delay(8);
  // } else {
  //   verticalServo.write(vServoPos);
  // }

  // if (leftAvg > rightAvg) {
  //   horizontalServo.write(hServoPos - 1);
  //   if (hServoPos > hServoLimitHigh) {
  //     hServoPos = hServoLimitHigh;
  //   }
  //   delay(8);
  // } else if (rightAvg > leftAvg) {
  //   horizontalServo.write(hServoPos + 1);
  //   if (hServoPos > hServoLimitLow) {
  //     hServoPos = hServoLimitLow;
  //   }
  //   delay(8);
  // } else {
  //   horizontalServo.write(hServoPos);
  // }

    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    display.print("Manual Mode: ");
    display.println("False");


    

  } else {
    Serial.println("Manual Mode");
    manualSolarMode = true;

    manualSolarCtrlMode();

    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    display.print("Manual Mode: ");
    display.println("True");

    
  }


  // delay(200);
}


void manualSolarCtrlMode() {

  // if (manualSolarMode == true) {
    // Joystick values
    int xJoyVal = mcp.readADC(joyYPin);
    int yJoyVal = mcp.readADC(joyXPin);


    // Print the values to the serial monitor
    Serial.print("X: ");
    Serial.print(xJoyVal);
    Serial.print(" Y: ");
    Serial.println(yJoyVal);
    // Serial.println(lastButtonState);


    // Check if the joystick is moved out of the neutral zone
    if (abs(yJoyVal - neutralY) > deadzone) {
      hServoPos = map(yJoyVal, 0, 1023, 0, 180); // Servo value range switch from 0-1023 to 0-180
      horizontalServo.write(hServoPos); // Set the position of servo 1 according the joystick
    }

    if (abs(xJoyVal - neutralX) > deadzone) {
      vServoPos = map(xJoyVal, 0, 1023, 0, 180); // Servo value range switch from 0-1023 to 0-180
      verticalServo.write(vServoPos); // Set the position of servo 2 according the joystick
    }
  // } else {
  //   autoSolarCtrlMode();
  // }
}


// void logData() {
//   // Check if the file exists
//   if (!SD.exists("datalog.csv")) {
//     // If the file doesn't exist, create it and add the header
//     File dataFile = SD.open("datalog.csv", FILE_WRITE);
//     if (dataFile) {
//       dataFile.println("Time, Voltage(V), Current(A), Temperature(C)");
//       dataFile.close();
//     }
//     else {
//       display.println("Error opening datalog.csv");
//       display.display();
//       return;
//     }
//   }

//   // Open the file and append the data
//   File dataFile = SD.open("datalog.csv", FILE_WRITE);
//   if (dataFile) {
//     dataFile.print(millis());
//     dataFile.print(", ");
//     dataFile.print(voltage);
//     dataFile.print(", ");
//     dataFile.print(current);
//     dataFile.print(", ");
//     dataFile.println(temperature);
//     dataFile.close();
//   }
//   else {
//     display.println("Error opening datalog.csv");
//     display.display();
//   }
// }


// void LDRTest(){
    
//   // LDR sensor when detecting light, it turns on the blue led

//   if (digitalRead(ldrPin) == 1) {
//     digitalWrite(BlueLed_pin, LOW);

//     display.setTextSize(1);
//     display.setTextColor(WHITE);
//     display.setCursor(0, 40);
//     display.print("Light: ");
//     display.println("False");
    
//   } else {
//     digitalWrite(BlueLed_pin, HIGH);

//     display.setTextSize(1);
//     display.setTextColor(WHITE);
//     display.setCursor(0, 40);
//     display.print("Light: ");
//     display.println("True");
//   }

//   Serial.println(digitalRead(ldrPin));  
// }