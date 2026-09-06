
#include <Arduino.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <Servo.h>
#include <Arduino_BMI270_BMM150.h> // Include Rev2 IMU Library

static const size_t LINE_MAX = 16;    // max length of command (excluding the terminate character '\0')
static char   lineBuf[LINE_MAX + 1];
static size_t lineLen = 0;
static const char SOC = '@';          // start-of-command marker
static bool   inCommand = false;
Servo Steering_Servo;
Servo lizardESC;
// --- IMU Variables ---
float heading = 0.0;
float gyroZBias = 0.0;
unsigned long lastTime;

void SetRGBLED(int state)//0 - Off; 1 - RED; 2 - Green; 3 - Blue
{
  if(state == 0)
  {
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);
  }
  else if(state == 1)
  {
    digitalWrite(LEDR, LOW);// Red ON
    digitalWrite(LEDG, HIGH);// Green OFF
    digitalWrite(LEDB, HIGH);// Blue OFF
  }
  else if(state == 2)
  {
    digitalWrite(LEDR, HIGH);// Red OFF
    digitalWrite(LEDG, LOW);// Green ON
    digitalWrite(LEDB, HIGH);// Blue OFF
  }
  else if(state == 3)
  {
    digitalWrite(LEDR, HIGH);// Red OFF
    digitalWrite(LEDG, HIGH);// Green OFF
    digitalWrite(LEDB, LOW);// Blue ON
  }
  else if(state == 4)
  {
    digitalWrite(LEDR, LOW);// Red ON
    digitalWrite(LEDG, LOW);// Green ON
    digitalWrite(LEDB, HIGH);// Blue OFF
  }
  else if(state == 5)
  {
    digitalWrite(LEDR, HIGH);// Red OFF
    digitalWrite(LEDG, LOW);// Green ON
    digitalWrite(LEDB, LOW);// Blue ON
  }
  else if(state == 6)
  {
    digitalWrite(LEDR, LOW);// Red ON
    digitalWrite(LEDG, HIGH);// Green OFF
    digitalWrite(LEDB, LOW);// Blue ON
  }
  else if(state == 7)
  {
    digitalWrite(LEDR, LOW);// Red ON
    digitalWrite(LEDG, LOW);// Green ON
    digitalWrite(LEDB, LOW);// Blue ON
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);
  Steering_Servo.attach(3);
  lizardESC.attach(4);
  lizardESC.writeMicroseconds(1500); // 1500us is typically neutral
  delay(2000);

  // Initialize the RGB pins as outputs
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);

  // Turn all colors OFF immediately at startup
  SetRGBLED(0);

  // Initialize the Rev2 IMU
  if (!IMU.begin()) { //Failed to initialize Rev2 IMU!
    SetRGBLED(1);
    while (1); 
  }

  // Gyro Calibration: Keep the car perfectly still when booting up!
  SetRGBLED(3);
  int samples = 500;
  float sum = 0;
  int validSamples = 0;
  for (int i = 0; i < samples; i++) {
    if (IMU.gyroscopeAvailable()) {
      float x, y, z;
      IMU.readGyroscope(x, y, z);
      sum += z;
      validSamples++;
    }
    delay(10);
  }
  gyroZBias = (validSamples > 0) ? sum / validSamples : 0.0;
  
  //Calibration Complete.
  SetRGBLED(2);
  delay(1000);
  SetRGBLED(0);
  
  lastTime = micros();
}

static void handleCommand(char* command) {
  if (!command || command[0] == '\0') return;

  // If the line includes '\r' (CRLF), terminate at it
  char* cr = strchr(command, '\r');
  if (cr) *cr = '\0';

  // Trim leading spaces
  while (*command && isspace((unsigned char)*command)) command++;
  if (*command == '\0') return;

  // Command type
  char type = *command++;

  // --- Handle Heading request immediately before integer parsing ---
  if (type == 'H') {
    Serial.println(heading);
    return;
  }

  // Parse integer (For S and M commands)
  char* endp = nullptr;
  long value = strtol(command, &endp, 10);

  // Must have at least one digit for S and M commands
  if (endp == command) return;

  if (type == 'S') {
    if (value < 0 || value > 180) return;

      Steering_Servo.write(value);

  } else if (type == 'M') {
    if (value < 1000 || value > 2000) return;
      lizardESC.writeMicroseconds(value);
  } else if (type == 'L') {
    if (value < 0 || value > 7) return;
      SetRGBLED(value);
  }
}

void loop() {
  // 1. Read exactly ONE available byte per loop execution pass
  if (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == SOC) {
      // Start a fresh command tracking window
      inCommand = true;
      lineLen = 0;
    } 
    else if (inCommand) {
      // Only process characters if we have successfully parsed a valid SOC ('@')
      if (c == '\n') {
        lineBuf[lineLen] = '\0';
        handleCommand(lineBuf);
        inCommand = false;
        lineLen = 0;
      } 
      else if (lineLen < LINE_MAX) {
        lineBuf[lineLen++] = c;
      } 
      else {
        // Buffer Overflow protection: dump frame, reset, wait for next '@'
        inCommand = false;
        lineLen = 0;
      }
    }
  }

  // =======================================================================
  // 2. Background Time-Critical Work: IMU Tracking
  // =======================================================================
  if (IMU.gyroscopeAvailable()) {
    float x, y, z;
    IMU.readGyroscope(x, y, z);

    // Remove baseline sensor noise drift
    z -= gyroZBias;

    // Ignore micro-vibrations below a small deadzone threshold
    if (abs(z) < 0.1) z = 0.0; 

    // Compute elapsed time (dt) in seconds
    unsigned long currentTime = micros();
    float dt = (currentTime - lastTime) / 1000000.0;
    lastTime = currentTime;

    // Integrate angular velocity over time
    heading -= z * dt; 
  }
}
