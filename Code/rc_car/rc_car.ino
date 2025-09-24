#include <Ps3Controller.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

// Servo setup
Servo servo;
const int servoPin = 13; // GPIO 13 for MG996R signal
const int ledPin = 2;    // ESP32 onboard LED
float servoPos = 90;     // Current servo position
float smoothedPos = 90;  // Smoothed servo position
const float alpha = 0.2; // Smoothing factor (0.0 to 1.0, lower = smoother)
const int minAngle = 20;  // Servo range: 20 degrees
const int maxAngle = 160; // Servo range: 160 degrees
unsigned long lastServoUpdate = 0; // Last servo update time
const int servoUpdateInterval = 20; // Update servo every 20ms to reduce jitter
const int servoDeadband = 2; // Deadband (degrees) to ignore small changes
const int lxCenterThreshold = 5; // LX range for centering (-5 to 5)

// Cytron MD10C pin assignments
const int dirPin = 4;  // GPIO 4 for MD10C DIR (HIGH = forward, LOW = reverse)
const int pwmPin = 18; // GPIO 18 for MD10C PWM
const int pwmFreq = 500; // 500 Hz PWM frequency to avoid servo interference
const int pwmResolution = 8; // 8-bit resolution (0-255)
const int pwmChannel = 2; // PWM channel for motor

// INA219 setup
Adafruit_INA219 ina219; // Default address 0x40
bool ina219Connected = false; // Tracks INA219 connection status
unsigned long lastINA219Update = 0; // Last INA219 reading time
const int ina219UpdateInterval = 500; // Update INA219 every 500ms

// Motor control variables
int motorSpeed = 0; // Current motor speed (0 to 255, positive = forward, negative = reverse)

// Reset control
unsigned long startButtonPressTime = 0; // Time when Start button was pressed
const unsigned long resetHoldTime = 1000; // Hold Start for 1s to reset

void setupMotors() {
  // Configure MD10C pins
  pinMode(dirPin, OUTPUT);
  
  // Setup PWM channel
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(pwmPin, pwmChannel);
  
  // Initialize motor off
  digitalWrite(dirPin, LOW);
  ledcWrite(pwmChannel, 0);
  Serial.println("MD10C motor initialized (off)");
}

void driveMotors(int speed) {
  if (speed > 0) {
    // Forward direction
    digitalWrite(dirPin, HIGH);
    ledcWrite(pwmChannel, speed);
    Serial.print("Driving motor forward at speed: ");
    Serial.println(speed);
  } else if (speed < 0) {
    // Reverse direction
    digitalWrite(dirPin, LOW);
    ledcWrite(pwmChannel, -speed); // Use absolute value for PWM
    Serial.print("Driving motor reverse at speed: ");
    Serial.println(-speed);
  } else {
    // Stop motor
    digitalWrite(dirPin, LOW);
    ledcWrite(pwmChannel, 0);
    // Serial.println("Motor stopped");
  }
}

void readINA219() {
  if (ina219Connected) {
    float busVoltage = ina219.getBusVoltage_V();
    float current_mA = ina219.getCurrent_mA();
    
    Serial.print("Bus Voltage: ");
    Serial.print(busVoltage);
    Serial.print(" V | Current: ");
    Serial.print(current_mA);
    Serial.println(" mA");
  }
}

void notify() {
  unsigned long now = millis();
  int lx = Ps3.data.analog.stick.lx; // Left analog stick X-axis (-128 to 127)
  int r1Pressure = Ps3.data.analog.button.r1; // R1 pressure (0 to 255)
  int r2Pressure = Ps3.data.analog.button.r2; // R2 pressure (0 to 255)
  int l2Pressure = Ps3.data.analog.button.l2; // L2 pressure (0 to 255, brake)
  bool startPressed = Ps3.data.button.start; // Start button (1 = released, 0 = pressed)
  
  // Handle reset with Start button (1s hold)
  if (startPressed) {
    if (startButtonPressTime == 0) {
      startButtonPressTime = now; // Start timing
      Serial.println("Start button pressed, hold for 1s to reset");
    } else if (now - startButtonPressTime >= resetHoldTime) {
      Serial.println("Resetting ESP32...");
      driveMotors(0); // Stop motor before reset
      servo.write(90); // Center servo
      delay(100); // Ensure outputs settle
      ESP.restart(); // Reset ESP32
    }
  } else {
    startButtonPressTime = 0; // Reset timer if button released
  }
  
  // Servo control (update every 20ms to reduce jitter)
  if (now - lastServoUpdate >= servoUpdateInterval) {
    if (abs(lx) <= lxCenterThreshold) {
      // Force exact 90° when LX is near center
      servoPos = 90;
      smoothedPos = 90;
    } else {
      // Map LX to servo angle (LX: -128 → 20°, 127 → 160°)
      servoPos = map(lx, -128, 127, minAngle, maxAngle);
      smoothedPos = (alpha * servoPos) + ((1 - alpha) * smoothedPos);
    }
    int finalPos = constrain((int)smoothedPos, minAngle, maxAngle);
    if (abs(finalPos - servo.read()) > servoDeadband) {
      servo.write(finalPos);
      Serial.print("LX: ");
      Serial.print(lx);
      Serial.print(" | Servo Angle: ");
      Serial.println(finalPos);
    }
    lastServoUpdate = now;
  }
  
  // Motor control (pressure-sensitive, L2 brakes)
  if (l2Pressure > 0) {
    // Brake with L2 (overrides R1/R2)
    motorSpeed = 0;
    driveMotors(motorSpeed);
    Serial.print("Braking with L2, pressure: ");
    Serial.println(l2Pressure);
  } else if (r1Pressure > 0 && r2Pressure == 0) {
    // Forward with R1 pressure
    motorSpeed = r1Pressure;
    driveMotors(motorSpeed);
    Serial.print("R1 Pressure: ");
    Serial.print(r1Pressure);
    Serial.print(" | Motor Speed Forward: ");
    Serial.println(motorSpeed);
  } else if (r2Pressure > 0 && r1Pressure == 0) {
    // Reverse with R2 pressure
    motorSpeed = -r2Pressure; // Negative for reverse
    driveMotors(motorSpeed);
    Serial.print("R2 Pressure: ");
    Serial.print(r2Pressure);
    Serial.print(" | Motor Speed Reverse: ");
    Serial.println(-motorSpeed);
  } else {
    // Stop if no pressure or both R1/R2 pressed
    motorSpeed = 0;
    driveMotors(motorSpeed);
    if (r1Pressure > 0 && r2Pressure > 0) {
      Serial.println("Both R1 and R2 pressed | Motor stopped");
    } else {
      Serial.println("No pressure | Motor stopped");
    }
  }
  
  // INA219 readings (optional, only if connected)
  if (now - lastINA219Update >= ina219UpdateInterval) {
    readINA219();
    lastINA219Update = now;
  }
}

void onConnect() {
  digitalWrite(ledPin, HIGH); // Turn on onboard LED
  Ps3.setPlayer(1);           // Set PS3 controller to player 1
  Serial.println("PS3 Controller Connected. LED 1 set.");
}

void onDisconnect() {
  digitalWrite(ledPin, LOW); // Turn off onboard LED
  motorSpeed = 0;           // Reset speed
  driveMotors(0);           // Stop motor
  Serial.println("PS3 Controller Disconnected.");
}

void setup() {
  Serial.begin(115200);
  
  // Initialize onboard LED
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW); // LED off initially
  
  // Initialize servo
  ESP32PWM::allocateTimer(0); // Use timer 0 for servo to avoid motor PWM conflict
  servo.setPeriodHertz(50); // 50 Hz for servos
  servo.attach(servoPin, 500, 2500); // 500-2500 µs pulse width
  servo.write((int)smoothedPos); // Set initial position
  Serial.println("Servo initialized");
  
  // Initialize motor
  setupMotors();
  
  // Initialize INA219 (optional)
  if (ina219.begin()) {
    ina219Connected = true;
    Serial.println("INA219 initialized");
  } else {
    ina219Connected = false;
    Serial.println("INA219 not found, continuing without it");
  }
  
  // Initialize PS3 controller
  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.attachOnDisconnect(onDisconnect);
  Ps3.begin(); // Use default ESP32 Bluetooth MAC
  
  Serial.println("Ready.");
}

void loop() {
  // PS3 events handled by callbacks
}