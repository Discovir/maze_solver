#include <Wire.h>
#include <MPU6050.h>

// Motor Driver Pins (L298N)
#define ENA 27  // Left motor PWM
#define IN1 25  // Left motor direction
#define IN2 26  // Left motor direction
#define ENB 18  // Right motor PWM
#define IN3 19  // Right motor direction
#define IN4 23  // Right motor direction

// IMU
MPU6050 mpu;

// Movement parameters
const int BASE_SPEED = 120;  // Slower speed for better control
const float CORRECTION_FACTOR = 2.5;  // Stronger correction for better response
const unsigned long MOVE_DURATION = 800;  // Move for 1 second

// Braking parameters
const int BRAKE_STRENGTH = 80;  // Reverse PWM strength (0-255)
const int BRAKE_DURATION = 60;  // Milliseconds of reverse thrust
const int SETTLE_TIME = 100;    // Time to wait after braking

// IMU variables
float currentAngle = 0;
float targetAngle = 0;
unsigned long lastTime = 0;
float gyroZ_offset = 0;

// Motor calibration - ADJUST THESE IF NEEDED
const float MOTOR_BIAS = 1.08;  // Increased to compensate for left drift
                                 // This makes the left motor slightly faster

void setup() {
  Serial.begin(115200);
  
  // Initialize motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Stop motors initially
  stopMotors();
  delay(100);
  
  // Initialize I2C
  Wire.begin(21, 22);  // SDA, SCL
  
  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }
  Serial.println("MPU6050 connected!");
  
  // Calibrate gyro (robot should be stationary)
  calibrateGyro();
  
  // Wait a bit before starting
  delay(2000);
  Serial.println("Starting straight movement...");
  
  // Reset angle tracking
  currentAngle = 0;
  targetAngle = 0;
  lastTime = millis();
  
  // Start moving straight with soft start
  moveStraightWithCorrection(MOVE_DURATION);
  
  // Advanced braking sequence
  activeBraking();
  
  Serial.println("Movement complete!");
}

void loop() {
  // Nothing to do here
}

void calibrateGyro() {
  Serial.println("Calibrating gyro... Keep robot still!");
  
  long sumZ = 0;
  int samples = 1000;
  
  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sumZ += gz;
    delay(3);
  }
  
  gyroZ_offset = (float)sumZ / samples;
  Serial.print("Gyro Z offset: ");
  Serial.println(gyroZ_offset);
}

void updateAngle() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;  // Convert to seconds
  
  if (dt > 0) {  // Only update if time has passed
    lastTime = currentTime;
    
    // Read gyro
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // Apply offset and convert to degrees/second
    float gyroZ = (gz - gyroZ_offset) / 131.0;  // 131 LSB/°/s for ±250°/s range
    
    // Only integrate significant rotations to reduce drift
    if (abs(gyroZ) > 1.0) {  // Increased threshold
      currentAngle += gyroZ * dt;
    }
  }
}

void moveStraightWithCorrection(unsigned long duration) {
  unsigned long startTime = millis();
  unsigned long elapsedTime = 0;
  
  // Start both motors forward with soft start
  setMotorDirectionForward();
  
  while (elapsedTime < duration) {
    elapsedTime = millis() - startTime;
    
    // Update current angle from IMU
    updateAngle();
    
    // Calculate error
    float error = currentAngle - targetAngle;
    
    // Calculate correction with smaller deadband for tighter control
    int correction = 0;
    if (abs(error) > 0.5) {  // More sensitive to small errors
      correction = (int)(error * CORRECTION_FACTOR);
    }
    
    // Limit correction to prevent overcorrection
    correction = constrain(correction, -80, 80);
    
    // Soft start/stop logic
    int currentBaseSpeed = BASE_SPEED;
    
    // Soft start: ramp up speed in first 100ms
    if (elapsedTime < 100) {
      currentBaseSpeed = map(elapsedTime, 0, 100, 60, BASE_SPEED);
    }
    // Soft stop: ramp down in last 100ms
    else if (elapsedTime > duration - 100) {
      currentBaseSpeed = map(elapsedTime, duration - 100, duration, BASE_SPEED, 60);
    }
    
    // Calculate motor speeds with bias compensation
    int leftSpeed = currentBaseSpeed * MOTOR_BIAS + correction;
    int rightSpeed = currentBaseSpeed / MOTOR_BIAS - correction;
    
    // Ensure minimum speed to keep moving forward
    leftSpeed = constrain(leftSpeed, 40, 200);
    rightSpeed = constrain(rightSpeed, 40, 200);
    
    // Apply speeds to motors (only PWM, direction already set)
    analogWrite(ENA, leftSpeed);
    analogWrite(ENB, rightSpeed);
    
    // Debug output
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 100) {  // Print every 100ms
      Serial.print("Time: ");
      Serial.print(elapsedTime);
      Serial.print("/");
      Serial.print(duration);
      Serial.print(" ms | Angle: ");
      Serial.print(currentAngle, 1);
      Serial.print("° | Error: ");
      Serial.print(error, 1);
      Serial.print("° | L: ");
      Serial.print(leftSpeed);
      Serial.print(" | R: ");
      Serial.println(rightSpeed);
      lastPrint = millis();
    }
    
    delay(10);  // Control loop delay
  }
}

void activeBraking() {
  Serial.println("Active braking...");
  
  // Method 1: Reverse thrust (most effective)
  // Apply reverse direction with controlled power
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  
  // Apply brake power with bias compensation
  analogWrite(ENA, BRAKE_STRENGTH * MOTOR_BIAS);
  analogWrite(ENB, BRAKE_STRENGTH / MOTOR_BIAS);
  
  // Hold reverse thrust briefly
  delay(BRAKE_DURATION);
  
  // Method 2: Short brake (motor terminals shorted)
  // This creates electromagnetic braking
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
  
  delay(30);  // Brief short brake
  
  // Finally, complete stop
  stopMotors();
  
  // Let robot settle
  delay(SETTLE_TIME);
}

void setMotorDirectionForward() {
  // Set both motors to forward
  // Left motor forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  
  // Right motor forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stopMotors() {
  // Full electrical stop - all pins LOW
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}