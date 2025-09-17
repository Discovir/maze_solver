#include <Wire.h>
#include <MPU6050.h>

// Motor pins
#define ENA 27
#define IN1 25
#define IN2 26
#define ENB 18
#define IN3 19
#define IN4 23

// IMU
MPU6050 mpu;

// Variables for angle calculation
float gyroZ;
float angle = 0;
unsigned long lastTime = 0;
float dt;

// PID variables
float targetAngle = 90.0; // Negative because right turn gives negative gyro values
float error, previousError = 0;
float integral = 0;
float derivative = 0;

// PID constants - tune these for your robot
float Kp = 2.5;   // Reduced proportional gain
float Ki = 0.0;   // Set to 0 as requested
float Kd = 1.2;   // Increased derivative gain for better braking

// Motor control variables
int baseSpeed = 100; // Reduced base speed
int maxSpeed = 160;  // Maximum motor speed
int minSpeed = 40;   // Minimum motor speed to overcome friction
bool turnComplete = false;

// Tolerance for considering turn complete
float angleTolerance = 0.8; // Tighter tolerance

// Predictive braking variables
float brakingZone = 15.0; // Start slowing down 15 degrees before target
float currentAngularVelocity = 0;

void setup() {
  Serial.begin(115200);
  
  // Initialize motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Initialize I2C
  Wire.begin(21, 22); // SDA=21, SCL=22
  
  // Initialize MPU6050
  mpu.initialize();
  
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while(1);
  }
  
  Serial.println("MPU6050 connected successfully");
  
  // Calibrate gyroscope (keep robot still during this)
  Serial.println("Calibrating gyroscope... Keep robot still!");
  delay(2000);
  
  // Reset variables
  angle = 0;
  integral = 0;
  previousError = 0;
  lastTime = micros();
  
  Serial.println("Starting 90-degree right turn with predictive braking...");
  delay(1000);
}

void loop() {
  if (!turnComplete) {
    // Read gyroscope data
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    
    // Convert to degrees per second (MPU6050 default sensitivity is 131 LSB/°/s)
    gyroZ = gz / 131.0;
    currentAngularVelocity = abs(gyroZ);
    
    // Calculate time difference
    unsigned long currentTime = micros();
    dt = (currentTime - lastTime) / 1000000.0; // Convert to seconds
    lastTime = currentTime;
    
    // Integrate gyroscope data to get angle (negative for right turn)
    angle += gyroZ * dt;
    
    // Calculate PID error
    error = targetAngle - angle;
    
    // Check if turn is complete
    if (abs(error) <= angleTolerance) {
      stopMotors();
      turnComplete = true;
      Serial.println("90-degree right turn completed!");
      Serial.print("Final angle: ");
      Serial.println(angle);
      Serial.print("Final error: ");
      Serial.println(error);
    } else {
      // Calculate PID components
      derivative = (error - previousError) / dt;
      previousError = error;
      
      // Calculate base PID output
      float pidOutput = (Kp * error) + (Kd * derivative);
      
      // Apply predictive braking
      float distanceToTarget = abs(error);
      float speedMultiplier = 1.0;
      
      if (distanceToTarget <= brakingZone) {
        // Calculate braking factor based on distance and velocity
        float brakingFactor = distanceToTarget / brakingZone;
        float velocityFactor = 1.0 - (currentAngularVelocity / 200.0); // Brake harder if moving fast
        velocityFactor = constrain(velocityFactor, 0.3, 1.0);
        
        speedMultiplier = brakingFactor * velocityFactor;
        speedMultiplier = constrain(speedMultiplier, 0.3, 1.0);
      }
      
      // Convert PID output to motor speed with braking
      int motorSpeed = constrain(abs(pidOutput) * speedMultiplier, minSpeed, maxSpeed);
      
      // Further reduce speed when very close to target
      if (distanceToTarget <= 5.0) {
        motorSpeed = constrain(motorSpeed, minSpeed, 80);
      }
      
      // Determine turn direction based on error
      if (error < -angleTolerance) {
        // Need to turn more right (angle is not negative enough)
        turnRight(motorSpeed);
      } else if (error > angleTolerance) {
        // Overshot, turn slightly left to correct
        turnLeft(motorSpeed);
      } else {
        stopMotors();
      }
      
      // Print status every 50ms for better monitoring
      static unsigned long printTime = 0;
      if (millis() - printTime > 50) {
        Serial.print("Angle: ");
        Serial.print(angle);
        Serial.print(" | Error: ");
        Serial.print(error);
        Serial.print(" | Speed: ");
        Serial.print(motorSpeed);
        Serial.print(" | AngVel: ");
        Serial.print(currentAngularVelocity);
        Serial.print(" | Brake: ");
        Serial.println(speedMultiplier);
        printTime = millis();
      }
    }
  }
  
  delay(5); // Small delay for stability
}

void turnRight(int speed) {
  // Left motor forward, right motor backward for right turn
  // Fixed IN3 and IN4 logic as requested
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);
  
  digitalWrite(IN4, HIGH);  // Reversed logic
  digitalWrite(IN3, LOW);   // Reversed logic
  analogWrite(ENB, speed);
}

void turnLeft(int speed) {
  // Left motor backward, right motor forward for left turn (correction)
  // Fixed IN3 and IN4 logic as requested
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);
  
  digitalWrite(IN4, LOW);   // Reversed logic
  digitalWrite(IN3, HIGH);  // Reversed logic
  analogWrite(ENB, speed);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}