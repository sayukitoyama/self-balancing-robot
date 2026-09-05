#include <Wire.h>

const int MPU_ADDR = 0x68;

int accX, accY, accZ;
int gyroX, gyroY, gyroZ;

float angle = 0;  //robot tilt angle in degrees 
float gyroOffset = 0;  //offset of gyroscope

//Values of the pins
const int PWMA = 25;
const int AIN1 = 26;
const int AIN2 = 27;
const int PWMB = 33;
const int BIN1 = 32;
const int BIN2 = 23;
const int STBY = 14;

float targetAngle = 0;  //what the PID control will angle at
float pro = 50;  //The P of PID
float inte = 0.5;  //The I
float der = 1.75;  //The D
float integral = 0;  //Value for the integral to add the erorr up
float fallAngle = 13;  //The max angle the robot can turn until it hits the ground
int maxP = 255;  //The max power (The motor driver can reach)
int minP = 10;  //The min power (The motor driver can reach)
long lastTime = 0;

void setup() 
{

  Serial.begin(115200);
  setupMotors();
  Wire.begin(21, 22);
  Wire.setClock(50000);
  Wire.setTimeOut(10);  //timeout if MPU does respond after 10ms just for safety
  setupMPU();
  delay(300);  //sometimes was bugging out without delay
  calibrateGyro();

  readMPU();
  angle = getAng();  //gets the initial angle

  lastTime = micros();

  Serial.println("start");

}

void loop()
{
  
  long now = micros();
  float dt = (now - lastTime) / 1000000.0;  //gets time it took from running the last loop function in seconds
  lastTime = now;
  readMPU();
  float accAngle = getAng();
  float gyroRate = (gyroX / 131.0) - gyroOffset;
  angle = 0.98 * (angle + gyroRate * dt) + 0.02 * accAngle;  //uses both data from gyro and accelerometer to find a more accurate angle of the robot

  if (abs(angle) > fallAngle)  //stops if the robot tilts too much
  {
    integral = 0;
    stopMotors();
    return;
  }

  float error = targetAngle - angle;
  float P = pro * error;
  integral += error * dt;
  float I = inte * integral;
  float D = -der * gyroRate;

  float output = P + I + D;

  output = constrain(output, -maxP, maxP);  //limits it between the actual value the motor driver can go

  setBothMotors((int)output);

}

void setupMPU()
{

  Wire.beginTransmission(MPU_ADDR);  //wake MPU from sleep mode
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);  

  Wire.beginTransmission(MPU_ADDR);  //setup gyro to +-250 degrees
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);  //set up accelerometer to +-2g
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission(true);

}

void readMPU() {

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  
  accX = Wire.read() << 8 | Wire.read();
  accY = Wire.read() << 8 | Wire.read();
  accZ = Wire.read() << 8 | Wire.read();

  Wire.read();  //skips temperature reading
  Wire.read();

  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();

}

float getAng() 
{

  return atan2(accY, accZ) * 180.0 / PI;
}

void calibrateGyro()
{

  float sum = 0;

  const int samples = 1000;

  Serial.println("keep robot still...");

  for (int i = 0; i < samples; i++) 
  {

    readMPU();
    sum += gyroX / 131.0;
    delay(2);

  }

  gyroOffset = sum / samples;

  Serial.print("Gyro offset: ");
  Serial.println(gyroOffset);

}

void setupMotors()
{

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);

  ledcAttach(PWMA, 20000, 8);
  ledcAttach(PWMB, 20000, 8);

  stopMotors();
}

void setBothMotors(int power)
{

  power = constrain(power, -255, 255);

  if (abs(power) < minP) {
    power = 0;
  }

  setLeftMotor(power);
  setRightMotor(-power);  //This is reversed becaused I made it so that the motors are not facing the same direction

}

void setLeftMotor(int power)
{

  power = constrain(power, -255, 255);

  if (power > 0)
  {

    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    ledcWrite(PWMA, power);

  } 
  else if (power < 0)
  {

    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    ledcWrite(PWMA, -power);

  }
  else
  {

    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    ledcWrite(PWMA, 0);
  
  }
}

void setRightMotor(int power) {

  power = constrain(power, -255, 255);

  if (power > 0) 
  {

    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    ledcWrite(PWMB, power);

  } 
  else if (power < 0)
  {

    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    ledcWrite(PWMB, -power);

  }
  else
  {

    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    ledcWrite(PWMB, 0);

  }
}

void stopMotors() 
{

  setLeftMotor(0);
  setRightMotor(0);

}