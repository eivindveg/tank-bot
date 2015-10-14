#include <math.h>
#include <Makeblock.h>
#include <SoftwareSerial.h>
#include <Wire.h>

MeDCMotor MotorL(M1);
MeDCMotor MotorR(M2);
MeInfraredReceiver infraredReceiverDecode(PORT_6);
MeUltrasonicSensor UltrasonicSensor(PORT_3);

#define MAX_DIST 150
#define SENSOR_DIST 10

int moveSpeed = 190;
int turnRate = 2;
int incomingByte [2];

uint8_t mode = 0;


void setup()
{
  infraredReceiverDecode.begin();
  Serial.begin(9600);
  Wire.begin(8);
  Wire.onReceive(readBytes);
}

void loop()
{
  Serial.print("Left reading: ");
  Serial.println(incomingByte[0]);
  Serial.print("Right reading: ");
  Serial.println(incomingByte[1]);
    if (!isTrapped()) {
      int closest = min(incomingByte[0], incomingByte[1]);
      int farthest = max(incomingByte[0], incomingByte[1]);
      if (closest >= MAX_DIST - SENSOR_DIST) {
        Drive(0);
      } else if (farthest < 5) {
        Stop();

      }else if(farthest < 10) {
        if(incomingByte[0] > incomingByte[1]) {
          DriveAccurate(-1, 1);
        } else {
          DriveAccurate(1, -1);
        }
      } else {
        int sides[] = {SENSOR_DIST, incomingByte[0], incomingByte[1], SENSOR_DIST};
        double angle = calculateTurnAngle(sides);
        Drive(angle);
        Serial.println(angle);
      }
    } else {
      Stop();
    }
  delay(50);

  
}

void Drive(double angle) {
  double left = (1 - angle) * moveSpeed;
  double right = (1 + angle) * moveSpeed;

  DriveAccurate(left, right);
}

void DriveAccurate(int left, int right) {
    Serial.print("Left: ");
  Serial.println(left);
  Serial.print("Right: ");
  Serial.println(right);
  Serial.print("My heading is: ");
  Serial.println(left > right ? "right" : "left");

  MotorL.run(left);
  MotorR.run(right);
}

double calculateTurnAngle(int sides[4]) {
  int triangleSide = sides[1] - sides[2];

  int hypotenuse = calculateHypotenuse(SENSOR_DIST, triangleSide);
  int triangle[] = {SENSOR_DIST, triangleSide, hypotenuse};
  double sinA = ((double)triangleSide) / ((double) hypotenuse);
  double radians = asin(sinA);

  return sinA;
}

int calculateHypotenuse(int side1, int side2) {
  return sqrt(pow(side1, 2) + pow(side2, 2));
}

bool isTrapped() {
  return incomingByte[0] < 10 && incomingByte[1] < 10 && readBackwardDistance() < 10;
}

void Stop()
{
  Serial.println("Stop");
  MotorL.run(0);
  MotorR.run(0);
}
void ChangeSpeed(int spd)
{
  moveSpeed = spd;
}

void readBytes(int howMany) {
  /*Serial.print("Reading ");
  Serial.print(howMany);
  Serial.println(" bytes");
  */
  for (int i = 0; i < howMany; i++) {
    byte b = Wire.read();
    incomingByte[i] = b;
    //  Serial.print("Read ");
    //Serial.println(b);
  }
}

byte readBackwardDistance() {
  int backDistance = UltrasonicSensor.distanceCm();
  return backDistance != 0 ? backDistance : MAX_DIST;
}
