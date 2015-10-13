#include <Makeblock.h>
#include <SoftwareSerial.h>
#include <Wire.h>

MeDCMotor MotorL(M1);
MeDCMotor MotorR(M2);
MeInfraredReceiver infraredReceiverDecode(PORT_6);
MeUltrasonicSensor UltrasonicSensor(PORT_3);

#define MAX_DIST 150
int moveSpeed = 75;
int turnSpeed = 200;
int minSpeed = 45;
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

  if (millis() % 50 == 0)
  {
    ultrCarProcess();
  }
}



void Forward()
{
  Serial.println("forward");
  MotorL.run(moveSpeed);
  MotorR.run(moveSpeed);
}
void Backward()
{
  Serial.println("back");
  MotorL.run(-moveSpeed);
  MotorR.run(-moveSpeed);
}

void BackwardAndTurnLeft()
{
  Serial.println("back left");
  MotorL.run(-moveSpeed / 4);
  MotorR.run(-moveSpeed);
}
void BackwardAndTurnRight()
{
  Serial.println("back right");
  MotorL.run(-moveSpeed);
  MotorR.run(-moveSpeed / 4);
}
void TurnLeft()
{
  Serial.println("left");
  MotorL.run(-moveSpeed);
  MotorR.run(moveSpeed);
}
void TurnLeftForward() {
  Serial.println("forward left");
  MotorL.run(moveSpeed / 4);
  MotorR.run(moveSpeed);
}
void TurnRightForward()
{
  Serial.println("forward right");
  MotorL.run(moveSpeed);
  MotorR.run(moveSpeed / 4);
}
void TurnRight()
{
  Serial.println("right");
  MotorL.run(moveSpeed);
  MotorR.run(-moveSpeed);
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

void ultrCarProcess()
{
  int backDistance = readBackwardDistance();
  Serial.print("Left: ");
  Serial.println(incomingByte[0]);
  Serial.print("Right: ");
  Serial.println(incomingByte[1]);
  Serial.print("Back: ");
  Serial.println(backDistance);
  int distance = min(incomingByte[0], incomingByte[1]);
  Serial.print("Minimum forward: ");
  Serial.println(distance);
  if (distance > 50) {
    Forward();
  } else if (distance > 30) {
    if (incomingByte[0] > incomingByte[1])
    {
      TurnLeftForward();
    }
    else
    {
      TurnRightForward();
    }
  } else if (distance <= 30) {
    if (backDistance < 25) {
      if (backDistance < 10) {
        Stop();
      } else if (incomingByte[0] > incomingByte[1]) {
        TurnLeft();
      } else {
        TurnRight();
      }
    } else if (incomingByte[0] > incomingByte[1]) {
      BackwardAndTurnRight();
    } else if (incomingByte[1] > incomingByte[1]) {
      BackwardAndTurnLeft();
    } else {
      Backward();
    }
  }
}
