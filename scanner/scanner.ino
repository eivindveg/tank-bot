#include "NewPing.h"
#include "Wire.h"

#define MAX_DIST 150
NewPing left(12, 11, MAX_DIST);
NewPing right(6, 5, MAX_DIST);

void setup() {
  Serial.begin(9600);
  Wire.begin();
}

void loop() {
  delayMicroseconds(25);
  int leftDist = left.convert_cm(left.ping_median(8));
  delayMicroseconds(25);
  int rightDist = left.convert_cm(right.ping_median(8));
  
  int rightReading = rightDist == 0 ? MAX_DIST : rightDist;
  int leftReading = leftDist == 0 ? MAX_DIST : leftDist;
  Serial.print("Left: ");
  Serial.println(leftDist);
  Serial.print("Right: ");
  Serial.println(rightDist);
  Wire.beginTransmission(8);
  Wire.write(leftReading);
  Wire.write(rightReading);
  Wire.endTransmission();
}
