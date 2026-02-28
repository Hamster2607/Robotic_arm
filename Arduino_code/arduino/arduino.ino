#include <Servo.h>
#include <math.h>

Servo base, shoulder, elbow;

float baseAngle = 90;
float shoulderAngle = 90;
float elbowAngle = 90;

float targetX;
float targetY;

const float L1 = 80;
const float L2 = 101.0;
const int baseX = 0;
const int baseY = 0;
const float offset = 20;

float x = 0, z = 0, y = 0;

void setup() {
  base.attach(10);
  shoulder.attach(9);
  elbow.attach(11);
  Serial.begin(9600);

  moveServos(90, 90, 90);
  Serial.println("Arm ready.");
}

void moveServos(float s, float e, float b) {
  shoulder.write(s);
  elbow.write(e);
  base.write(b);
  
  shoulderAngle = s;
  elbowAngle = e;
  baseAngle = b;
}

float base_Angle(float targetX, float targetY){
     float angleRad = atan2(targetY, targetX);
     baseAngle = 180 - angleRad * (180.0 / PI);
     return baseAngle;
     
}
void forwardKinematics(float &x, float &z, float &y) {
  float theta1 = radians(shoulderAngle);
  float theta2 = radians(elbowAngle - shoulderAngle);
  float theta3 = radians(baseAngle);

  float r = L1 * cos(theta1) + L2 * cos(theta1 - (PI - theta2));
  
 // x = L1 * cos(theta1 - PI/2) + L2 * cos(theta1 + theta2 - PI/2);
 
  z = L1 * sin(theta1 - PI/2) + L2 * sin(theta1 + theta2 - PI/2);
  x = r * sin(theta3);
  y = r * cos(theta3);
}

/*float distance2Points(float targetX, float targetY){
  float distance = hypot(baseX - targetX, baseY - targetY);
  //Serial.print(distance);
  return distance;
}

float distance2Points(float targetX, float targetY){
  float distance = hypot(0 - targetX, 66 - targetY);
  //Serial.print(distance);
  return distance;
}*/

void inverseKinematics(float targetX, float targetY){
  float horizDist = hypot(baseX - targetX, baseY - targetY);
  float reachToTarget = horizDist - 20;
  float distance = hypot(0 - reachToTarget, 30 - 0);
  float tilt = atan2(30, horizDist) * 180.0 / PI;
   Serial.print("Horizontal: ");
  Serial.print(horizDist);
  Serial.print(" | 3D distance: ");
  Serial.println(distance);
  if(distance > L1 + L2){
      Serial.println("Out of reach");
  }else{
    
      float a = (pow(L1, 2) + pow(L2, 2) - pow(distance, 2)) / (2*L1*L2);
      float b = (pow(L1, 2) + pow(distance, 2) - pow(L2, 2)) / (2*L1*distance);
      
      float angleRadA = acos(a);
      elbowAngle = 90 + angleRadA * 180.0 / PI;
      float angleRadB = acos(b);
      shoulderAngle = angleRadB * 180.0 / PI + 90 + tilt - 15;
      Serial.println(baseAngle);
      baseAngle = base_Angle(targetX, targetY);
      if(baseAngle < 0) baseAngle = 0;
     if(baseAngle > 180){ 
      baseAngle = baseAngle - 180;
      elbowAngle = 180 - elbowAngle;
      shoulderAngle = 180 - shoulderAngle;
     }
Serial.print("SENT TO SERVOS - Base:");
Serial.print(baseAngle);
Serial.print(" Shoulder:");
Serial.print(shoulderAngle);  
Serial.print(" Elbow:");
Serial.println(elbowAngle);

      if(elbowAngle < 0) elbowAngle = 0;
     if(elbowAngle> 180) elbowAngle = 180;
      base.write(baseAngle);
      delay(500);
      
      shoulder.write(shoulderAngle);
      elbow.write(elbowAngle);
      
  }
}

void loop() {
  if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      if(cmd.startsWith("POS")){
        forwardKinematics(x, z, y);
        Serial.print("Arm tip is at X:");
        Serial.print(x);
        Serial.print(" Y:");
        Serial.print(y);
        Serial.print(" Z: ");
        Serial.print(z);
      }
      if (cmd.startsWith("reset")) {
        moveServos(90, 90, 90);
      }
        if (cmd.startsWith("T")) {
      int space1 = cmd.indexOf(' ', 2);
      targetX = cmd.substring(2, space1).toFloat();
      targetY = cmd.substring(space1 + 1).toFloat();
      //moveServos(90, 90, 90);
      //Serial.print("Target received: X=");
      //Serial.print(targetX);
      //Serial.print(" Y=");
      //Serial.println(targetY);
      //Serial.println('\n');
      //distance2Points(targetX, targetY);
      //base_Angle(targetX, targetY);
      inverseKinematics(targetX, targetY);
      
      }
    }

}
