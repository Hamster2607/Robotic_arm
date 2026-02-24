#include <Servo.h>
#include <math.h>

Servo base, shoulder, elbow;

const float L1 = 90.0;
const float L2 = 101.0;

float baseAngle = 90;
float shoulderAngle = 90;
float elbowAngle = 90;

const float SHOULDER_MIN = 10;
const float SHOULDER_MAX = 150;
const float ELBOW_MIN = 20;
const float ELBOW_MAX = 160;

void setup() {
  base.attach(10);
  shoulder.attach(9);
  elbow.attach(11);
  Serial.begin(9600);
  
  moveServos(90, 90, 90);
  Serial.println("Arm ready.");
}

void forwardKinematics(float shoulder_deg, float elbow_deg, float &x, float &y) {
  float theta1 = radians(shoulder_deg);
  float theta2 = radians(elbow_deg - shoulder_deg);
  
  x = L1 * cos(theta1 - PI/2) + L2 * cos(theta1 + theta2 - PI/2);
  y = L1 * sin(theta1 - PI/2) + L2 * sin(theta1 + theta2 - PI/2);
}

bool inverseKinematics(float x, float y, float &shoulder_deg, float &elbow_deg) {
  float dist = sqrt(x*x + y*y);
  
  if (dist > (L1 + L2) || dist < abs(L1 - L2)) {
    Serial.println("Out of reach!");
    return false;
  }
  
  float cos_elbow = (L1*L1 + L2*L2 - dist*dist) / (2 * L1 * L2);
  cos_elbow = constrain(cos_elbow, -1.0, 1.0);
  float elbow_rad = acos(cos_elbow);
  
  float alpha = atan2(y, x);
  float beta = acos((L1*L1 + dist*dist - L2*L2) / (2 * L1 * dist));
  float shoulder_rad = alpha - beta + PI/2;
  
  shoulder_deg = degrees(shoulder_rad);
  elbow_deg = degrees(elbow_rad) + shoulder_deg;
  
  if (shoulder_deg < SHOULDER_MIN || shoulder_deg > SHOULDER_MAX ||
      elbow_deg < ELBOW_MIN || elbow_deg > ELBOW_MAX) {
    Serial.println("Angles out of range!");
    return false;
  }
  
  return true;
}

void moveServos(float s, float e, float b) {
  shoulder.write(s);
  elbow.write(e);
  base.write(b);
  
  shoulderAngle = s;
  elbowAngle = e;
  baseAngle = b;
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd.startsWith("MOVE")) {
      int space1 = cmd.indexOf(' ', 5);
      float targetX = cmd.substring(5, space1).toFloat();
      float targetY = cmd.substring(space1 + 1).toFloat();
      
      Serial.print("Target: X=");
      Serial.print(targetX);
      Serial.print(" Y=");
      Serial.println(targetY);
      
      // BASE CENTER IS AT ORIGIN (0, 0)
      // Calculate angle to point at target
      float angleRad = atan2(targetX, targetY);  // atan2(X, Y) for base rotation
      float b = degrees(angleRad) + 90;  // Convert to servo degrees (90 = forward)
      b = constrain(b, 0, 180);
      
      Serial.print("Base angle: ");
      Serial.println(b);
      
      base.write(b);
      baseAngle = b;
      delay(800);
      
      // Calculate distance from base to target (in horizontal plane)
      float horizontalDist = sqrt(targetX*targetX + targetY*targetY);
      
      // Object is on table, 66mm below shoulder
      const float BASE_HEIGHT = 66.0;
      
      Serial.print("Horizontal distance: ");
      Serial.print(horizontalDist);
      Serial.print(" Down: ");
      Serial.println(BASE_HEIGHT);
      
      // IK in the vertical plane (side view after base rotation)
      // X = horizontal distance from base
      // Y = -66 (down to table level)
      
      float s, e;
      if (inverseKinematics(horizontalDist, -BASE_HEIGHT, s, e)) {
        shoulder.write(s);
        elbow.write(e);
        shoulderAngle = s;
        elbowAngle = e;
        
        Serial.print("SUCCESS - Shoulder:");
        Serial.print(s);
        Serial.print(" Elbow:");
        Serial.println(e);
      }
    }
    
    else if (cmd.startsWith("ANGLES")) {
      int space1 = cmd.indexOf(' ', 7);
      float s = cmd.substring(7, space1).toFloat();
      float e = cmd.substring(space1 + 1).toFloat();
      
      moveServos(s, e, baseAngle);
      Serial.println("Angles set");
    }
    
    else if (cmd == "HOME") {
      moveServos(90, 90, 90);
      Serial.println("Home");
    }
  }
}
