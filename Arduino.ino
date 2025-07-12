#include <Servo.h>

Servo servoX;  // Pan
Servo servoY;  // Tilt

const int laserPin = 8;  

int angleX = 90;
int angleY = 80;      

String inputString = "";
bool stringComplete = false;

unsigned long lastCommandTime = 0;          
const unsigned long resetTimeout = 3000;    

void setup() {
  Serial.begin(9600);
  servoX.attach(9);
  servoY.attach(10);
  pinMode(laserPin, OUTPUT);
  digitalWrite(laserPin, LOW);  

  servoX.write(angleX);
  servoY.write(angleY);

  inputString.reserve(30);
  lastCommandTime = millis();
}

void loop() {
  if (stringComplete) {
    inputString.trim();
    int firstComma = inputString.indexOf(',');
    int secondComma = inputString.indexOf(',', firstComma + 1);

    if (firstComma > 0 && secondComma > firstComma) {
      String xStr = inputString.substring(0, firstComma);
      String yStr = inputString.substring(firstComma + 1, secondComma);
      String laserStr = inputString.substring(secondComma + 1);

      int xVal = xStr.toInt();
      int yVal = yStr.toInt();
      int laserState = laserStr.toInt();

      xVal = constrain(xVal, 0, 180);
      yVal = constrain(yVal, 0, 180);

      servoX.write(xVal);
      servoY.write(yVal);

      if (laserState == 1) {
        digitalWrite(laserPin, HIGH);
      } else {
        digitalWrite(laserPin, LOW);
      }

      Serial.print("Pan: ");
      Serial.print(xVal);
      Serial.print(" Tilt: ");
      Serial.print(yVal);
      Serial.print(" Laser: ");
      Serial.println(laserState);

      lastCommandTime = millis();  
    }

    inputString = "";
    stringComplete = false;
  }

  if (millis() - lastCommandTime > resetTimeout) {
    servoY.write(80);
    digitalWrite(laserPin, LOW);
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}
