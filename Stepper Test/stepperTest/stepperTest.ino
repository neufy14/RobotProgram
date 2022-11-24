#include <SoftwareSerial.h>
#include <Stepper.h>

int step_pin = 12;
int direct_pin = 10;
int fast = 50;
int input = 0;
String command = "";

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(step_pin, OUTPUT);
  pinMode(direct_pin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(direct_pin, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  /*
  digitalWrite(step_pin, HIGH);
  digitalWrite(LED_BUILTIN, HIGH);
  delayMicroseconds(fast);
  digitalWrite(step_pin, LOW);
  digitalWrite(LED_BUILTIN, LOW);
  delayMicroseconds(fast);
  */

  if (Serial.available() > 0) {
    command = Serial.readString();
    Serial.print("command = ");
    Serial.println(command);
    //moveStepper(direct_pin, step_pin);
    oneRev(direct_pin, step_pin);
  }
}

float pulseStep(int direct) {
  if (Serial.available() > 0) {
    command = Serial.readString();
    Serial.print("command = ");
    Serial.println(command);
  }
}

float oneRev(int directPin, int stepPin) {
  int currentSpeed = 100;
  float stepsPerRev = 5000;
  float gearRatio = 46;
  float degreesToMove = 30;
  float stepsToMove = (3200.0 / 360.0) * (degreesToMove * gearRatio);
  //float totalSteps = (stepsPerRev*gearRatio);
  int count = 0;
  int takeAverageCount = 0;
  int sampleAnalogIn = 0;
  float onePotTurn = 102.3;
  int motorPotGear = 60.0;
  int potGear = 10.0;
  float potGearRatio = motorPotGear / potGear; 
  float potStart = 0;
  float potStop = 0;
  float potChange = 0;
  float degChange = 0;
  bool firstPotVal = true;
  bool grabPotVal = false;
  Serial.print("stepsToMove = ");
  Serial.println(stepsToMove);
  Stepper currentStepper((stepsPerRev*gearRatio), directPin, directPin+1, stepPin, stepPin+1);
  currentStepper.setSpeed(currentSpeed);
  //for (float i=0; i<stepsToMove; i++) {
  while (degChange < degreesToMove) {
    currentStepper.step(-1);
    count++;
    if (firstPotVal == true) {
      sampleAnalogIn = analogRead(A1) + sampleAnalogIn;
      takeAverageCount++;
      potStart = (sampleAnalogIn) / takeAverageCount;
      potStop = potStart;
      if (takeAverageCount == 10) {
        firstPotVal = false;
        takeAverageCount = 0;
        sampleAnalogIn = 0;
        Serial.print("potStart = ");
        Serial.println(potStart);
      }
      Serial.print("sampleAnalogIn = ");
      Serial.println(sampleAnalogIn);
    }
    if (count > 100) {
      grabPotVal = true;
    }
    if (grabPotVal == true) {
      if (takeAverageCount == 10) {
        takeAverageCount = 0;
        grabPotVal = false;
        potStop = sampleAnalogIn / takeAverageCount;
        count = 0;
        sampleAnalogIn = 0;
      }
      takeAverageCount++;
      Serial.print("takeAverageCount = ");
      Serial.println(takeAverageCount);
      sampleAnalogIn = (analogRead(A1) + sampleAnalogIn);      
      Serial.print("pot value = ");
      Serial.println(analogRead(A1));
    }
    Serial.print("count = ");
    Serial.println(count);
    potChange = abs(potStart - potStop);
    degChange = (potChange * 360) / (onePotTurn * potGearRatio);
  }
//  potChange = abs(potStart - potStop);
//  degChange = (potChange * 360) / (onePotTurn * potGearRatio);
  Serial.print("Degrees moved = ");
  Serial.println(degChange);
  Serial.println("done");
}

float moveStepper(int directPin, int stepPin) {
  int currentSpeed = 6000;
  int stepsPerRev = 3200;
  bool cont = true;
  while (cont == true) {
    Stepper currentStepper(stepsPerRev, directPin, directPin+1, stepPin, stepPin+1);
    currentStepper.setSpeed(currentSpeed);
    Serial.println("Moving");
    currentStepper.step(-100);
    if (Serial.available() > 0) {
      command = Serial.readString();
      Serial.print("command = ");
      Serial.println(command);
      Serial.println(command.indexOf("stop"));
      if (command.indexOf("stop") != -1) {
        Serial.println("stopping");
        cont = false;
      }
    }
  }
  
}
