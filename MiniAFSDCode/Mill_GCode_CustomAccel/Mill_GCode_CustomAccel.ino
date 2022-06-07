//Mill Notes
//Hole Spacing in X: 2.146 in

#include <Wire.h>
#include <CustomAccelStepper.h>

#define xStepPin 21
#define xDirPin 17

#define yStepPin 16
#define yDirPin 19       //y pins not final!
#define xyEnableuint8_t B01100000

#define aStepPin 18
#define aDirPin 5
#define aEnableuint8_t B10000000

#define sensorPin 36
#define expanderID 0x20

#define serDataSize 64

#define skipPin 14
#define eStopPin

#define xyStepsPerMil 80
#define zStepsPerMil 80 //unused for now
#define aStepsPerMil 4080

ESP_AccelStepper xAxis;
ESP_AccelStepper yAxis;
ESP_AccelStepper aAxis;

const uint16_t xyRapidSpeed = 15 * xyStepsPerMil; //15 mm/sec * steps/mm
const uint16_t aRapidSpeed = 1 * aStepsPerMil; //1 mm/sec * steps/mm

const uint16_t maxXY_Accel = 50 * xyStepsPerMil; //50 mm/sec^2 * steps/mm
const uint16_t maxA_Accel = 50 * aStepsPerMil; //250 mm/sec^2 * steps/mm

const uint16_t maxXY_Decel = 100 * xyStepsPerMil;
const uint16_t maxA_Decel = 100 * aStepsPerMil;

const uint16_t curveSection = (uint16_t)(0.2 * xyStepsPerMil); //0.2 mm * steps/mm

uint8_t avgCount = 0;
uint8_t enableuint8_t = 0x00;
uint16_t aLoad = 0;

boolean xyEnabled = false;
boolean aEnabled = false;
boolean reachedPos = true;
boolean incMode = false; //False = Absolute, True = Incremental

uint16_t xySpeed = xyRapidSpeed;

uint16_t aMaxForce = 0;

char serData[serDataSize];
uint8_t lineSize = 0;
uint8_t endBuffer[] = {3, 4};
uint8_t ackBuffer[] = {6, 3, 4};
uint8_t ctgBuffer[] = {1, 8, 3, 4};


struct Pos {
  int32_t X;
  int32_t Y;
  int32_t A;
};

struct Command {
  uint8_t G;
  uint8_t M;
  int32_t X;
  int32_t Y;
  int32_t Z;
  int32_t A;
  int32_t I;
  int32_t J;
  int32_t F;
  int32_t P;
  int32_t L;
};

struct CommandFlag {
  bool G;
  bool M;
  bool X;
  bool Y;
  bool Z;
  bool A;
  bool I;
  bool J;
  bool F;
  bool P;
  bool L;
};

struct Spiral {
  boolean dir; //0 for CW, 1 for CCW
  uint32_t radius; //in um
  int32_t angleA; //in urads
  int32_t angleB; //in urads
  int32_t angle; //in urads
  int32_t xCenter; //in um
  int32_t yCenter; //in um
  uint16_t steps;
  uint16_t stepNum;
  int32_t startA; //in um
};

Pos targetPos;
Command inCode = (Command) {
  0, 0, 0, 0, 0, 0, 0, 0, xyRapidSpeed * 60, aRapidSpeed * 60, 0
};
CommandFlag inCodeFlag = (CommandFlag) {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

Spiral gCurve;


void feedController(void * parameter) {
  while (1) {
    while (!reachedPos) {
      while (!xAxis.motionComplete() || !yAxis.motionComplete() || !aAxis.motionComplete()) {
        if (xyEnabled) {
          xAxis.processMovement();
          yAxis.processMovement();
        }
        if (aEnabled)
          aAxis.processMovement();
      }
      if (!determineNextStep()) {
        reachedPos = true;
        gCurve = (Spiral) {
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        };
        Serial.write(ctgBuffer, 3);
      }
    }
    delay(1);
  }
}

void reportData() {
  uint8_t serBuffer[15];
  if (avgCount == 10) {
    aLoad /= avgCount;
    avgCount = 0;
    serBuffer[0] = 70;
    serBuffer[13] = (uint8_t)(aLoad);
    serBuffer[14] = (uint8_t)(aLoad >> 8);
    aLoad = 0;
  } else {
    serBuffer[0] = 80;
    serBuffer[13] = 0;
    serBuffer[14] = 0;
    aLoad += analogRead(sensorPin);
    avgCount++;
  }
  int32_t xPos, yPos, aPos;
  xPos = xAxis.getCurrentPosition();
  serBuffer[1] = uint8_t(xPos);
  for (uint8_t i = 1; i < 4; i++)
    serBuffer[i + 1] = uint8_t(xPos >> (8 * i));

  yPos = yAxis.getCurrentPosition();
  serBuffer[5] = uint8_t(yPos);
  for (uint8_t i = 1; i < 4; i++)
    serBuffer[i + 5] = uint8_t(yPos >> (8 * i));

  aPos = aAxis.getCurrentPosition();
  serBuffer[9] = uint8_t(aPos);
  for (uint8_t i = 1; i < 4; i++)
    serBuffer[i + 9] = uint8_t(aPos >> (8 * i));

  Serial.write(1);
  Serial.write(serBuffer, 15);
  Serial.write(endBuffer, 2);
}

void setup()
{
  Wire.begin();
  Wire.beginTransmission(expanderID);
  Wire.write(enableuint8_t);
  Wire.endTransmission();

  pinMode(sensorPin, INPUT);
  pinMode(skipPin, INPUT_PULLUP);

  xAxis.connectToPins(xStepPin, xDirPin);
  yAxis.connectToPins(yStepPin, yDirPin);
  aAxis.connectToPins(aStepPin, aDirPin);

  for (uint16_t i = 0; i < serDataSize; i++) {
    serData[i] = 0;
  }

  Serial.begin(115200);

  xTaskCreatePinnedToCore(feedController, "Feed Force Control", 10000, NULL, 1, NULL,  0);
  disableCore0WDT();
}


void loop() {
  if (Serial.available()) {
    processSerialData();
  }
  if (!digitalRead(skipPin) && !reachedPos) { //If skip pin is pressed (active low)
    reachedPos = true;
    xAxis.setCurrentPosition(targetPos.X);
    yAxis.setCurrentPosition(targetPos.Y);
    aAxis.setCurrentPosition(targetPos.A);
    delay(10);
    while(!digitalRead(skipPin)) {
      delay(1);
    }
    Serial.write(ctgBuffer, 3);
  }
}

void processSerialData() {
  char c = Serial.read();
  serData[0] = c;
  if (c > 32) {
    lineSize = 1;
    while (lineSize < serDataSize && Serial.peek() != 4) {
      if (Serial.available())
        serData[lineSize++] = Serial.read();
      else
        delayMicroseconds(10);
    }
    Serial.read();
  }
  switch (c) {
    case 'B': {//Begin
        if (!timerStarted()) {
          Serial.write(endBuffer, 2);
          setupTimer(50000); //Setup timer with 70 ms intervals
          startTimer();
        }
        beginMill(serData);
        Serial.print("Starting Mill Operation");
        Serial.write(endBuffer, 2);
      }
      break;
    case 'G': //G Codes
    case 'M'://M Codes
      readCode(serData);
      Serial.write(ackBuffer, 3);
      break;
    case 'S': {//Stop
        if (!reachedPos) {
          xAxis.emergencyStop();
          yAxis.emergencyStop();
          aAxis.emergencyStop();
          while (!reachedPos) {
            delay(1);
          }
        }
        Serial.print("Stopping all outputs");
        Serial.write(endBuffer, 2);
        enableuint8_t = 0x00;
        Wire.beginTransmission(expanderID);
        Wire.write(enableuint8_t);
        Wire.endTransmission();
        xyEnabled = false;
        aEnabled = false;
      }
      break;
    default: {
        Serial.print("Recieved unknown character: ");
        Serial.println(c);
        Serial.write(endBuffer, 2);
      }
  }
  for (uint16_t i = 0; i < serDataSize; i++) {
    serData[i] = 0;
  }
}


void readCode(char uint8_tData[]) {
  determineFlags(uint8_tData, lineSize);
  determineValues(uint8_tData, lineSize);
  if (inCodeFlag.G) {
    switch (inCode.G) {
      case 0: {                                  //Rapid Movement
          xySpeed = xyRapidSpeed;
        }
      case 1: {                                 //Controlled Movement
          int32_t xToGo, yToGo, aToGo;

          if (incMode) {
            targetPos.X += inCode.X;
            targetPos.Y += inCode.Y;
            targetPos.A += inCode.A;
          } else {
            targetPos.X = inCodeFlag.X ? inCode.X : targetPos.X;
            targetPos.Y = inCodeFlag.Y ? inCode.Y : targetPos.Y;
            targetPos.A = inCodeFlag.A ? inCode.A : targetPos.A;
          }
          xToGo = targetPos.X - xAxis.getCurrentPosition();
          yToGo = targetPos.Y - yAxis.getCurrentPosition();
          aToGo = targetPos.A - aAxis.getCurrentPosition();

          calculateSpeeds(xToGo, yToGo, aToGo);

          xAxis.setTargetPosition(targetPos.X);
          yAxis.setTargetPosition(targetPos.Y);
          aAxis.setTargetPosition(targetPos.A);
          reachedPos = false;
        }
        break;
      case 2:
      case 3: {
          int32_t aX, aY, bX, bY;
          gCurve.xCenter = inCode.I + xAxis.getCurrentPosition();
          gCurve.yCenter = inCode.J + yAxis.getCurrentPosition();

          targetPos.X = incMode ? (inCode.X + xAxis.getTargetPosition()) : inCode.X;
          targetPos.Y = incMode ? (inCode.Y + yAxis.getTargetPosition()) : inCode.Y;
          if (inCodeFlag.A)
            targetPos.A = incMode ? (inCode.A + aAxis.getTargetPosition()) : inCode.A;

          gCurve.startA = aAxis.getTargetPosition();

          aX = -inCode.I;
          aY = -inCode.J;
          bX = targetPos.X - gCurve.xCenter;
          bY = targetPos.Y - gCurve.yCenter;

          if (inCode.G == 2) { // Clockwise
            gCurve.angleA = (int32_t)(atan2(bY, bX) * 1000);
            gCurve.angleB = (int32_t)(atan2(aY, aX) * 1000);
          } else { // Counterclockwise
            gCurve.angleA = (int32_t)(atan2(aY, aX) * 1000);
            gCurve.angleB = (int32_t)(atan2(bY, bX) * 1000);
          }

          if (gCurve.angleB <= gCurve.angleA)
            gCurve.angleB += 6283;
          gCurve.angle = gCurve.angleB - gCurve.angleA;

          gCurve.radius = sqrt(aX * aX + aY * aY);
          gCurve.steps = (int16_t) ceil(gCurve.radius * gCurve.angle / (1000 * curveSection));

          gCurve.dir = inCode.G - 2;
          gCurve.stepNum = 1;
          xySpeed = (uint16_t)(xySpeed * 1.13); //Accounts for curves being slightly slow
          determineNextStep();
          reachedPos = false;
        }
        break;

      case 12:                                  //Parameter Updatings Command
        break;
      case 28: {                                //Auto Home

        }
        break;
      case 90: {                                //Absolute Positioning
          incMode = false;
          Serial.write(ctgBuffer, 3);
        }
        break;
      case 91: {                                //Relative Positioning
          incMode = true;
          Serial.write(ctgBuffer, 3);
        }
        break;
      case 92: {                                //Set Current Position
          if (inCodeFlag.X) {
            targetPos.X = inCode.X;
            xAxis.setCurrentPosition(targetPos.X);
            xAxis.setTargetPosition(targetPos.X);
          }
          if (inCodeFlag.Y) {
            targetPos.Y = inCode.Y;
            yAxis.setCurrentPosition(targetPos.Y);
            yAxis.setTargetPosition(targetPos.Y);
          }
          if (inCodeFlag.A) {
            targetPos.A = inCode.A;
            aAxis.setCurrentPosition(targetPos.A);
            aAxis.setTargetPosition(targetPos.A);
          }
          Serial.write(ctgBuffer, 3);
        }
        break;
      default: {
          Serial.print("Recieved Unknown GCode: ");
          Serial.println(inCode.G);
          Serial.write(endBuffer, 2);
          Serial.write(ctgBuffer, 3);
        }
    }
  } else if (inCodeFlag.M) {
    switch (inCode.M) {
      case 17: {//Enable Motors
          if (inCodeFlag.X && inCodeFlag.Y) {
            enableuint8_t = enableuint8_t | xyEnableuint8_t;
            xyEnabled = true;
          }
          if (inCodeFlag.A) {
            enableuint8_t = enableuint8_t | aEnableuint8_t;
            aEnabled = true;
          }
          if (!((inCodeFlag.X && inCodeFlag.Y) || inCodeFlag.A)) {
            enableuint8_t = 0xE0;
            xyEnabled = true;
            aEnabled = true;
          }
          Wire.beginTransmission(expanderID);
          Wire.write(enableuint8_t);
          Wire.endTransmission();
          Serial.write(ackBuffer, 3);;
        }
        break;
      case 18: {//Disable Motors
          if (inCodeFlag.X && inCodeFlag.Y) {
            enableuint8_t = enableuint8_t & (~xyEnableuint8_t);
            xyEnabled = false;
          }
          if (inCodeFlag.A) {
            enableuint8_t = enableuint8_t & (~aEnableuint8_t);
            aEnabled = false;
          }
          if (!((inCodeFlag.X && inCodeFlag.Y) || inCodeFlag.A)) {
            enableuint8_t = 0x00;
            xyEnabled = false;
            aEnabled = false;
          }
          Wire.beginTransmission(expanderID);
          Wire.write(enableuint8_t);
          Wire.endTransmission();
          Serial.write(ackBuffer, 3);;
        }
        break;
      default: {
          Serial.print("Recieved Unknown MCode: ");
          Serial.print(inCode.M);
          Serial.write(endBuffer, 3);
        }
    }
  }


}

void beginMill(char uint8_tData[]) {
  xAxis.setAcceleration(maxXY_Accel);
  yAxis.setAcceleration(maxXY_Accel);
  aAxis.setAcceleration(maxA_Accel);

  xAxis.setDeceleration(maxXY_Decel);
  yAxis.setDeceleration(maxXY_Decel);
  aAxis.setDeceleration(maxA_Decel);

  xAxis.setSpeed(xyRapidSpeed);
  yAxis.setSpeed(xyRapidSpeed);
  aAxis.setSpeed(aRapidSpeed);

  xAxis.setTargetPosition(xAxis.getCurrentPosition());
  yAxis.setTargetPosition(yAxis.getCurrentPosition());
  aAxis.setTargetPosition(aAxis.getCurrentPosition());
}

//Calculates next step (if applicable) and returns whether another step is needed

boolean determineNextStep() {
  uint32_t timeElapsed = micros();
  if (gCurve.steps != 0 && gCurve.stepNum <= gCurve.steps) {
    int16_t s = gCurve.dir ? gCurve.stepNum : gCurve.steps - gCurve.stepNum;

    int32_t xPos, yPos, aPos;
    if (gCurve.stepNum != gCurve.steps) {
      xPos = gCurve.xCenter + gCurve.radius * cos((gCurve.angleA + gCurve.angle * ((float) s / gCurve.steps)) / 1000.0);
      yPos = gCurve.yCenter + gCurve.radius * sin((gCurve.angleA + gCurve.angle * ((float) s / gCurve.steps)) / 1000.0);
      aPos = gCurve.startA + ((float)gCurve.stepNum / gCurve.steps) * (targetPos.A - gCurve.startA);
    } else {
      xPos = targetPos.X;
      yPos = targetPos.Y;
      aPos = targetPos.A;
      xySpeed = inCode.F;
    }
    calculateSpeeds(xPos - xAxis.getCurrentPosition(), yPos - yAxis.getCurrentPosition(), aPos - aAxis.getCurrentPosition());
    xAxis.setTargetPosition(xPos);
    yAxis.setTargetPosition(yPos);
    aAxis.setTargetPosition(aPos);
    gCurve.stepNum++;
    timeElapsed = micros() - timeElapsed;
    Serial.print("Time Elapsed: " + String(timeElapsed) + " us.");
    Serial.write(endBuffer, 2);
    return true;
  }
  else {
    return false;
  }
}


void calculateSpeeds(int32_t xToGo, int32_t yToGo, int32_t aToGo) {
  xToGo = abs(xToGo);
  yToGo = abs(yToGo);
  aToGo = abs(aToGo);

  float dist = sqrt(xToGo * xToGo + yToGo * yToGo + (aToGo*aToGo)/((aStepsPerMil/xyStepsPerMil)*(aStepsPerMil/xyStepsPerMil)));

  if (dist != 0) {
    float cx = xToGo / dist;
    float cy = yToGo / dist;
    float ca = (aToGo / dist);
    xAxis.setSpeed(xySpeed * cx);
    yAxis.setSpeed(xySpeed * cy);
    aAxis.setSpeed((xySpeed * ca > aRapidSpeed) ? aRapidSpeed : (xySpeed * ca));

    xAxis.setAcceleration(maxXY_Accel * cx);
    yAxis.setAcceleration(maxXY_Accel * cy);
    aAxis.setAcceleration(maxA_Accel * ca);

    xAxis.setDeceleration(maxXY_Decel * cx);
    yAxis.setDeceleration(maxXY_Decel * cy);
    aAxis.setDeceleration(maxA_Decel * ca);
  }
}

//look for the number that appears after the char key and return it
float searchString(char key, char instruction[], uint8_t stringSize)
{
  char temp[10] = "";

  for (uint8_t i = 0; i < stringSize; i++) {
    if (instruction[i] == key) {
      i++;
      uint8_t k = 0;
      while (i < stringSize && k < 10) {
        if (instruction[i] == 0 || instruction[i] == ' ')
          break;
        temp[k] = instruction[i];
        i++;
        k++;
      }
      return atof(temp);
    }
  }

  return 0;
}

void determineFlags(char instruction[], uint8_t stringSize) {
  inCodeFlag = (CommandFlag) {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
  };
  for (uint8_t i = 0; i < stringSize; i++) {
    if (instruction[i] > 64 && instruction[i] < 91) {
      switch (instruction[i]) {
        case 'G':
          inCodeFlag.G = true;
          break;
        case 'M':
          inCodeFlag.M = true;
          break;
        case 'X':
          inCodeFlag.X = true;
          break;
        case 'Y':
          inCodeFlag.Y = true;
          break;
        case 'Z':
          inCodeFlag.Z = true;
          break;
        case 'A':
          inCodeFlag.A = true;
          break;
        case 'I':
          inCodeFlag.I = true;
          break;
        case 'J':
          inCodeFlag.J = true;
          break;
        case 'F':
          inCodeFlag.F = true;
          break;
        case 'P':
          inCodeFlag.P = true;
          break;
        case 'L':
          inCodeFlag.L = true;
          break;
      }
    }
  }
}

void determineValues(char instruction[], uint8_t stringSize) {
  inCode.G = 0; inCode.M = 0;
  inCode.X = 0; inCode.Y = 0; inCode.Z = 0; inCode.A = 0;
  inCode.I = 0; inCode.J = 0;
  if (inCodeFlag.G)
    inCode.G = (uint8_t)searchString('G', instruction, stringSize);
  if (inCodeFlag.M)
    inCode.M = (uint8_t)searchString('M', instruction, stringSize);
  if (inCodeFlag.X)
    inCode.X = (int32_t)(searchString('X', instruction, stringSize) * xyStepsPerMil);
  if (inCodeFlag.Y)
    inCode.Y = (int32_t)(searchString('Y', instruction, stringSize) * xyStepsPerMil);
  if (inCodeFlag.Z) //unused for now
    inCode.Z = (int32_t)(searchString('Z', instruction, stringSize) * zStepsPerMil);
  if (inCodeFlag.A)
    inCode.A = (int32_t)(searchString('A', instruction, stringSize) * aStepsPerMil);
  if (inCodeFlag.I)
    inCode.I = (int32_t)(searchString('I', instruction, stringSize) * xyStepsPerMil);
  if (inCodeFlag.J)
    inCode.J = (int32_t)(searchString('J', instruction, stringSize) * xyStepsPerMil);
  if (inCodeFlag.F) {
    inCode.F = (int32_t)(searchString('F', instruction, stringSize) * (1.0 / 60.0) * xyStepsPerMil); //steps/sec
    xySpeed = inCode.F;
  }
  if (inCodeFlag.P) {
    inCode.P = (int32_t)(searchString('P', instruction, stringSize) * (1.0 / 60.0) * aStepsPerMil);
    aAxis.setSpeed(inCode.P);
  }
  if (inCodeFlag.L)
    inCode.L = (int32_t)(searchString('L', instruction, stringSize) * 1000); //in mN
}
