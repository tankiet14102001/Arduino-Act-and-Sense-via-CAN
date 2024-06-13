/*
 *Version: 1.0.6
 *Author: Kiet
 *Time: 27 March 2024, 09:08
 *
 *Full functionality for LIGHTS, FANS, SEAT, and PEDAL for dreamPACKxArduinoR4.
 */

#include <Arduino_CAN.h>
#include <Servo.h>

#define SERIAL_BAUDRATE             (115200)
#define DELAY_PERIOD                (5000)
#define MAX_MESSAGE_SIZE            (8)

#define   HEADLIGHT                 (13)
  
  #define   LIGHT_STATE_ON          (1)
  #define   LIGHT_STATE_OFF         (0)
  //#define   LIGHT_STATE             0

  #define   LIGHT_HEADLIGHT_MODE0   (0)     // HEADLIGHT OFF
  #define   LIGHT_HEADLIGHT_MODE1   (1)     // HEADLIGHT ON
  #define   LIGHT_HEADLIGHT_MODE2   (2)     // HEADLIGHT BLINKING

#define   HAZARDLIGHT               (12)                 
  #define   LIGHT_HAZARDLIGHT_MODE0 (0)     // HAZARDLIGHT OFF
  #define   LIGHT_HAZARDLIGHT_MODE1 (4)     // HAZARDLIGHT ON

#define   BRAKELIGHT                (8)
  #define LIGHT_BRAKELIGHT_MODE0    (0)     // BRAKELIGHT OFF
  #define LIGHT_BRAKELIGHT_MODE1    (32)    // BRAKELIGHT ON

#define   DAS_BODYCONTROLS          (1001)  // CAN_ID = 0x3E9

#define   SERVO_PIN                 (6)     // SERVO signal PIN

#define   FAN_LEFT                  (3)     // FAN signal PIN
#define   FAN_RIGHT                 (9)

#define   PEDAL                     (2)     // PEDAL PIN

#define   POSITION0                 (0x00)     // Seat Position 0
#define   POSITION1                 (0x04)     // Seat Position 0
#define   POSITION2                 (0x08)     // Seat Position 0
#define   POSITION3                 (0x0C)     // Seat Position 0
#define   POSITION4                 (0x10)     // Seat Position 0
#define   POSITION5                 (0x14)     // Seat Position 0
#define   POSITION6                 (0x18)     // Seat Position 0
#define   POSITION7                 (0x1C)     // Seat Position 0
#define   POSITION8                 (0x20)     // Seat Position 0
#define   POSITION9                 (0x24)     // Seat Position 0
#define   POSITION10                (0x28)     // Seat Position 0


bool headLightState;
bool hazardLightState;

Servo mySeat;

int buttonState = 0;
int lastButtonState = 0;
//unsigned long long counter = 0;
int counter = 0;

int currentButtonState = 0;
unsigned long lastDebounceTime = 0;
unsigned int debounceDelay = 5;

void setup()
{
  Serial.begin(SERIAL_BAUDRATE);
  //while (!Serial) { }

  if (!CAN.begin(CanBitRate::BR_500k))
  {
    Serial.println("CAN.begin(...) failed.");
    for (;;) {}
  }

  pinMode(HEADLIGHT, OUTPUT);
  pinMode(HAZARDLIGHT, OUTPUT);
  digitalWrite(HAZARDLIGHT, LOW);
  pinMode(BRAKELIGHT, OUTPUT);

  //pinMode(PEDAL, INPUT_PULLDOWN);
  pinMode(PEDAL, INPUT);
  //pinMode(PEDAL, INPUT_PULLUP);
  mySeat.attach(SERVO_PIN);
  mySeat.writeMicroseconds(540);

  pinMode(FAN_LEFT, OUTPUT);
  pinMode(FAN_RIGHT, OUTPUT);
  digitalWrite(FAN_LEFT, LOW);
  digitalWrite(FAN_RIGHT, LOW);
  //myFan.attach(VIRTUAL_FAN_PIN);
}


void loop() {
  uint8_t const pedal_not_pressed[] = {0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0xFF, 0xFF, 0x30};
  uint8_t const pedal_pressed[]     = {0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0x30};
  CanMsg const pedalNotPressedMsg(CanStandardId(0x082), sizeof(pedal_not_pressed), pedal_not_pressed);
  CanMsg const pedalPressedMsg(CanStandardId(0x082), sizeof(pedal_pressed), pedal_pressed);

  currentButtonState = digitalRead(PEDAL);
    if (currentButtonState != 0) {
      lastDebounceTime = millis();
      //counter++;
    }
    while ( (millis() - lastDebounceTime) <= debounceDelay) {
      if (currentButtonState != 0) {
        counter = 1;
        Serial.print("Counter: ");
        Serial.println(counter);
      }
    }
    if ( (millis() - lastDebounceTime) > debounceDelay ) {
      if (counter != 0) {
        Serial.println("PEDAL is not pressed");
        //digitalWrite(13, LOW);
        CAN.write(pedalNotPressedMsg);
      }
      else {
        Serial.println("PEDAL is pressed");
        //digitalWrite(13, HIGH);
        CAN.write(pedalPressedMsg);
      }
      counter = 0;
    }
    lastButtonState = currentButtonState;

  if (CAN.available())
  {
    /* Receiving CAN message */
    CanMsg const msg = CAN.read();
    Serial.println(msg);

    // copy read msg to a variable
    CanMsg const inputMsg(msg);

    uint8_t triggerHeadLight[MAX_MESSAGE_SIZE]   = {2, 0, 0, 0, 0, 0, 0, 0};
    uint8_t triggerHazardLight[MAX_MESSAGE_SIZE] = {4, 0, 0, 0, 0, 0, 0, 0};
    
    // CAN ID for lightControl
    if ( (inputMsg.id == 0x3E9 ) ) {  
      printCanMsg(inputMsg);
      Serial.println(inputMsg.data[0] & 0b1100); 
      switch(inputMsg.data[0] & 0b11) {
        case (LIGHT_HEADLIGHT_MODE0):
          Serial.println("LIGHT_HEADLIGHT_MODE0");
          turnLedOff();
          break;
        case (LIGHT_HEADLIGHT_MODE1):
          Serial.println("LIGHT_HEADLIGHT_MODE1");
          turnLedOn();
          break;
        case (LIGHT_HEADLIGHT_MODE2):
          Serial.println("LIGHT_HEADLIGHT_MODE2");
          toggleLed();
          break;

        default:
          break;        
      }
      switch(inputMsg.data[0] & 0b1100) {
        case(LIGHT_HAZARDLIGHT_MODE0):
          Serial.println("LIGHT_HAZARDLIGHT_MODE0");
          turnHazardOff();
          break;

        case(LIGHT_HAZARDLIGHT_MODE1):
          Serial.println("LIGHT_HAZARDLIGHT_MODE1");
          toggleHazard();
          break;

        default:
          break;
      }
      switch( (inputMsg.data[2] >> 1) & 0b00100000) {
        case(LIGHT_BRAKELIGHT_MODE0):
          Serial.println("LIGHT_BRAKELIGHT_MODE0");
          turnBrakeOff();
          break;
        case(LIGHT_BRAKELIGHT_MODE1):
          Serial.println("LIGHT_BRAKELIGHT_MODE1");
          turnBrakeOn();
          break;

        default:
          break;
      }
    }
    
    // functions for SeatPositionControl
    if ( inputMsg.id == 0x3C3 ) {
      switch (inputMsg.data[0]) {
        case (POSITION0):
          servoControl(mySeat, servoAngle(0));
          break;
      }
      switch (inputMsg.data[0]) {
        case (POSITION1):
          servoControl(mySeat, servoAngle(1));
          break;
      }
      switch (inputMsg.data[0]) {
        case (POSITION2):
          servoControl(mySeat, servoAngle(2));
          break;
      }
      switch (inputMsg.data[0]) {
        case (POSITION3):
          servoControl(mySeat, servoAngle(3));
          break;
      }
      switch (inputMsg.data[0]) {
        case (POSITION4):
          servoControl(mySeat, servoAngle(4));
          break;
      }
      switch (inputMsg.data[0]) {
        case (POSITION5):
          servoControl(mySeat, servoAngle(5));
          break;
      }
      switch (inputMsg.data[0]) {
        case (POSITION6):
          servoControl(mySeat, servoAngle(6));
          break;
      }
      switch (inputMsg.data[0]) {
        case (POSITION7):
          servoControl(mySeat, servoAngle(7));
          break;
      }
      switch (inputMsg.data[0]) {
        case (POSITION8):
          servoControl(mySeat, servoAngle(8));
          break;
      }
      switch (inputMsg.data[0]) {
        case (POSITION9):
          servoControl(mySeat, servoAngle(9));
          break;
      }
      switch (inputMsg.data[0]) {
        case (POSITION10):
          servoControl(mySeat, servoAngle(10));
          break;
      default:
        break;
      }
    }

    // function for FAN
    if (inputMsg.id == 0x282) {     
      //fanControl(myFan, PWMspeed( int(inputMsg.data[1]) ));
      Serial.println("Controling Left Fan!");
      Serial.print("Speed is: ");
      PWMleftFanControl(FAN_LEFT, int(inputMsg.data[1]));      
      //Serial.println(PWMspeed(inputMsg.data[1]));
    }

    if (inputMsg.id == 0x20C) {
      Serial.println("Controling Right Fan!");
      Serial.print("Speed is: ");
      PWMrightFanControl(FAN_RIGHT, int(inputMsg.data[4]));      
      //Serial.println(PWMspeed(inputMsg.data[4]));
    }
  }
}

/* LED functions */
void turnLedOn() {
  digitalWrite(HEADLIGHT, HIGH);
  headLightState = LIGHT_STATE_ON;
}

void turnLedOff() {
  digitalWrite(HEADLIGHT, LOW);
  headLightState = LIGHT_STATE_OFF;
}

void toggleLed() {
  if ( (headLightState) == (LIGHT_STATE_ON) ) {
    turnLedOff();
  }
  else {
    turnLedOn();
  } 
}

void turnHazardOn() {
  digitalWrite(HAZARDLIGHT, HIGH);
  hazardLightState = LIGHT_STATE_ON;
}

void turnHazardOff() {
  digitalWrite(HAZARDLIGHT, LOW);
  hazardLightState = LIGHT_STATE_OFF;
}

void toggleHazard() {
  //hazardLightState = LIGHT_STATE_OFF;
  if ( (hazardLightState) == (LIGHT_STATE_OFF) ) {
    turnHazardOn();
  }
  else {
    turnHazardOff();
  }
}

void turnBrakeOn() {
  digitalWrite(BRAKELIGHT, HIGH);
}

void turnBrakeOff() {
  digitalWrite(BRAKELIGHT, LOW);
}

void printCanMsg(CanMsg const feed) {
  for(int i = 0; i < MAX_MESSAGE_SIZE; i++) {
    Serial.print(feed.data[i]);
    Serial.print("\t");
  }
  Serial.println();
}

int servoAngle(int positionNo) {
  if (positionNo < 0) positionNo = 0;
  if (positionNo > 10) positionNo = 10;
  int outputAngle = 540 + 185 * positionNo;  // 550 is the initial voltage for position 0
                                                 //185 is the voltage step between the positions
  return outputAngle;
}

void servoControl(Servo myServo, int outputAngle) {
  myServo.writeMicroseconds(outputAngle);
}

int PWMspeed(uint8_t dataInput) {
  if (dataInput < 0x00) {
    dataInput = 0x00;
  }
  if (dataInput > 0x28) {
    dataInput = 0x28;
  }
  if ( (dataInput >= 0x00) && (dataInput <= 0x28) ) {
    int dataOutput = map(dataInput, 0x00, 0x28, 1500, 2400);
    return dataOutput;
  }
}

void fanControl(Servo myFan, int calculatedSpeed) {
  myFan.writeMicroseconds(calculatedSpeed);
}

void PWMleftFanControl(int PWMpin, int FanSpeed) {
  int dataOutput = map(FanSpeed, 0x00, 0x28, 0, 256);
  analogWrite(PWMpin, dataOutput);
  Serial.print("Fan speed is: ");
  Serial.println(dataOutput);
}

void PWMrightFanControl(int PWMpin, int FanSpeed) {
  int dataOutput = map(FanSpeed, 0x00, 0x14, 0, 256);
  analogWrite(PWMpin, dataOutput);
  Serial.print("Fan speed is: ");
  Serial.println(dataOutput);
}