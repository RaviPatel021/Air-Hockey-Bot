// #include "ourtests.h"
// #include "GantryConfig.hpp"
// #include <Arduino.h>
// #include <TMCStepper.h>
// #include <FastAccelStepper.h>

// #define RP(x) digitalRead(x)
// #define LIMIT_TOP_MASK (0x1 << 4)
// #define LIMIT_BOT_MASK (0x1 << 3)
// #define LIMIT_RIGHT_MASK (0x1 << 1)
// #define LIMIT_LEFT_MASK (0x1 << 0)
// #define DEBUG_EN true

// static void waitForEnter(){
//   bool enter_detected = false;
//   while(!enter_detected){
//     while(!Serial.available());
//     char incoming = Serial.read();
//     if (incoming == '\r' || incoming == '\n') {
//       enter_detected = true;
//     }
//   }

// }

// static uint8_t readLimitSwitchState(void){
//   return ((RP(TOP_LIMIT_RIGHT)<<4)|(RP(BOTTOM_LIMIT_LEFT)<<3)|(RP(RIGHT_LIMIT)<<1)|(RP(LEFT_LIMIT)<<0));
// }


// int test_limit_switches(void){
//   Serial.begin(115200);
//   pinMode(RIGHT_LIMIT, INPUT);
//   pinMode(LEFT_LIMIT, INPUT);
//   pinMode(BOTTOM_LIMIT_LEFT, INPUT);
//   pinMode(BOTTOM_LIMIT_RIGHT, INPUT);
//   pinMode(TOP_LIMIT_LEFT, INPUT);
//   pinMode(TOP_LIMIT_RIGHT, INPUT);
//   uint8_t last_sw_state = readLimitSwitchState();
//   while(1){
//     uint8_t switch_state = readLimitSwitchState();
//     if(last_sw_state != switch_state){
//       Serial.printf("Left Middle Pressed: %d\n", (switch_state&LIMIT_LEFT_MASK) >= 1);
//       Serial.printf("Left Bottom Pressed: %d\n", (switch_state&LIMIT_BOT_MASK) >= 1);
//       Serial.printf("Right Top Pressed: %d\n", (switch_state&LIMIT_TOP_MASK) >= 1);
//       Serial.printf("Right Mid Pressed: %d\n", (switch_state&LIMIT_RIGHT_MASK) >= 1);
//       Serial.printf("Total State: %x\n", switch_state);
//       Serial.println();
//       last_sw_state = switch_state;
//     }
//   }
// }

// FastAccelStepperEngine engine;

// // Does the setSpeed method actually set speed or max speed
// // No it doesn't, it is only used for max speed
// // Do the steppers we have go clockwise or anticlockwise for dirPin=0
// int test_stepper_motor_connections(){
//   HardwareSerial* serials = new HardwareSerial(STEPPERS_SERIAL_NUM);
//   TMC2209Stepper* drivers = new TMC2209Stepper(serials, R_SENSE, DRIVER_ADDRESS);

//   // Setup Right Motor Configs
//   serials->begin(DRIVER_BAUD, SERIAL_8N1, STEPPERS_RXD, STEPPERS_TXD);
//   drivers->begin();
//   drivers->toff(5);
//   drivers->en_spreadCycle(true);
//   drivers->microsteps(MICROSTEPS);

//   delay(100);

//   // Setup Stepper Motors with FastAccelStepper Library
//   engine.init();

//   FastAccelStepper* stepperR = engine.stepperConnectToPin(MOTOR_STEP_PINR);
//   stepperR->setDirectionPin(MOTOR_DIRECTION_PINR, DIR0_CLOCKWISE);
//   // stepperR->setEnablePin(MOTOR_ENABLE_PINR, true);
//   stepperR->setAcceleration(MAX_ACCELERATION);
//   stepperR->setSpeedInHz(MAX_SPEED);

//   FastAccelStepper* stepperL = engine.stepperConnectToPin(MOTOR_STEP_PINL);
//   stepperL->setDirectionPin(MOTOR_DIRECTION_PINL, DIR0_CLOCKWISE);
//   // stepperL->setEnablePin(MOTOR_ENABLE_PINL, true);
//   stepperL->setAcceleration(MAX_ACCELERATION);
//   stepperL->setSpeedInHz(MAX_SPEED);

//   Serial.begin(115200);
//   // if (stepperL->enableOutputs()){
//   //   Serial.println("Stepper Left Active");
//   // }
//   // if(stepperR->enableOutputs()){
//   //   Serial.println("Stepper Right Active");
//   // }
//   unsigned long long location = 0;
//   while(1){
//     Serial.println("Is it moving?");
//     stepperL->moveTo(200, true);
//     stepperR->moveTo(200, true);
//     delay(2000);
//     stepperR->moveTo(-200);
//     stepperL->moveTo(-200, true);
//     delay(2000);
//   }
// }



