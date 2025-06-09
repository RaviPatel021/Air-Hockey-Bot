#include "Gantry.h"
#include "GantryConfig.hpp"
#include <array>


unsigned long lastTime = 0;
unsigned long lastTimeE = 0;
const unsigned long debounceDelay = 5000;
static GantryClass* gantry = nullptr;
volatile bool limitSwitchHit = false;
const float width = TABLE_WIDTH;
const float length = TABLE_HEIGHT;
const float paddle_radius = PADDLE_RADIUS;

#define RP(x) digitalRead(x)
#define LIMIT_TOP_MASK (0x1 << 3)
#define LIMIT_BOTTOM_MASK (0x1 << 2)
#define LIMIT_RIGHT_MASK (0x1 << 1)
#define LIMIT_LEFT_MASK (0x1 << 0)
#define DEBUG_EN true

int table_length_left;
int table_width_left;
int table_length_right;
int table_width_right;

int left_motor_bottom_left_position;
int right_motor_bottom_left_position;


static void waitForEnter(){
  bool enter_detected = false;
  while(!enter_detected){
    while(!Serial.available());
    char incoming = Serial.read();
    if (incoming == '\r' || incoming == '\n') {
      enter_detected = true;
    }
  }

}

static uint8_t readLimitSwitchState(void){
  return (RP(TOP_LIMIT)<<3)|(RP(BOTTOM_LIMIT)<<2)|(RP(RIGHT_LIMIT)<<1)|(RP(LEFT_LIMIT)<<0);
}

void IRAM_ATTR emergencyStopISR(){
  unsigned long currentTime = millis();
  if (currentTime - lastTimeE > debounceDelay){
    lastTimeE = currentTime;
    gantry->eStop();
  }
}

volatile bool lastLeftRailState = false;
volatile bool lastRightRailState = false;
volatile bool lastTopRailState = false;
volatile bool lastBottomRailState = false;


void IRAM_ATTR leftRailISR(){
  unsigned long currentTime = micros();
  bool currentLeftRailState = digitalRead(LEFT_LIMIT);
  if (currentTime - lastTime > debounceDelay && currentLeftRailState && !lastLeftRailState) {
    gantry->disableGantryLimitINT();
    lastTime = currentTime;
    limitSwitchHit = true;
  }
  lastLeftRailState = currentLeftRailState;
}

void IRAM_ATTR rightRailISR(){
  unsigned long currentTime = micros();
  bool currentRightRailState = digitalRead(RIGHT_LIMIT);
  if (currentTime - lastTime > debounceDelay && currentRightRailState && !lastRightRailState) {
    gantry->disableGantryLimitINT();
    lastTime = currentTime;
    limitSwitchHit = true;
  }
  lastRightRailState = currentRightRailState;
}

void IRAM_ATTR topRailISR(){
  unsigned long currentTime = micros();
  bool currentTopRailState = digitalRead(TOP_LIMIT);
  if (currentTime - lastTime > debounceDelay && currentTopRailState && !lastTopRailState) {
    gantry->disableGantryLimitINT();
    lastTime = currentTime;
    limitSwitchHit = true;
  }
  lastTopRailState = currentTopRailState;
}

void IRAM_ATTR bottomRailISR(){
  unsigned long currentTime = micros();
  bool currentBottomRailState = digitalRead(BOTTOM_LIMIT);
  if (currentTime - lastTime > debounceDelay && currentBottomRailState && !lastBottomRailState) {
    gantry->disableGantryLimitINT();
    lastTime = currentTime;
    limitSwitchHit = true;
  }
  lastBottomRailState = currentBottomRailState;
}

GantryClass::GantryClass() {}

GantryClass::~GantryClass(){
  Serial.print("Should not have been called");
  
} 

void GantryClass::initialize(){
  if (DEBUG_EN) {
    Serial.begin(115200);
    Serial.println("Creating GantryClass");
  }

  lastTime = 0;
  gantry = this;

  // Initialize Limit Switches
  pinMode(RIGHT_LIMIT, INPUT_PULLUP);
  pinMode(LEFT_LIMIT, INPUT_PULLUP);
  pinMode(BOTTOM_LIMIT, INPUT_PULLUP);
  pinMode(TOP_LIMIT, INPUT_PULLUP);


  // Use Serial1 for stepper driver communication
  serials = &Serial1;
  drivers = new TMC2209Stepper(serials, R_SENSE, DRIVER_ADDRESS);

  // Setup Motor Drivers
  serials->begin(DRIVER_BAUD, SERIAL_8N1, STEPPERS_RXD, STEPPERS_TXD);
  drivers->begin();
  drivers->toff(5);
  drivers->en_spreadCycle(true);
  drivers->microsteps(MICROSTEPS);

  delay(100);

  // Setup Stepper Motors with FastAccelStepper
  engine.init();

  stepperR = engine.stepperConnectToPin(MOTOR_STEP_PINR);
  stepperR->setDirectionPin(MOTOR_DIRECTION_PINR, DIR0_CLOCKWISE);
  stepperR->setEnablePin(MOTOR_ENABLE_PINR, true);
  stepperR->setAcceleration(MAX_ACCELERATION);
  stepperR->setSpeedInHz(MAX_SPEED);

  stepperL = engine.stepperConnectToPin(MOTOR_STEP_PINL);
  stepperL->setDirectionPin(MOTOR_DIRECTION_PINL, DIR0_CLOCKWISE);
  stepperL->setEnablePin(MOTOR_ENABLE_PINL, true);
  stepperL->setAcceleration(MAX_ACCELERATION);
  stepperL->setSpeedInHz(MAX_SPEED);

  stepperR->enableOutputs();
  stepperL->enableOutputs();

  if (DEBUG_EN) {
    Serial.println("Stepper Active");
  }
  enableGantryLimitINT();
}

std::array<int, 2> GantryClass::getSteps(){
  int left_pos = stepperL->getCurrentPosition();
  int right_pos = stepperR->getCurrentPosition();
  return {left_pos, right_pos};
}

bool GantryClass::calibrate(){
  int left_motor_top_left_position;
  int right_motor_top_left_position;
  int left_motor_top_right_position;
  int right_motor_top_right_position;
  int left_motor_top_middle_position;
  int right_motor_top_middle_position;
  int left_motor_bottom_middle_position;
  int right_motor_bottom_middle_position;

  // Disable interrupts and clear flags
  disableGantryLimitINT();
  uint8_t swState = readLimitSwitchState();

  // Set speed for calibration
  stepperR->setSpeedInHz(STEPS_PER_REVOLUTION * 2);
  stepperL->setSpeedInHz(STEPS_PER_REVOLUTION * 2);

  swState = readLimitSwitchState();
  if (!(swState & LIMIT_TOP_MASK)){
    // Begin Up calibration
    stepperR->move(STEPS_PER_REVOLUTION * 20);
    stepperL->move(-STEPS_PER_REVOLUTION * 20);

    while(!(swState & LIMIT_TOP_MASK)){
      delay(1);
      swState = readLimitSwitchState();
    }
  }
  eStop();
  delay(1000);

  Serial.println("Top Calibration Done");
  swState = readLimitSwitchState();
  if (!(swState & LIMIT_LEFT_MASK)){
    // Begin Left Calibration
    stepperR->move(STEPS_PER_REVOLUTION * 20);
    stepperL->move(STEPS_PER_REVOLUTION * 20);

    while(!(swState & LIMIT_LEFT_MASK)){
      delay(1);
      swState = readLimitSwitchState();
    }
  }
  eStop();
  delay(1000);

  left_motor_top_left_position = stepperL->getCurrentPosition();
  right_motor_top_left_position = stepperR->getCurrentPosition();

  Serial.println("Left Calibration Done");

  swState = readLimitSwitchState();
  if (!(swState & LIMIT_RIGHT_MASK)){
    // Begin Right Calibration
    stepperR->move(-STEPS_PER_REVOLUTION * 20);
    stepperL->move(-STEPS_PER_REVOLUTION * 20);
    

    while(!(swState & LIMIT_RIGHT_MASK)){
      delay(1);
      swState = readLimitSwitchState();
    }
  }
  eStop();
  delay(1000);

  left_motor_top_right_position = stepperL->getCurrentPosition();
  right_motor_top_right_position = stepperR->getCurrentPosition();

  Serial.printf("Left Motor Left Top: %d\n", left_motor_top_left_position);
  Serial.printf("Right Motor Left Top: %d\n", right_motor_top_left_position);
  Serial.printf("Left Motor Right Top: %d\n", left_motor_top_right_position);
  Serial.printf("Right Motor Right Top: %d\n", right_motor_top_right_position);
  auto width = left_motor_top_right_position - left_motor_top_left_position;
  Serial.printf("Width: %d\n", width);

  Serial.println("Right Calibration Done");

  //Move to the middle
  stepperR->moveTo(right_motor_top_right_position-width/2);
  stepperL->moveTo(left_motor_top_right_position-width/2, true);
  delay(1000);
  left_motor_top_middle_position = stepperL->getCurrentPosition();
  right_motor_top_middle_position = stepperR->getCurrentPosition();

  Serial.printf("Left Motor Top Middle: %d\n", left_motor_top_middle_position);
  Serial.printf("Right Motor Top Middle: %d\n", right_motor_top_middle_position);


  //move to the bottom
  swState = readLimitSwitchState();
  if(!(swState & LIMIT_BOTTOM_MASK)){
    stepperR->move(-STEPS_PER_REVOLUTION * 20);
    stepperL->move(STEPS_PER_REVOLUTION * 20);
    
    while(!(swState & LIMIT_BOTTOM_MASK)){
      delay(1);
      swState = readLimitSwitchState();
    }
  }
  eStop();
  delay(1000);
  left_motor_bottom_middle_position = stepperL->getCurrentPosition();
  right_motor_bottom_middle_position = stepperR->getCurrentPosition();
  
  Serial.printf("Left Motor Bottom Middle: %d\n", left_motor_bottom_middle_position);
  Serial.printf("Right Motor Bottom Middle: %d\n", right_motor_bottom_middle_position);

  Serial.println("Bottom Calibration Done");
  
  //Move to the middle
  stepperR->moveTo((right_motor_top_middle_position+right_motor_bottom_middle_position)/2);
  stepperL->moveTo((left_motor_top_middle_position+left_motor_bottom_middle_position)/2, true);
  delay(1000);
  enableGantryLimitINT();

  table_length_left = left_motor_top_middle_position - left_motor_bottom_middle_position;
  table_width_left = left_motor_top_right_position - left_motor_top_left_position;
  left_motor_bottom_left_position = left_motor_top_left_position - table_length_left;
  table_length_right = right_motor_top_middle_position - right_motor_bottom_middle_position;
  table_width_right = right_motor_top_right_position - right_motor_top_left_position;
  right_motor_bottom_left_position = right_motor_top_left_position - table_length_right;


  // table_length_left = -855;
  // table_width_left = -1465;
  // left_motor_bottom_left_position = 988 - table_length_left;
  // table_length_right = 855;
  // table_width_right = -1465;
  // right_motor_bottom_left_position = 988 - table_length_right;

  stepperL->setSpeedInHz(MAX_SPEED);
  stepperR->setSpeedInHz(MAX_SPEED);



  return true;

}

bool GantryClass::hitLimit(void){
  if (limitSwitchHit){
    limitSwitchHit = false;
    gantry->eStop();
    gantry->enableGantryLimitINT();
    return true;
  } 
  return false; 

}

void GantryClass::moveTo(int left_pos, int right_pos){
  stepperR->moveTo(right_pos);
  stepperL->moveTo(left_pos);  
  return;
}

std::array<int, 2> GantryClass::position_to_step(float x, float y){

  float new_y = constrain(y - paddle_radius, 0, length - 2 * paddle_radius);
  float new_x = constrain(x - paddle_radius, 0, width - 2 * paddle_radius);



  float ratio_length = new_y/(length - 2*paddle_radius);
  float ratio_width = new_x/(width - 2*paddle_radius);


  int left_pos = int(table_length_left*ratio_length + table_width_left*ratio_width) + left_motor_bottom_left_position;



  int right_pos = int(table_length_right*ratio_length + table_width_right*ratio_width) + right_motor_bottom_left_position;

  // Serial.println(int(table_length_right*ratio_length + table_width_right*ratio_width));
  // Serial.println(int(table_length_left*ratio_length + table_width_left*ratio_width));



  return {left_pos, right_pos};
}

std::array<float, 2> GantryClass::step_to_position(int left_pos, int right_pos) {
  float left_step_transormed = left_pos - left_motor_bottom_left_position;
  float right_step_transormed = right_pos - right_motor_bottom_left_position;
  // Serial.println(left_step_transormed);
  // Serial.println(right_step_transormed);


  float ratio_length = (1.0/((table_length_left*table_width_right)-(table_width_left*table_length_right)))*(table_width_right*left_step_transormed - table_width_left*right_step_transormed);
  float ratio_width = (1.0/((table_width_left*table_length_right)-(table_width_right*table_length_left)))*(table_length_right*left_step_transormed - table_length_left*right_step_transormed);

  // Serial.println(ratio_length);
  // Serial.println(ratio_width);

  float x = (ratio_width*(width - 2*paddle_radius)) + paddle_radius;
  float y = (ratio_length*(length - 2*paddle_radius)) + paddle_radius;

  return {x, y};
}

std::array<int, 2> GantryClass::velocity_to_step_speed(float vx, float vy){
  float ratio_length = vy/(length - 2*paddle_radius);
  float ratio_width = vx/(width - 2*paddle_radius);

  int left_pos = int(table_length_left*ratio_length + table_width_left*ratio_width);

  int right_pos = int(table_length_right*ratio_length + table_width_right*ratio_width);

  if (abs(left_pos) < 30){
    left_pos = 0;
  }
  if (abs(right_pos) < 30){
    right_pos = 0;
  }
  return {left_pos, right_pos};
}

void GantryClass::eStop(void){
  stepperL->forceStop();
  stepperR->forceStop();
  current_velocity_steps = {0, 0};
}

void GantryClass::Stop(void){
  stepperL->stopMove();
  stepperR->stopMove();
  current_velocity_steps = {0, 0};
}

bool GantryClass::setVelocity(float vx, float vy) {

  uint8_t swState = readLimitSwitchState();
  
  float new_vx = vx;
  float new_vy = vy;
  // left
  if ((swState & LIMIT_LEFT_MASK) && (vx < 0)){
    new_vx = 0;
  }
  //right
  if ((swState & LIMIT_RIGHT_MASK) && (vx > 0)){
    new_vx = 0;
  }
  //bottom
  if ((swState & LIMIT_BOTTOM_MASK) && (vy < 0)){
    new_vy = 0;
  }
  //top
  if ((swState & LIMIT_TOP_MASK) && (vy > 0)){
    new_vy = 0;
  }
  // Serial.printf("(%f,%f)\n", new_vx, new_vy);
  std::array<int, 2> new_step_velocity = velocity_to_step_speed(new_vx, new_vy);
  // Serial.printf("(%d,%d)\n", new_step_velocity[0], new_step_velocity[1]);


  bool left_stop = ((current_velocity_steps[0] < 0 && new_step_velocity[0] > 0) || (current_velocity_steps[0] > 0 && new_step_velocity[0] < 0) || new_step_velocity[0] == 0);
  bool right_stop = ((current_velocity_steps[1] < 0 && new_step_velocity[1] > 0) || (current_velocity_steps[1] > 0 && new_step_velocity[1] < 0) || new_step_velocity[1] == 0);

  if (left_stop || right_stop){
    if(left_stop){
      stepperL->stopMove();
    }
    if(right_stop){
      stepperR->stopMove();
    }
    delay(5);
  }
  
  stepperL->setSpeedInHz(abs(new_step_velocity[0]));
  stepperR->setSpeedInHz(abs(new_step_velocity[1]));

  if (new_step_velocity[0] != 0) {
    stepperL->move(new_step_velocity[0] > 0 ? 5000000 : -5000000);
  }

  if (new_step_velocity[1] != 0) {
    stepperR->move(new_step_velocity[1] > 0 ? 5000000 : -5000000);
  }

  current_velocity_steps = new_step_velocity;
  return true;
}

void GantryClass::enableGantryLimitINT(){
  attachInterrupt(digitalPinToInterrupt(RIGHT_LIMIT), rightRailISR, RISING);
  attachInterrupt(digitalPinToInterrupt(LEFT_LIMIT), leftRailISR, RISING);
  attachInterrupt(digitalPinToInterrupt(BOTTOM_LIMIT), bottomRailISR, RISING);
  attachInterrupt(digitalPinToInterrupt(TOP_LIMIT), topRailISR, RISING);
}

void GantryClass::disableGantryLimitINT(){
  detachInterrupt(RIGHT_LIMIT);
  detachInterrupt(LEFT_LIMIT);
  detachInterrupt(BOTTOM_LIMIT);
  detachInterrupt(TOP_LIMIT);
}