#include <Gantry.h>
#include <array>


GantryClass gantry;

void setup() {
  // put your setup code here, to run once:
  delay(4000);
  gantry.initialize();
  gantry.calibrate();
}

int max_vel = 100;
int max_accel = 500;
int FPS = 100;

void loop() {
  if(gantry.hitLimit()){
    gantry.eStop();
    // Serial.println("limit switch hit");
  }
  if (Serial.available()) {
    // unsigned long start_time = micros();
    String input = Serial.readStringUntil('\n');
    int commaIndex = input.indexOf(',');
 
    if (commaIndex != -1) {
      //Read velocity value from the serial monitor
      float vx = input.substring(0, commaIndex).toFloat();
      float vy = input.substring(commaIndex + 1).toFloat();
 
      //Limit the max velocity
      float new_vy = constrain(vy, -1*max_vel, max_vel);
      float new_vx = constrain(vx, -1*max_vel, max_vel);
      gantry.setVelocity(new_vx, new_vy);

    }
  }
}
 