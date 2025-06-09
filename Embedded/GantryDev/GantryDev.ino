#include <ourtests.h>
#include <Gantry.h>
#include <array>


GantryClass gantry;

void setup() {
  // put your setup code here, to run once:
  delay(2000);
  delay(2000);
  gantry.initialize();
  gantry.calibrate();
}

int max_vel = 100;
int max_accel = 500;
int FPS = 100;

// void loop() {

//   if (!gantry.isMovingSafely()){
//     gantry.eStop();
//   }
  
//   if (Serial.available()) {
//     String input = Serial.readStringUntil('\n');
//     int commaIndex = input.indexOf(',');

//     if (commaIndex != -1) {
//       //Read velocity value from the serial monitor
//       float x = input.substring(0, commaIndex).toFloat();
//       float y = input.substring(commaIndex + 1).toFloat();

//       //Limit the max velocity
//       float new_y = constrain(y, PADDLE_RADIUS, TABLE_HEIGHT - PADDLE_RADIUS);
//       float new_x = constrain(x, PADDLE_RADIUS, TABLE_WIDTH - PADDLE_RADIUS);

//       Serial.print("x: ");
//       Serial.print(new_x);
//       Serial.print(", y: ");
//       Serial.println(new_y);
//       auto pos_step = gantry.position_to_step(x, y);
//       gantry.moveTo(pos_step[0], pos_step[1]);

//       // if(gantry.setPosition(new_vx, new_vy)){
//       //   Serial.println("YEY");
//       // }else{
//       //   Serial.println("NOO");
//       // }

//     }
//   }
// }


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

      // // End timing
      // unsigned long end_time = micros();
      // unsigned long duration = end_time - start_time;

      // Serial.print("Command processed in: ");
      // Serial.print(duration);
      // Serial.println(" us");
 
      // if(gantry.setVelocity(new_vx, new_vy)){
      // Serial.printf("(%f,%f)\n", new_vx, new_vy);
      // }else{
        // Serial.printf("(0,0)\n");
      // }
    }
  }
}
 