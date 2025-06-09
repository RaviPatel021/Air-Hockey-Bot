#ifndef GANTRY_H
#define GANTRY_H

#include <array>
#include <Arduino.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>
#include <GantryConfig.hpp>

class GantryClass{
  private:
    HardwareSerial* serials;
    TMC2209Stepper* drivers;
    FastAccelStepper* stepperR;
    FastAccelStepper* stepperL;
    FastAccelStepperEngine engine;
    std::array<int, 2> current_velocity_steps = {0,0};;
    float stepsPerCM;
    int widthStepsMotorL, widthStepsMotorR, heightStepsMotorL, heightStepsMotorR;
  public:
    GantryClass();

    std::array<int, 2> position_to_step(float x, float y);
    std::array<int, 2> velocity_to_step_speed(float vx, float vy);
    std::array<float, 2> step_to_position(int left_pos, int right_pos);
    void moveTo(int left_pos, int right_pos);

    /*
      Set the x and y velocity of the gantry
      returns true if successful
      returns false if action makes gantry go deeper in the unsafe zone 
    */
    bool setVelocity(float vx, float vy);

    void recalibrate(void);

    // return the x position in cm
    std::array<int, 2> getSteps();

    void initialize();

    // calibrate the gantry based on limit switch placement
    bool calibrate();

    /*
      Stops the Gantry Within 20ms
    */
    void eStop(void);
    void Stop(void);

    /*
      Stops the Gantry if limit switch contact detected
    */
    void enableGantryLimitINT();

    /*
      Remove Limit Switch Protection
    */
    void disableGantryLimitINT();

    /*
      moves x and y steps on the coordinate plane
    */
    void moveSteps(int x, int y);

    void setStepsPerCM(float newStepsPerCM);

    bool inSafeZone(void);
    bool isMovingSafely(void);

    bool hitLimit();

    ~GantryClass();
};

void polar2cartesian(float r, float theta, float xy_return[2]);



#endif