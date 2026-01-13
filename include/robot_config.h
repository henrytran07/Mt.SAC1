#ifndef ROBOT_CONFIG 
#define ROBOT_CONFIG 

#include "vex.h"

using namespace vex;


// empty ports: 6, 15, 16, 17, 21
brain Brain;

motor LeftMotor1 = motor(PORT1, ratio36_1, false);
motor LeftMotor2 = motor(PORT2, ratio36_1, false);
motor LeftMotor3 = motor(PORT3, ratio36_1, true);

motor RightMotor1 = motor(PORT8, ratio36_1, false);
motor RightMotor2 = motor(PORT9, ratio36_1, false);
motor RightMotor3 = motor(PORT10, ratio36_1, true);

motor chainMotor = motor(PORT11, ratio36_1, false);
motor middleIntakeMotor = motor(PORT12, ratio36_1, true);
motor bottomIntakeMotor = motor(PORT4, ratio36_1, true);
motor topIntakeMotor = motor(PORT15, ratio36_1, false);

motor frontWheelMotor1 = motor(PORT5, ratio36_1, false);
motor frontWheelMotor2 = motor(PORT7, ratio36_1, false);


rotation rotVert = rotation(PORT19, false);
rotation rotHor = rotation(PORT20, false);
inertial inertial_sen = inertial(PORT18);

optical optical_sensor = optical(PORT13);
vex::distance distance_sensor = vex::distance(PORT14);

digital_out pneu_front_wheel1 = digital_out(Brain.ThreeWirePort.A);
digital_out pneu_front_wheel2 = digital_out(Brain.ThreeWirePort.B);


controller Controller1 = controller(primary);
vex::motor_group LeftGroup(LeftMotor1, LeftMotor2, LeftMotor3);
vex::motor_group RightGroup(RightMotor1, RightMotor2, RightMotor3);

// # of balls 
int countBalls = 0; 

// mechanism activation flags 
bool activate_intake = false; 
bool activate_two_front_wheels = false; 
bool activate_chains = false; 
bool activate_outtake_low_goal = false; 
bool activate_outtake_top_goal = false; 
bool activate_outtake_mid_goal = false; 
bool activate_pneumatics = false; 


// prevButton flags 
bool prevIntakeButton = false; 
bool prevOuttakeButton_lg = false; 
bool prevOuttakeButton_tg = false; 
bool prevOuttakeButton_mg = false;
bool prevPneumaticButton = false; 
bool prevOverrideSortingBtn = false;    
bool prevOverrideDistance = false; 

void initializeRandomSeed(){
    int systemTime = Brain.Timer.systemHighResolution();
    double batteryCurrent = Brain.Battery.current();
    double batteryVoltage = Brain.Battery.voltage(voltageUnits::mV);

    int seed = int(batteryVoltage + batteryCurrent * 100) + systemTime;

    srand(seed);
}

void initIMU() {
    inertial_sen.calibrate();
    while (inertial_sen.isCalibrating()) wait(10, msec);
}

void initOpticalSensor() {
    optical_sensor.integrationTime(5);
    optical_sensor.objectDetectThreshold(100);
}

void vexcodeInit() {
    initializeRandomSeed(); 
    initIMU(); 
    initOpticalSensor(); 
}

void stop_mechanism_motors() {
    chainMotor.stop(vex::brakeType::hold); 

    middleIntakeMotor.stop(vex::brakeType::hold);
    bottomIntakeMotor.stop(vex::brakeType::hold);
    topIntakeMotor.stop(vex::brakeType::hold);

    frontWheelMotor1.stop(vex::brakeType::coast);
    frontWheelMotor2.stop(vex::brakeType::coast);
}
#endif 