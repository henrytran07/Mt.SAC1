#ifndef DRIVING
#define DRIVING 

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <list> 
#include <cmath>
#include <iostream>

#include "vex.h"
#include "robot_config.h"
#include "outake.h"

using namespace vex; 


bool checkSafety() {
    if (LeftGroup.temperature(celsius) > 75) {
        Brain.Screen.print("Warning: Motor temperature is above 75°C! Motor might overheat.");
        return false;  
    } 
    else if (LeftGroup.temperature(celsius) > 50) {
        Brain.Screen.print("Temperature is above 50°C... Inefficiency might occur.");
        return true; 
    }
    else {
        return true; 
    }
}


void setBrakeForIndividualMotors(const brakeType& mode) {
    LeftMotor1.setBrake(mode);
    LeftMotor2.setBrake(mode);
    LeftMotor3.setBrake(mode);

    RightMotor1.setBrake(mode);
    RightMotor2.setBrake(mode);
    RightMotor3.setBrake(mode);
}

int forward_speed_sent_into_brain  = 0; 
int turn_speed_sent_into_brain  = 0;

const double forward_gain = 0.1; 
const double turn_gain = 1;
int tankDriveTask() {
    setBrakeForIndividualMotors(brakeType::brake);

    while (true) {
        int forwardSpeed = Controller1.Axis3.position(percent);
        int turnSpeed    = -Controller1.Axis1.position(percent);

        if (abs(forwardSpeed) < 5) forwardSpeed = 0;
        if (abs(turnSpeed) < 5) turnSpeed = 0;

        forward_speed_sent_into_brain += forward_gain * (forwardSpeed - forward_speed_sent_into_brain);
        turn_speed_sent_into_brain    += turn_gain * (turnSpeed - turn_speed_sent_into_brain);

        int leftTurn  = forward_speed_sent_into_brain - turn_speed_sent_into_brain;
        int rightTurn = forward_speed_sent_into_brain + turn_speed_sent_into_brain;

        leftTurn  = std::max(-60, std::min(60, leftTurn));
        rightTurn = std::max(-60, std::min(60, rightTurn));

        LeftGroup.setVelocity(leftTurn, percent);
        RightGroup.setVelocity(rightTurn, percent);

        LeftGroup.spin(vex::forward);
        RightGroup.spin(vex::reverse);

        vex::task::sleep(10);  
    }
    return 0;
}



int shared_intake_lg_tasks() {
    while (true) {
        if (activate_intake) {
            activateTheIntake(); 
        }
        else if (activate_outtake_low_goal) {
            scoringLowGoal();
        } else if (activate_outtake_top_goal){
            scoringTopGoal(); 
        } else if (activate_outtake_mid_goal) {
            scoringMidGoal(); 
        } else stop_mechanism_motors(); 
        
        vex::task::sleep(10);
    }
    return 0; 
}

void driverControl() {
    vex::task outake(outakeTask);
    vex::task shared_task(shared_intake_lg_tasks);
    
    vex::task driveTask(tankDriveTask);
    
    while (true) {
        vex::task::sleep(10);
    }

}
 
#endif 




