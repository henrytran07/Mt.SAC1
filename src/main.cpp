
#include "driving.h"
#include "vex.h"
#include "autonomous_driving.h"
using namespace vex;

bool isAutonomousMode = false; 
bool arcade = true; 


/*
    From left to right: 5.5 in
    diameter: 3 inc
    left --> middle wheel's center: 3 in

    center to the front: 4.5 in
*/

competition Comp; 
int main() {
    vexcodeInit();
    // int waitTime = 20; 
    // // while (true) {
    //     // while (Brain.Timer.value() < waitTime) {
    //     //     if (Controller1.ButtonL1.pressing() || Controller1.ButtonR1.pressing()) {
    //     //         isAutonomousMode = true; 
    //     //         wait(0.2, sec); 
    //     //         break; 
    //     //     }

    //     //     if (Controller1.ButtonL2.pressing()) {
    //     //         isAutonomousMode = false; 
    //     //         wait(0.2, sec); 
    //     //         break;
    //     //     }

    //     //     if (Controller1.ButtonR2.pressing()) {
    //     //         arcade = false; 
    //     //         wait(0.2, sec);
    //     //         break;
    //     //     }

    //     //     vex::task::sleep(20);  
    //     // }

    //     // if (isAutonomousMode) {
    //     //     autonomous();
    //     // } else {
    //     //     driverControl(arcade);
    //     // }

    //     // driverControl();
    //     autonomous();

    //     // vex::task::sleep(20);
    
    // // while (true) {
    // //     Brain.Screen.clearLine(1);
    // //     intake(); 
    // // }

    Comp.autonomous(autonomous);
    Comp.drivercontrol(driverControl);
    while (true) wait(20, msec);
}