#ifndef AUTONOMOUS_DRIVING 
#define AUTONOMOUS_DRIVING 

#include "robot_config.h"
#include "driving.h"
#include "auto_drive.h"
#include "pid_control.h"
#include "routine_config.h"
#include "routine_config.h"


#include <utility>
#include <tuple>
#include <cmath>


using namespace vex; 
using namespace std; 

void autonomous() {
    // Brain.Screen.clearLine(1);
    Brain.Screen.print("Switched to Driving Autonomous mode");
    AutoDrive drive; 
    map<Key, Routine> routine_map = routineId();
    runRoutine(routine_map, RoutineID::R0, RIGHT, drive);
    
    // Brain.Screen.clearLine(); 

}

#endif 
