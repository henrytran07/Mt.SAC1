#ifndef OUTTAKE 
#define OUTTAKE 

#include "robot_config.h"
#include "vex.h"

using namespace vex; 
using namespace std; 

const string team_color = "RED";
bool rejection_in_progress = false;
bool sorting_paused = false;     
bool pauseIntake_distance = false; 

bool distanceOverride = false; 
bool lastDistanceOverride = false; 
double prevDistance = -1;
bool wasSeeingObject = false;


bool opponentHue(double hue) {
    if (team_color == "RED") return hue >= 210 && hue <= 270; 
    else return hue >= 330 || hue <= 30; 
}

void scoringTopGoal();
void scoringMidGoal();
void toggle_outtake_tg(); 

bool pauseGuard() {
    bool overrideSortingBtn = Controller1.ButtonDown.pressing();

    if (overrideSortingBtn && !prevOverrideSortingBtn) {
        sorting_paused = !sorting_paused;
    }

    prevOverrideSortingBtn = overrideSortingBtn;

    if (sorting_paused) {
        stop_mechanism_motors();
        Controller1.rumble("-");
        return true;
    }
    return false;
}

void updateOverrideDistanceToggle() {
    bool overrideDistanceBtn = Controller1.ButtonLeft.pressing(); 

    if (overrideDistanceBtn && !prevOverrideDistance) {
        distanceOverride = !distanceOverride;
    }

    if (distanceOverride) {
        pauseIntake_distance = false;  
    }
    prevOverrideDistance = overrideDistanceBtn; 
}

void handleOverrideTransitionDistanceResync() {
    if (!distanceOverride && lastDistanceOverride) {
        wasSeeingObject = distance_sensor.isObjectDetected(); 
        prevDistance = -1; 
    }
    lastDistanceOverride = distanceOverride; 
}

void rejectToMid() {
    chainMotor.setVelocity(100, percent);
    chainMotor.spin(vex::reverse);

    bottomIntakeMotor.setVelocity(100, percent);
    bottomIntakeMotor.spin(vex::reverse);

    middleIntakeMotor.setVelocity(100, percent);
    middleIntakeMotor.spin(vex::forward);
}

void rejectToTop() {
    bottomIntakeMotor.stop(brakeType::hold);
    chainMotor.setVelocity(100, percent);
    chainMotor.spin(vex::reverse);
    middleIntakeMotor.setVelocity(100, percent);
    middleIntakeMotor.spin(vex::reverse);

    topIntakeMotor.setVelocity(100, percent);
    topIntakeMotor.spin(vex::forward);
}

void scoringLowGoal() {
    if (pauseGuard()) return; 
    chainMotor.setVelocity(100, percent);
    chainMotor.spin(vex::forward);
    
    if (!rejection_in_progress && optical_sensor.isNearObject()) {
        double hue = optical_sensor.hue(); 
        if (opponentHue(hue)) {
            rejection_in_progress = true;
            timer t; t.reset(); 

            while (t.time(msec) < 500) {
                if (pauseGuard()) {
                    rejection_in_progress = false; 

                    stop_mechanism_motors(); 

                    Controller1.Screen.clearScreen();
                    Controller1.Screen.setCursor(1, 1);
                    Controller1.Screen.print("Driver override: stop LOW");
                    return; 
                } 
                rejectToMid(); 
                wait(10, msec);
            }
            rejection_in_progress = false;
            return;
        }
    }

    //front wheels 
    frontWheelMotor1.setVelocity(100, percent);
    frontWheelMotor1.spin(vex::forward);
    frontWheelMotor2.setVelocity(100, percent); 
    frontWheelMotor2.spin(vex::reverse);
}

void scoringTopGoal() {
    if (pauseGuard()) return; 
    optical_sensor.setLight(ledState::on);
    optical_sensor.setLightPower(50, percent);

    if (!rejection_in_progress && optical_sensor.isNearObject()) {
        double hue = optical_sensor.hue(); 
        if (opponentHue(hue)) {
            rejection_in_progress = true;
            timer t; t.reset(); 

            while (t.time(msec) < 500) {
                if (pauseGuard()) {
                    rejection_in_progress = false; 

                    stop_mechanism_motors(); 

                    Controller1.Screen.clearScreen();
                    Controller1.Screen.setCursor(1, 1);
                    Controller1.Screen.print("Driver override: stop TOP");
                    return; 
                } 
                rejectToMid(); 
                wait(10, msec);
            }
            rejection_in_progress = false;
            return;
        }
    }
    
    bottomIntakeMotor.stop(brakeType::hold);
    chainMotor.setVelocity(100, percent);
    chainMotor.spin(vex::reverse);

    middleIntakeMotor.setVelocity(100, percent);
    middleIntakeMotor.spin(vex::reverse);

    topIntakeMotor.setVelocity(100, percent);
    topIntakeMotor.spin(vex::forward);
}

void scoringMidGoal() {
    if (pauseGuard()) return; 
    optical_sensor.setLight(ledState::on);
    optical_sensor.setLightPower(50, percent);
    
    if (!rejection_in_progress && optical_sensor.isNearObject()) {
        double hue = optical_sensor.hue(); 
        if (opponentHue(hue)) {
            rejection_in_progress = true;
            timer t; t.reset(); 
            
            while (t.time(msec) < 500) {
                if (pauseGuard()) {
                    rejection_in_progress = false; 

                    stop_mechanism_motors(); 

                    Controller1.Screen.clearScreen();
                    Controller1.Screen.setCursor(1, 1);
                    Controller1.Screen.print("Driver override: stop MID");

                    return; 
                }
                rejectToTop(); 
                wait(10, msec);
            }
            rejection_in_progress = false;
            return;
        }
    }

    chainMotor.setVelocity(100, percent);
    chainMotor.spin(vex::reverse);

    bottomIntakeMotor.setVelocity(100, percent);
    bottomIntakeMotor.spin(vex::reverse);

    middleIntakeMotor.setVelocity(100, percent);
    middleIntakeMotor.spin(vex::forward);

    topIntakeMotor.stop(brakeType::hold);
}

void toggle_intake() {
    activate_intake = !activate_intake; 
    if (activate_intake){
        Brain.Screen.clearLine(1);
        Controller1.Screen.setCursor(1, 1);
        Controller1.Screen.print("MODE: INTAKE       ");
    }

    activate_two_front_wheels = true;
    if (activate_outtake_low_goal) {
        activate_outtake_low_goal = !activate_outtake_low_goal;
        prevOuttakeButton_lg = !prevOuttakeButton_lg; 
    }

    if (activate_outtake_top_goal) {
        activate_outtake_top_goal = !activate_outtake_top_goal; 
        prevOuttakeButton_tg = !prevOuttakeButton_tg;
    }
    if (activate_outtake_mid_goal) {
        activate_outtake_mid_goal = !activate_outtake_mid_goal; 
        prevOuttakeButton_mg = !prevOuttakeButton_mg;
    }
}

void toggle_pneumatics() {
    activate_pneumatics = !activate_pneumatics; 
}

void toggle_outtake_lg() {
    activate_outtake_low_goal = !activate_outtake_low_goal;
    if (activate_outtake_low_goal) {
            Brain.Screen.clearLine(1);
            Controller1.Screen.setCursor(1, 1);
            Controller1.Screen.print("MODE: LOW          ");
    }

    if (activate_intake) {
        activate_intake = !activate_intake; 
        prevIntakeButton = !prevIntakeButton;
    }
    if (!activate_two_front_wheels) activate_two_front_wheels = true;

    if (activate_outtake_top_goal) {
        activate_outtake_top_goal = !activate_outtake_top_goal; 
        prevOuttakeButton_tg = !prevOuttakeButton_tg;
    }
    if (activate_outtake_mid_goal) {
        activate_outtake_mid_goal = !activate_outtake_mid_goal; 
        prevOuttakeButton_mg = !prevOuttakeButton_mg;
    }

}

void toggle_outtake_tg() {
    activate_outtake_top_goal = !activate_outtake_top_goal; 
    if (activate_outtake_top_goal) {
        Brain.Screen.clearLine(1);
        Controller1.Screen.setCursor(1, 1);
        Controller1.Screen.print("MODE: TOP          ");
    }

    if (activate_intake) {
        activate_intake = !activate_intake; 
        prevIntakeButton = !prevIntakeButton;
    }
    if (activate_two_front_wheels) {
        activate_two_front_wheels = !activate_two_front_wheels; 
    }
    if (activate_outtake_low_goal) {
        activate_outtake_low_goal = !activate_outtake_low_goal;
        prevOuttakeButton_lg = !prevOuttakeButton_lg; 
    }
    if (activate_outtake_mid_goal) {
        activate_outtake_mid_goal = !activate_outtake_mid_goal; 
        prevOuttakeButton_mg = !prevOuttakeButton_mg;
    }
}

void toggle_outtake_mg() {
    activate_outtake_mid_goal = !activate_outtake_mid_goal; 
    if (activate_outtake_mid_goal) {
        Brain.Screen.clearLine(1);
        Controller1.Screen.setCursor(1, 1);
        Controller1.Screen.print("MODE: MID          ");
    }

    if (activate_intake) {
        activate_intake = !activate_intake; 
        prevIntakeButton = !prevIntakeButton;
    }
    if (activate_two_front_wheels) {
        activate_two_front_wheels = !activate_two_front_wheels; 
    }
    if (activate_outtake_low_goal) {
        activate_outtake_low_goal = !activate_outtake_low_goal;
        prevOuttakeButton_lg = !prevOuttakeButton_lg; 
    }
    if (activate_outtake_top_goal) {
        activate_outtake_top_goal = !activate_outtake_top_goal; 
        prevOuttakeButton_tg = !prevOuttakeButton_tg;
    }

}

void activateTheIntake() {
    updateOverrideDistanceToggle(); 
    handleOverrideTransitionDistanceResync(); 

    if (pauseGuard()) return; 

    // if (!distanceOverride) {
    //     Controller1.Screen.clearScreen();
    //     Controller1.Screen.setCursor(1, 1);
    //     Controller1.Screen.print(" Balls: %d", countBalls);
    //     bool seeingObject = distance_sensor.isObjectDetected();
    //     if (seeingObject && !wasSeeingObject){
    //         double currentDistance = distance_sensor.objectDistance(mm);
    //         if (currentDistance > 0 && currentDistance < 500){
    //             if ((prevDistance < 0 || fabs(prevDistance - currentDistance) > 10) && countBalls < 5) {
    //                 countBalls += 1;
    //                 prevDistance = currentDistance; 
    //             }

    //             if (countBalls >= 5) {
    //                 pauseIntake_distance = true;  
    //                 stop_mechanism_motors(); 
    //                 Controller1.Screen.clearScreen();
    //                 Controller1.Screen.setCursor(1,1);
    //                 Controller1.Screen.print("STOP INTAKE: 5 BALLS");
    //                 Controller1.rumble("- -");
    //                 return; 
    //             }
    //         }
    //     }
    //     if (!seeingObject) prevDistance = -1; 
    //     wasSeeingObject = seeingObject;
    // } else {
    //     Controller1.Screen.clearScreen();
    //     Controller1.Screen.setCursor(1, 1);
    //     Controller1.Screen.print("DIST OFF | Balls: %d", countBalls);
    // }

    optical_sensor.setLight(ledState::on);
    optical_sensor.setLightPower(50, percent);

    if (optical_sensor.isNearObject()) {
        double hue = optical_sensor.hue(); 
        if (!rejection_in_progress && opponentHue(hue)) {
            timer t; t.reset(); 
            rejection_in_progress = true; 
            while (t.time(msec) < 500){
                if (pauseGuard()){
                    rejection_in_progress = false; 

                    Controller1.Screen.clearScreen();
                    Controller1.Screen.setCursor(1, 1);
                    Controller1.Screen.print("Driver override: stop INTAKE");
                    return; 
                }
                rejectToMid(); 
                wait(10, msec);
            }
        }
    } 

    chainMotor.setVelocity(100, percent);
    chainMotor.spin(vex::reverse);
    //front wheels 
    frontWheelMotor1.setVelocity(100, percent);
    frontWheelMotor1.spin(vex::reverse);
    frontWheelMotor2.setVelocity(100, percent);
    frontWheelMotor2.spin(vex::forward);

    bottomIntakeMotor.stop(brakeType::hold);
    middleIntakeMotor.stop(brakeType::hold);
    topIntakeMotor.stop(brakeType::hold);
}

void activatePneumatics() {
    pneu_front_wheel1.set(activate_pneumatics);
    pneu_front_wheel2.set(activate_pneumatics);
}

void handleButton() {
    bool intakeButton = Controller1.ButtonB.pressing(); 
    bool outtakeButton_lg = Controller1.ButtonA.pressing(); 
    bool outtakeButton_tg = Controller1.ButtonX.pressing(); 
    bool outtakeButton_mg = Controller1.ButtonY.pressing(); 
    bool pneumaticButton = Controller1.ButtonUp.pressing(); 

    if (intakeButton && !prevIntakeButton) toggle_intake(); 
    if (outtakeButton_lg && !prevOuttakeButton_lg) toggle_outtake_lg(); 
    if (outtakeButton_tg && !prevOuttakeButton_tg) toggle_outtake_tg();
    if (outtakeButton_mg && !prevOuttakeButton_mg) toggle_outtake_mg(); 
    if (pneumaticButton && !prevPneumaticButton) toggle_pneumatics(); 

    prevIntakeButton = intakeButton; 
    prevOuttakeButton_lg = outtakeButton_lg; 
    prevOuttakeButton_tg = outtakeButton_tg;
    prevOuttakeButton_mg = outtakeButton_mg;
    prevPneumaticButton = pneumaticButton;
}

int outakeTask() {
    while (true) {
        handleButton(); 
        activatePneumatics(); 

        if (!activate_two_front_wheels) {
            frontWheelMotor1.stop(vex::coast);
            frontWheelMotor2.stop(vex::coast);
        }

        if ((!activate_intake && !activate_outtake_mid_goal && !activate_outtake_top_goal) || sorting_paused) {
            optical_sensor.setLight(ledState::off);
        }

        if (activate_outtake_top_goal && activate_outtake_mid_goal && activate_outtake_low_goal)
            countBalls = 0; 

        vex::task::sleep(10);
    }

    return 0; 
}

#endif 
