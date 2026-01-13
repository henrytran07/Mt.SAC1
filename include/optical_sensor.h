#ifndef OPTICAL_SENSOR 
#define OPTICAL_SENSOR 

#include "vex.h"
#include "robot_config.h"
#include <tuple>

using namespace vex; 
using namespace std; 

double get_hue_color() {
    double hue_color = optical_sensor.hue(); 
    return hue_color; 
}

void lost() {
  Brain.Screen.clearLine(1);
  Brain.Screen.print("object lost");
}

bool is_red(double hue_value) {
    return (hue_value >= 340 && hue_value < 360) || (hue_value >= 0 && hue_value <= 20);
}

bool is_blue(double hue_value) {
    return (hue_value >= 180 && hue_value <= 240);
}

void adjustAndDetectObject() {
    double brightness_value = optical_sensor.brightness(); 
    double light_power = 100 - brightness_value; 
    Brain.Screen.clearLine(1);
    Brain.Screen.print("Light power: %.2f", light_power);
    if (light_power < 20)  
        light_power = 20; 

    optical_sensor.setLight(ledState::on);
    optical_sensor.setLightPower(light_power, percent);

    if (optical_sensor.isNearObject()) {
        double hue_value = get_hue_color(); 
        Brain.Screen.print("Hue color: %.2f", hue_value);
        if (is_red(hue_value)) 
            Brain.Screen.print("Red Object Detected");
        else if (is_blue(hue_value))
            Brain.Screen.print("Blue Object Detected");
        else 
            Brain.Screen.print("Object Detected, but not Red or Blue");
    } else {
        optical_sensor.objectLost(lost);
    }
}
#endif 