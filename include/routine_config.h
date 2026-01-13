#ifndef ROUTINE_CONFIG
#define ROUTINE_CONFIG

#include "auto_drive.h"
#include "vex.h"
#include "odometry.h"
#include "outake.h"

#include <vector>
#include <map>
#include <functional>
#include <tuple>

using namespace vex;
using namespace std;

class AutoDrive;

enum Side { LEFT, RIGHT };

enum RoutineID { R0, R1, R2, R3 };

struct Step {
    enum Type { STRAIGHT, BACKWARD, TURN_LEFT, TURN_RIGHT, TOP_GOAL, STOP, BALL_LOADING, RESET } type;
    double value;
};

using RoutineStep = function<void(AutoDrive&, Odometry&)>;
using Routine     = vector<RoutineStep>;
using Key         = pair<RoutineID, Side>;

static tuple<double, double, double> getInitialDataForOdometry() {
    double initial_inertial = inertial_sen.rotation(degrees);
    double enc_forward = rotVert.position(degrees);
    double enc_side    = rotHor.position(degrees);
    return make_tuple(initial_inertial, enc_forward, enc_side);
}

static void printOdom(Odometry& odom) {
    double x = odom.getX();
    double y = odom.getY();
    double h = odom.get_headings();

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("x: %.2f", x);
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("y: %.2f", y);
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("heading: %.2f", h);
}

static void addStraight(Routine& actions, double distance) {
    actions.push_back([distance](AutoDrive &d, Odometry& odom) {
        d.goStraightPID(distance);
        odom.update();
        printOdom(odom);
    });
}

static void addBackward(Routine& actions, double distance) {
    actions.push_back([distance](AutoDrive &d, Odometry& odom) {
        d.goBackwardPID(distance);
        odom.update();
        printOdom(odom);
    });
}

static void addTurnLeft(Routine& actions, double angle) {
    actions.push_back([angle](AutoDrive &d, Odometry& odom) {
        double current = inertial_sen.rotation(degrees);
        d.turnToHeadingPID(current - angle);

        odom.update();
        printOdom(odom);
    });
}

static void addTurnRight(Routine& actions, double angle) {
    actions.push_back([angle](AutoDrive &d, Odometry& odom) {
        double current = inertial_sen.rotation(degrees);
        d.turnToHeadingPID(current + angle);
        odom.update();
        printOdom(odom);
    });
}

static void addTopGoal(Routine& actions) {
    actions.push_back([](AutoDrive &d, Odometry& odom) {
        timer t; t.reset(); 
        while (t.time(msec) < 1000) {
            scoringTopGoal();
            wait(10, msec);
        }
        odom.update();
        printOdom(odom);
    });
}

static void addBallLoading(Routine& actions, double wait_time) {
    actions.push_back([wait_time](AutoDrive& d, Odometry& odom) {
        wait(wait_time, sec);
        odom.update();
        printOdom(odom);
    });
}

static void addMotorStop(Routine& actions) {
    actions.push_back([](AutoDrive &d, Odometry& odom) {
        stop_mechanism_motors();  
        odom.update();
        printOdom(odom);
    });
}

static Routine buildRoutine(const vector<Step>& steps) {
    Routine actions;
    for (const auto &s : steps) {
        switch (s.type) {
            case Step::STRAIGHT:      addStraight(actions, s.value); break;
            case Step::BACKWARD:      addBackward(actions, s.value); break;
            case Step::TURN_LEFT:     addTurnLeft(actions, s.value); break;
            case Step::TURN_RIGHT:    addTurnRight(actions, s.value); break;
            case Step::TOP_GOAL:      addTopGoal(actions); break;
            case Step::STOP:          addMotorStop(actions); break;
            case Step::BALL_LOADING:  addBallLoading(actions, s.value); break;
            case Step::RESET:          break;
        }
    }
    return actions;
}

// First, load balls and score in the long goal. Then return to load the next 5 balls,
// score them in the long goal, pick up the last two red balls, go to the middle goal,
// and finally return to open the sticks and push balls into the long goal for bonuses.

static vector<Step> R0_R = {
    {Step::STRAIGHT, -2313.16},
    {Step::TURN_LEFT, 90},
    {Step::BACKWARD, 1008.5},
    {Step::TOP_GOAL, 0},
    {Step::STRAIGHT, -1425.64},
    // {Step::BALL_LOADING, 0},
    // {Step::STRAIGHT, -552.14},
    // {Step::BALL_LOADING, 5},
    {Step::STOP, 0},
};

static map<Key, Routine> routineId() {
    map<Key, Routine> my_routine;
    my_routine[{RoutineID::R0, RIGHT}] = buildRoutine(R0_R);
    return my_routine;
}

static void runRoutine(const map<Key, Routine>& all, RoutineID id, Side s, AutoDrive& d) {
    Key k{id, s};
    auto it = all.find(k);
    if (it == all.end()) return;

    auto [i, e1, e2] = getInitialDataForOdometry();
    Odometry odom(i, e1, e2);

    wait(2, sec);
    for (auto& step : it->second) {
        step(d, odom);
        wait(20, msec);
    }
}

#endif
