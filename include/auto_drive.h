#ifndef AUTO_DRIVE
#define AUTO_DRIVE

#include "vex.h"
#include "robot_config.h"
#include "pid_control.h"
#include <cmath>

using namespace vex;

class AutoDrive {
private:
    static double clamp(double x, double lo, double hi) {
        if (x > hi) return hi;
        if (x < lo) return lo;
        return x;
    }

public:
    AutoDrive() { wait(0.25, msec); }

    double goStraight(double target = 0, double percent_pct = 30) {
        LeftGroup.setVelocity(percent_pct, percent);
        RightGroup.setVelocity(percent_pct, percent);

        double start = rotVert.position(degrees);
        double target_position = start - target;

        while (rotVert.position(degrees) - target_position < 0) {
            LeftGroup.spin(vex::forward);
            RightGroup.spin(vex::reverse);
            wait(10, msec);
        }

        LeftGroup.stop(brake);
        RightGroup.stop(brake);
        return target_position;
    }

    double goBackward(double targetDeg = 0, double speedPct = 10) {
        speedPct = fabs(speedPct);
        targetDeg = fabs(targetDeg);

        LeftGroup.setVelocity(speedPct, percent);
        RightGroup.setVelocity(speedPct, percent);

        double start = rotVert.position(degrees);
        double target_position = start - targetDeg;

        while (rotVert.position(degrees) > target_position) {
            LeftGroup.spin(vex::reverse);
            RightGroup.spin(vex::forward);
            wait(10, msec);
        }

        LeftGroup.stop(brake);
        RightGroup.stop(brake);
        return start;
    }

    double turnRight(double deg = 90, int drive_pct = 5) {
        double start = inertial_sen.rotation(degrees);
        double goal  = start + deg;

        RightGroup.setVelocity(drive_pct, percent);
        LeftGroup.setVelocity(drive_pct, percent);

        while (inertial_sen.rotation(degrees) < goal) {
            LeftGroup.spin(vex::forward);
            RightGroup.spin(vex::forward);
            wait(10, msec);
        }

        LeftGroup.stop(vex::brake);
        RightGroup.stop(vex::brake);
        return start;
    }

    double turnLeft(double deg = 90, int drive_pct = 5) {
        double start = inertial_sen.rotation(degrees);
        double goal  = start - deg;   

        RightGroup.setVelocity(drive_pct, percent);
        LeftGroup.setVelocity(drive_pct, percent);

        while (inertial_sen.rotation(degrees) > goal) {
            RightGroup.spin(vex::reverse);
            LeftGroup.spin(vex::reverse);
            wait(10, msec);
        }

        LeftGroup.stop(vex::brake);
        RightGroup.stop(vex::brake);
        return start;
    }

    void driveToEncoderPID(double targetPosDeg,
                           double kP = 0.33, double kI = 0.0, double kD = 0.08,
                           double maxPct = 30,
                           double settleErrDeg = 2.0,
                           int timeoutMs = 2500) {
        timer t;
        t.reset();

        PID_Control pid(kP, kI, kD, targetPosDeg, t.time(msec));

        while (t.time(msec) < timeoutMs) {
            double now = t.time(msec);
            double cur = rotVert.position(degrees);
            double err = targetPosDeg - cur;

            if (fabs(err) <= settleErrDeg) break;

            double out = pid.compute_pid(cur, now);
            out = clamp(out, -maxPct, maxPct);

            if (out >= 0) {
                LeftGroup.spin(vex::forward,  out, percent);
                RightGroup.spin(vex::reverse, out, percent);
            } else {
                LeftGroup.spin(vex::reverse,  -out, percent);
                RightGroup.spin(vex::forward, -out, percent);
            }

            wait(10, msec);
        }

        LeftGroup.stop(brake);
        RightGroup.stop(brake);
    }

    void goStraightPID(double deltaDeg,
                       double kP = 0.33, double kI = 0.0, double kD = 0.0,
                       double maxPct = 30,
                       double settleErrDeg = 30.0,
                       int timeoutMs = 2500) {
        double start = rotVert.position(degrees);
        double target = start - deltaDeg; 
        driveToEncoderPID(target, kP, kI, kD, maxPct, settleErrDeg, timeoutMs);
    }

    void goBackwardPID(double deltaDeg,
                       double kP = 0.33, double kI = 0.0, double kD = 0.08,
                       double maxPct = 25,
                       double settleErrDeg = 30.0,
                       int timeoutMs = 2500) {
        double start = rotVert.position(degrees);
        double target = start - fabs(deltaDeg); 
        driveToEncoderPID(target, kP, kI, kD, maxPct, settleErrDeg, timeoutMs);
    }

    void turnToHeadingPID(double goalDeg,
                          double kP = 0.33, double kI = 0.0, double kD = 0.0,
                          double maxPct = 20,
                          double settleErrDeg = 0.05,
                          int timeoutMs = 2500) {
        timer t;
        t.reset();

        PID_Control pid(kP, kI, kD, goalDeg, t.time(msec));

        while (t.time(msec) < timeoutMs) {
            double now = t.time(msec);
            double cur = inertial_sen.rotation(degrees);
            double err = goalDeg - cur;

            if (fabs(err) <= settleErrDeg) break;

            double out = pid.compute_pid(cur, now);
            out = clamp(out, -maxPct, maxPct);

            if (out >= 0) {
                LeftGroup.spin(vex::forward,  out, percent);
                RightGroup.spin(vex::forward, out, percent);
            } else {
                LeftGroup.spin(vex::reverse, -out, percent);
                RightGroup.spin(vex::reverse, -out, percent);
            }

            wait(10, msec);
        }

        LeftGroup.stop(brake);
        RightGroup.stop(brake);
    }
};

#endif
