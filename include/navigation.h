#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "vex.h"
#include "pushback_map.h"
#include "auto_drive.h"
#include "pid_control.h"
#include "polynomial_trajectory_generation.h"
#include "odometry.h"

#include <algorithm>
#include <utility>
#include <unordered_map>

using namespace vex;
using namespace std;

class Navigator {
    private:
        double robotX; 
        double robotY; 

        AutoDrive &drive;      
        PushBackMap map; 
        Poly_Trajectory trajectory; 
        float wheel_diameter = 0.0762; 
        float wheel_base = 0.3175; 
        float max_speed = 7.98;
    public:
        Navigator(PushBackMap &m, AutoDrive &d, startingPos pos = POSITION_C)
            :drive(d), map{m} {
            bot_dimension bot; 
            switch(pos) {
                case POSITION_A: {
                    positionA a;
                    robotX = a.x;
                    robotY = a.y;
                    break;
                }
                case POSITION_B: {
                    positionB b;
                    robotX = b.x;
                    robotY = b.y;
                    break;
                }
                case POSITION_C: {
                    positionC c;
                    robotX = c.x;
                    robotY = c.y;
                    break;
                }
                case POSITION_D: {
                    positionD d_pos;
                    robotX = d_pos.x;
                    robotY = d_pos.y;
                    break;
                }
            }
        }

        double getX() const { return robotX; }
        double getY() const { return robotY; }

        void updatePosition(float newX, float newY) {
            robotX = newX;
            robotY = newY;
        }

        double degToRad(double deg) {
            return deg * M_PI / 180; 
        }
        
        void driveToCoordinate(float targetX, float targetY) {
            float dx = targetX - robotX; 
            float dy = targetY - robotY; 

            vex::timer timer; 
            timer.clear(); 
            Poly_Trajectory trajectory(0, dx, 0, dy); 
            double duration = trajectory.get_duration();

            double initial_inertial_angle = inertial_sensor.rotation(degrees);
            double encoder_angle = encoder1.position(degrees);
            double rotational_angle = rotational_sensor1.position(degrees);

            Odometry odometry(initial_inertial_angle, rotational_angle, encoder_angle);

            while (timer.time() / 1000 < duration) {
                double t = timer.time() / 1000; 

                // planned path (using polynomial trajectory)
                double poly_x = trajectory.get_x(t);
                double poly_y = trajectory.get_y(t);

                // planned v (using polynomial trajectory)
                double v = trajectory.chassis_speed(t);

                odometry.update();

                // real path (using odometry)
                double real_x = odometry.getX();
                double real_y = odometry.getY(); 

                PID_Control pid_x(0.95, 0, 0, poly_x);
                PID_Control pid_y(0.95, 0, 0, poly_y);

                double ex = pid_x.compute_pid(real_x, t);
                double ey = pid_y.compute_pid(real_y, t);
                
                double headings = degToRad(odometry.get_headings());

                double e_forward = cos(headings) * ex + sin(headings) * ey; 
                double e_turn = -sin(headings) * ex  + cos(headings) * ey; 

                double e_angle = atan2(ey, ex);
                double e_theta = (e_angle - headings);

                double omega = trajectory.get_omega(t);
                double align; 

                if (cos(e_theta) < 0) align = 0; 
                else align = cos(e_theta);

                double v_linear = v + 0.3 * e_forward * align; 
                double w = omega + e_theta;

                double v_left = v_linear - wheel_base / 2 * w;
                double v_right = v_linear + wheel_base / 2 * w; 

                LeftGroup.setVelocity(v_left / max_speed * 100, percent);
                RightGroup.setVelocity(v_right / max_speed * 100, percent);

                LeftGroup.spin(vex::forward);
                RightGroup.spin(vex::reverse);

                wait(10, msec);
            }


            updatePosition(robotX + odometry.getX(), robotY + odometry.getY());

            LeftGroup.stop(brakeType::brake);
            RightGroup.stop(brakeType::brake);
        }


        void driveToFinalDestination(CellType type, bool right = true) {
            float x = 0, y = 0; 
            bool found = false; 
            for (auto &row : map.grid) {
                for (auto &cell : row) {
                    if (cell.type == type) {
                        if (right) {
                            if (cell.x > 0 && cell.y >= 0) {x = cell.x; y = cell.y; found = true; break;}
                        } else {
                            if (cell.x < 0 && cell.y >= 0) {x = cell.x; y = cell.y; found = true; break;}
                        }
                    }
                }
                if (found) break; 
            }

            if (found)
                driveToCoordinate(x, y);
        }

};

#endif
