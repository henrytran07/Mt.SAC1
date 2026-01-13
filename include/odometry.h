#ifndef ODOMETRY 
#define ODOMETRY    

#include "robot_config.h"
#include "vex.h"

#include <stack> 
#include <utility>
#include <cmath>

using namespace vex; 
using namespace std; 

class Odometry {
    private: 
        double last_angle_rotational_1 = 0; 
        double last_angle_rotational_2 = 0; 
        double last_angle_inertial = 0; 
        double sl = 0.0635; // 2 inches
        double sr = 0.0635; // 2inches
        double ss = 0.0254; // 1 inch
        double wheel_diameter = 0.1016; // 4inches 

        stack<pair<double, double>> positions; 

        double deg_to_rad(double deg) {
            return deg * M_PI / 180; 
        }

    public: 
        Odometry(double inertial, double rotational_1, double rotational_2) 
            :  last_angle_inertial{inertial}, last_angle_rotational_1{rotational_1}, last_angle_rotational_2{rotational_2}{
                positions.push({0, 0});
        }

        void update() {
            double angle_rot_1 = rotVert.position(degrees);  
            double angle_rot_2 = rotHor.position(degrees);   
            double angle_inertial = inertial_sen.rotation(degrees);

            double delta_rot_1 = angle_rot_1 - last_angle_rotational_1;
            double delta_rot_2 = angle_rot_2 - last_angle_rotational_2;
            double delta_angle_inertial = angle_inertial - last_angle_inertial;

            last_angle_rotational_1 = angle_rot_1;
            last_angle_rotational_2 = angle_rot_2;
            last_angle_inertial = angle_inertial;

            double wheel_radius = wheel_diameter / 2.0;

            double delta_forward = deg_to_rad(delta_rot_1) * wheel_radius; 
            double delta_side    = deg_to_rad(delta_rot_2) * wheel_radius; 

            double delta_theta = deg_to_rad(delta_angle_inertial);

            double local_dx = 0, local_dy = 0;
            if (fabs(delta_theta) < 1e-9) {
                local_dy = delta_forward;
                local_dx = delta_side;
            } else {
                local_dx = 2 * sin(delta_theta / 2) * (delta_side / delta_theta + ss);
                local_dy = 2 * sin(delta_theta / 2) * (delta_forward / delta_theta + sr);
            }

            double heading_mid = deg_to_rad(angle_inertial) - delta_theta / 2.0;

            double global_dx = local_dx * cos(heading_mid) - local_dy * sin(heading_mid);
            double global_dy = local_dx * sin(heading_mid) + local_dy * cos(heading_mid);

            const auto& last_pos = positions.top();
            positions.push({ last_pos.first + global_dx, last_pos.second + global_dy });
        }


        double getX() const{
            return positions.top().first; 
        }

        double getY() const{
            return positions.top().second; 
        }

        double get_headings() const {
            return inertial_sen.rotation(degrees);
        }
};
#endif 