#ifndef POLYNOMIAL_TRAJECTORY_GENERATION 
#define POLYNOMIAL_TRAJECTORY_GENERATION

#include "vex.h"
#include "pushback_map.h"

#include <cmath> 
#include <utility>
#include <array>
#include <utility>

using namespace vex; 
using namespace std; 

class QuinticMotionProfile {
    private: 
        double t0, s0, v0, a0; 
        double t1, s1, v1, a1; 
    public: 
        QuinticMotionProfile(double t0, double s0, double v0, double a0, double t1, double s1, double v1, double a1)
            : t0{t0}, s0{s0}, v0{v0}, a0{a0}, 
              t1{t1}, s1{s1}, v1{v1}, a1{a1} {}
        
        vector<double> quintic_coefficients() const {
            const double T = t1 - t0; 
            const double A0 = s0; 
            const double A1 = v0; 
            const double A2 = 0.5 * a0; 

            const double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T; 
            const double c0 = s1 - (A0 + A1*T + A2*T2);
            const double c1 = v1 - (A1 + 2.0*A2*T);
            const double c2 = a1 - (2.0*A2);

            const double A3 = ( 10.0*c0 - 4.0*c1*T - 0.5*c2*T2) / T3;
            const double A4 = (-15.0*c0 + 7.0*c1*T +      c2*T2) / T4;
            const double A5 = (  6.0*c0 - 3.0*c1*T - 0.5*c2*T2) / T5;

            return { A0, A1, A2, A3, A4, A5 };
        }
};

class Poly_Trajectory {
    private: 
        const double ratio_gear = 0.3; 
        const double wheel_diameter = 0.254; 
        const double motor_rps = 10; 
        const double v_max = ratio_gear * motor_rps * M_PI * wheel_diameter; 
        const double wheel_base = 0.3302; 

        const double x0, x1, y0, y1;   
        double T; 
        double A0, A1, A2, A3, A4, A5; 
        double B0, B1, B2, B3, B4, B5; 

        double  _x_prime(double t) const {
            double t2 = t*t; 
            double t3 = t2 * t; 
            double t4 = t3 * t; 
            return A1 + 2 * A2 * t + 3 * A3 * t2 + 4 * A4 * t3 + 5 * A5 * t4;
        }

        double _y_prime(double t) const {
            double t2 = t*t; 
            double t3 = t2 * t; 
            double t4 = t3 * t; 
            return B1 + 2*B2*t + 3*B3*t2 + 4*B4*t3 + 5*B5*t4;
        }

        double _x_double_prime(double t) const {
            double t2 = t*t; 
            double t3 = t2 * t; 
            return 2 * A2 + 6 * A3 * t + 12 * A4 * t2 + 20 * A5 * t3; 
        }
        
        double _y_double_prime(double t) const {
            double t2 = t*t; 
            double t3 = t2 * t; 
            return 2 * B2 + 6 * B3 * t + 12 * B4 * t2 + 20 * B5 * t3; 
        }

        double _get_kappa(double t) const {
            const double vx = _x_prime(t),  vy = _y_prime(t);
            const double ax = _x_double_prime(t), ay = _y_double_prime(t);
            const double num = vx*ay - vy*ax;
            const double v2 = vx*vx + vy*vy;
            const double den = pow(v2, 1.5);
            return (den > 1e-12) ? (num / den) : 0.0;
        }

    public: 
        Poly_Trajectory(const double x0 = 0, const double x1 = 0, const double y0 = 0, double y1 = 0) 
            : x0{x0}, x1{x1}, y0{y0}, y1{y1} {
                double delta_x = x1 - x0; 
                double delta_y = y1 - y0; 
                double d = sqrt(delta_x * delta_x + delta_y * delta_y); 
                T = 15.0/8.0 * d / v_max; 
                QuinticMotionProfile qx = QuinticMotionProfile(0, x0, 0, 0, T, x1, 0, 0);
                QuinticMotionProfile qy = QuinticMotionProfile(0, y0, 0, 0, T, y1, 0, 0);

                auto A = qx.quintic_coefficients(); 
                A0 = A[0]; A1 = A[1]; A2 = A[2]; A3 = A[3]; A4 = A[4]; A5 = A[5];

                auto B = qy.quintic_coefficients();
                B0 = B[0]; B1 = B[1]; B2 = B[2]; B3 = B[3]; B4 = B[4]; B5 = B[5];
            }

        double get_duration() const {
            return T; 
        }

        pair<double, double> get_l_r_speed(double t) {
            double v = chassis_speed(t);
            double kappa = _get_kappa(t);

            double b = wheel_base / 2; 
            double v_left = v * (1 - kappa * b);
            double v_right = v * (1 + kappa * b);

            return {v_left, v_right};
        }  

        double get_x(double t) const {
            double t2= t * t; 
            double t3 = t2 * t; 
            double t4 = t3 * t; 
            double t5 = t4 * t;  
            return A0 + A1 * t + A2 * t2 + A3 * t3 + A4 * t4 + A5 * t5; 
        }  

        double get_y(double t) const {
            double t2= t * t; 
            double t3 = t2 * t; 
            double t4 = t3 * t; 
            double t5 = t4 * t;  
            return B0 + B1 * t + B2 * t2 + B3 * t3 + B4 * t4 + B5 * t5; 
        }  

        double get_omega(double t) const {
            double v = chassis_speed(t);
            double kappa = _get_kappa(t);
            return v * kappa; 
        }

        double chassis_speed(double t) const {
            double t2 = t*t; 
            double t3 = t2 * t; 
            double t4 = t3 * t; 

            double v_x =  _x_prime(t);
            double v_y = _y_prime(t);

            return sqrt(v_x * v_x + v_y * v_y);
        }

};

#endif
