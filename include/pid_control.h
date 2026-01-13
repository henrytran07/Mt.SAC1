#ifndef PID_CONTROL 
#define PID_CONTROL 

#include "vex.h"
#include <vector>
using namespace vex;
using namespace std; 

class PID_Control {
    private:
        double k_p;             
        double k_i;             
        double k_d;              
        double desired;          
        double previous_error;   
        double last_time;        
        vector<double> errors;   

    public:
        PID_Control(double k_p, double k_i, double k_d, double desired, double last_time = 0) {
            this->k_p = k_p;
            this->k_i = k_i;
            this->k_d = k_d;
            this->desired = desired;
            this->previous_error = 0;  
            this->last_time = last_time;       
        }

        double compute_proportional_term(double current) {
            double error = desired - current; 
            double P = k_p * error;           
            errors.push_back(error);   
            // Brain.Screen.print("Error: %.2f", error);       
            return P;
        }

        double compute_integral_term() {
            double integral_sum = 0.0;
            for (auto err : errors)
                integral_sum += err;
            double I = k_i * integral_sum;    
            return I;
        }

        double compute_derivative_term(double current, double current_time) {
            if (fabs(current_time - last_time) < 1) return 0; 
            double current_error = desired - current;  
            double error_diff = current_error - previous_error;  
            double delta_t = current_time - last_time;  

            double derivative_term = 0;
            if (delta_t > 0) {
                derivative_term = k_d * error_diff / delta_t;  
            }

            previous_error = current_error;  
            last_time = current_time;       

            return derivative_term;
        }

        double compute_pid(double current, double current_time) {
            double P = compute_proportional_term(current);
            double I = compute_integral_term();
            double D = compute_derivative_term(current, current_time);
            return P + I + D;  
        }
};

#endif 
