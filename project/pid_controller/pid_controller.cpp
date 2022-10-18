/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>
#include <limits>

using namespace std;

PID::PID() {}

PID::~PID() {
   proportional_error =0;
   intergrator_error =0;
   differential_error =0;
   proportional_param=0;
   integral_param=0;
   differential_param=0;
   k_param[0] =0;
   k_param[1] =0;
   k_param[2] =0;
   output_lim_max=0;
   output_lim_min=0;
   delta_time=0;
   best_err = std::numeric_limits<double>::max(); //Initialize to high value
}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   proportional_param=Kpi;
   integral_param=Kii;
   differential_param=Kdi;
   output_lim_max=output_lim_maxi;
   output_lim_min=output_lim_mini;

   step_num = 1;
  
   //Store coefficients in array for twiddle
   k_param[0] = Kpi*0.1;
   k_param[1] = Kii*0.1;
   k_param[2] = Kdi*0.1;

   //Current index of twiddle coefficients
   p_index = 0;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
   if(delta_time<1e-8){
      differential_error =0;
   }
   else{
      differential_error =(cte - proportional_error)/delta_time;
   }
   intergrator_error += cte*delta_time;
   proportional_error = cte;

   total_err += cte*cte;

#if 0
  //Run the twiddle algorithm every 'n' evaluation steps
  if(step_num % 2 == 0) {
    //if the current error is a new best, update
    if(total_err < best_err) {
      best_err = total_err;
      k_param[p_index] *= 1.1;

      //Setup for the twiddler
      p_add = p_sub = false;
    }

    if(!p_add && !p_sub) {
      //First iteration after start of cycle, add elements
      updateparam(p_index, k_param[p_index]);
      p_add = true;
    } else if(p_add && !p_sub) {
      //Second iteration after cycle
      //No best error found,
      updateparam(p_index, -2*k_param[p_index]);
      p_sub = true;
    } else {
      //Third iteration
      //No best error found after two attempts, time to try something new
      updateparam(p_index, k_param[p_index]);
      k_param[p_index] *= 0.9;
      p_add = p_sub = false;

      //Cycle through the 3 hyperparameters
      p_index = (p_index + 1) % 3;
    }
    //Reset total error at end of cycle
    total_err = 0;

    std::cout << "Adjusted parameters ..." << "\n";
    std::cout << "Kp = " << proportional_param << " Ki = " << integral_param << " Kd = " << differential_param << "\n\n";
  }
  step_num++;
#endif
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control;

    control = proportional_param*proportional_error 
               + integral_param*intergrator_error 
               + differential_param*differential_error;

   //control greater than max value equate it to max
   if (control > output_lim_max){
      control = output_lim_max;
   }

   //control lesser that min value then equate to min
   if (control < output_lim_min){
      control = output_lim_min;
   }
   
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
  delta_time = new_delta_time;
  return delta_time;
}

//Twiddles variables based on index of hyperparameter and value determined above.
void PID::updateparam(int index, double value) {
  switch(index) {
    case 0:
      proportional_param += value;
      break;
    case 1:
      integral_param += value;
      break;
    case 2:
      differential_param += value;
      break;
    default:
      break;
  }
}