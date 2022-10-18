/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PID {
public:

    /*
    * Errors
    */
    double proportional_error;
    double intergrator_error;
    double differential_error;
    double total_err;
    double best_err;
    /*
    * Coefficients
    */
    double proportional_param;
    double integral_param;
    double differential_param;
    double k_param[3];
    /*
    * Output limits
    */
    double output_lim_max;
    double output_lim_min;
    /*
    * Delta time
    */
    double delta_time;

    /*supporting variables*/
    int step_num;
    int p_index;
    bool p_add;
    bool p_sub;
    bool twiddler;
    /*
    * Constructor
    */
    PID();

    /*
    * Destructor.
    */
    virtual ~PID();

    /*
    * Initialize PID.
    */
    void Init(double Kp, double Ki, double Kd, double output_lim_max, double output_lim_min, bool t);

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte);

    /*
    * Calculate the total PID error.
    */
    double TotalError();
  
    /*
    * Update the delta time.
    */
    double UpdateDeltaTime(double new_delta_time);

    void updateparam(int index, double value);
};

#endif //PID_CONTROLLER_H


