#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
    i_error = 0;
    p_error = 0;
    d_error = 0;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}

void PID::UpdateError(double cte) {
    d_error = p_error != 0 ? cte-p_error : 0;
    p_error = cte;
    i_error += cte;
}

double PID::TotalError() {
    return Kp*p_error + Kd*d_error + Ki*i_error;
}

