#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
     this->Kp = Kp;
     this->Ki = Ki;
     this->Kd = Kd;
     this->cte_old = 0;
     this->error_sum = 0;
}

void PID::UpdateError(double cte) {
  p_error = cte;
  i_error = cte + error_sum;
  d_error = (cte - cte_old);
  cte_old = cte;
  error_sum += cte;

}

double PID::TotalError() {
  i_error;
}

