#include "pid.h"

void arm_pid_init_f(pid_t * S)
{
  /* Derived coefficient A0 */
  S->A0 = S->Kp + S->Ki + S->Kd;
  /* Derived coefficient A1 */
  S->A1 = (-S->Kp) - ((float) 2.0f * S->Kd);
  /* Derived coefficient A2 */
  S->A2 = S->Kd;

}

float arm_pid_f(pid_t * S,float in)
{
 float out;

//  /* y[n] = y[n-1] + A0 * x[n] + A1 * x[n-1] + A2 * x[n-2]  */
  out = (S->A0 * in) + (S->A1 * S->state[0]) + (S->A2 * S->state[1]) + (S->state[2]);

  /* Update state */
  S->state[0] = in;
  S->state[1] = S->state[0];
  S->state[2] = out;

  /* return to application */
  return (out);
}
float pid_process(pid_t *pid,float current, float aim, float max_thereshold, float min_thereshold)
{
    float act_out;
    arm_pid_init_f(pid);
    act_out=arm_pid_f(pid,current-aim);
    if(act_out<min_thereshold) 
        act_out=min_thereshold;
    if(act_out>max_thereshold) 
        act_out=max_thereshold;
    return act_out;
}