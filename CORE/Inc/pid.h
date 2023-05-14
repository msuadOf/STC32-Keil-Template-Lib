#ifndef _PID_H
#define _PID_H

typedef struct
{
  float A0;          /**< The derived gain, A0 = Kp + Ki + Kd . */
  float A1;          /**< The derived gain, A1 = -Kp - 2Kd. */
  float A2;          /**< The derived gain, A2 = Kd . */
  volatile float state[3];    /**< The state array of length 3. */
  float Kp;          /**< The proportional gain. */
  float Ki;          /**< The integral gain. */
  float Kd;          /**< The derivative gain. */
	float in_last;
	float sum;
} pid_t;


float arm_pid_f(pid_t * S,float in);
void arm_pid_init_f(pid_t * S);
float pid_process(pid_t *pid,float current, float aim, float max_thereshold, float min_thereshold);

#endif