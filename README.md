# car-pid-control

The project is dedicated to the implementation of PID Control for Udacity Self-Driving Car Nanodegree.

The PID Control is implemented in class PID (see PID.h, PID.cpp). It consists of the following parts:
1. Initialize the parameters (Kp, Ki, Kd - proprtional, integral and differential coefficients accordingly).
  - Kp - proportional coefficient. It's responsible for producing the control action proportional to the error. Having only proportional part leads to overshooting - constant oscillations without convergence.
  - Kd - differential coefficient - responsible for making the convergence possible. It compensates the proportional part.
  - Ki - integral coefficient - responsible for making the vehicle to be prone to the bias, so the vehicle is able to properly react to unexpected error. This part brings slight oscillations.
2. Update the error (the cross-track error is used):
  - proportional error is taken as the exact value of the cross-track error (cte).
  - differential error is taken as the difference between the current cte and the previous one.
  - integral error is calculated as the sum of all the cte's including current one.
3. Calculate the total error which then will be used for the actual control. It's calculated as follows (please note, there's no minus sign as the total error will be inversed during the control action):
  - total_error = Kp*p_error + Kd*d_error + Ki*i_error

During the control part (see main.cpp), if the absolute value of total_error exceeds 1, it's being clipped to 1 as the steering value can be in range [-1, 1].

I was managed to come up with the following final values of the coefficients manually:
 - Kp = 0.2
 - Ki = 0.0029
 - Kd = 3.0
 
It turned out that the proportional and differential parts were already good enough, so I played with the integral part a bit which turned out to be the important part for making a smooth turns.

Please see a short video with the driving behavior produced: track_driving.MOV
 
As the starting values I've taken the values provided in the lectures which appeared to be very optimal:
 - Kp = 0.2
 - Ki = 0.004
 - Kd = 3.0
 
I also worked on the implementation of "twiddle". With "twiddle", if you don't pay attention to the speed limit, you may end up in a situation where the PID coefficients rise quickly, producing the large error, which after clipping turns into -1 or 1, so the car goes slowly over the needed trajectory with minimal possible error. However, due to these frequent steering actions, it goes with the speed around 1 mph :(
  

