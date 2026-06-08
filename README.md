# What is PID ?
 A PID controller is an instrument that receives input data from sensors, calculates the difference between the actual value and the desired setpoint, and adjusts outputs to control variables such as temperature, flow rate, speed, pressure, and voltage. It does this through three mechanisms: proportional control, which reacts to current error; integral control, which addresses accumulated past errors; and derivative control, which predicts future errors. The PID controller sums those three components to compute the output.

 
 ![images.png](https://github.com/salma-ahmed11/PID-task/blob/38f72fe9671732e31aa8c0888de5d37a88408bef/images/images.png)
 
 # Why do we use PID?
 PID is a closed loop system which provides you feedback every output command, this helps you to achieve accuracy and make sure that everything is going like what you've planned.

 # What's Proportional control?
 It's a type of linear feedback system where a machine's corrective action is scaled directly to the size of the error(the difference between the set point and the actual state) by multiplying it to a constant kp.

 
 $$Output = kp * error $$
 
 (you can use a voltage regulator in this case) however, this value might exceed the set point, causing an overshoot. After that, the system will recalculate all this again and it's possible that another overshoot may occur, and it might even continue oscillating.

 At this point, the role of **derivative control** comes into play.

 # What's the role of Derivative control?

 Its primary role is to predict future error by analyzing the rate at which the process variable is changing and multiplting it by constant kd, thereby applying a dampening effect that minimizes overshoot and stabilizes the system.

 $$Output = (error - pervious.error) / (time.difference)$$

 # What's the role of Integral control?
 It's primary role is to eliminate steady-state error. By accumulating the error over time and multiplying it by constant ki, it forces the syst10em's output to exactly match the desired target, compensating for persistent offsets and external disturbances. 
                       $$Output = integral + error * dt $$



 ## PID Controller Formula

The controller calculates the control output $u(t)$ based on the error $e(t)$, which is the difference between the setpoint and the measured speed:

![WhatsApp Image 2026-06-09](https://github.com/salma-ahmed11/PID-task/blob/83fa5413ff7362d803e2cc5e0336d3cc53a269b6/images/WhatsApp%20Image%202026-06-09%20at%2012.31.38%20AM.jpeg)

### Where:
* **$K_p$ (Proportional Gain):** Corrects the error based on its current value.
* **$K_i$ (Integral Gain):** Corrects the error based on the accumulation of past errors.
* **$K_d$ (Derivative Gain):** Predicts future error based on its current rate of change.






