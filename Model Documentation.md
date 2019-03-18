# Path Planning Writeup

**Author:** Evan Loshin

**Final Project Video:** https://youtu.be/5iiIuRBAC_s

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

---

### Model Architecture
The path planning code in `main.cpp` uses the finite state machine (FSM) concept for decision-making and spline fitting for trajectory generation. Additional code implements collision avoidance and jerk minimization to comply with safety and comfort requirements.

The portions of code resembling a FSM help to organize lane change planning. At each timestep, sensor fusion data is evaluated to transition between states *Keep Lane* (KL), *Prepare Lane Change Left/Right* (PLCL/PLCR), and *Lane Change Left/Right* (LCL/LCR). Line 109 defaults the state to KL at the beginning of each cycle. If the car is behind a slower moving vehicle, code for PLCL/PLCR states is invoked to evaluate the feasibility of changing lanes. If safe, LCL/LCR code executes the lane change. Additional "stay right except to pass" logic biases the car to move right when doing so would not require decelerating. Reevaluating the state at every iteration seemed like the easiest way to ensure decision-making stayed current and efficient given the dynamic environment.

My code relies on the cubic spline interpolation library contained in `spline.h` to generate trajectories. Each iteration fits a spline to waypoints taken from the final two point of the vehicle's previous trajectory as well as three new points at 30m increments along the target path. Spacing the values in between waypoints is determined by evenly distributing linear distance for the given reference velocity.

### Additional Considerations
While this project successfully navigates the car around the track, shortcomings exist that don't materialize in simulation. For instance, collision avoidance needs further consideration for situations requiring significant braking such as sudden heavy traffic. While this code uses a static acceleration parameter to meet the zero jerk requirement, real-life circumstances necessitate a cost function that dynamically calculates deceleration, allowing uncomfortable jerk in some cases.

An additional feature to enhance this project is consideration for multiple lane changes. In some cases, the vehicle gets stuck behind slow moving vehicles in the current lane as well as one lane over. When a farther lane is empty, the vehicle could consider multiple lane changes to reach the target.
