# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## How PID works

![alt text](./images/PIDfront.PNG "Logo Title Text 1")

The distinguishing feature of the PID controller is the ability to use the three control terms of proportional, integral and derivative influence on the controller output to apply accurate and optimal control. The block diagram on the right shows the principles of how these terms are generated and applied. It shows a PID controller which continuously calculates an error value e ( t )  as the difference between a desired setpoint SP= r ( t )  and a measured process variable PV= y ( t )  and applies a correction based on proportional, integral, and derivative terms. The controller attempts to minimize the error over time by adjustment of a control variable u ( t ),  such as the opening of a control valve, to a new value determined by a weighted sum of the control terms.

###  Proportional control P
```
P provides a correction factor proportional to the CTE. The problem is that as the CTE being corrected approaches zero, so too does the correction, with the result that the error never really goes down to zero. On the other hand, if the correction is too strong, the correction may overshot and create a contrary error:

High P correction
- More capable of adjusting the error
- Higher overshot
	
Low P correction
- Less sensitivity
- Less capable of adjusting the error
```
### Integral control I
```
I is a summation of the errors cumulated along time. Since some systematic bias may exist when applying corrections (for istance the steering is not executed precisely right), the summation may point out how the bias is contributing to the error generation and help annulating it by applying the correct correction: 

High I correction:
- More oscillatory
- Faster reduction of cumulated error
	
Low I correction
- Less oscillatory
- Slower reduction of cumulated error
```
### Derivative control D
```
D looks at the rate of change of the error, and adds an instantaneous value to the loop output that is proportional to the rate of change of the error. As an effect, it reduces the oscillatory behaviour and countereffects the overshot of the response due to P because it can anticipates how the error is changing by using its rate of change.

 High D correction:
- Overdamped
- It will take a long time to correct
	
Low D correction
- Underdamped
- Still somehow oscillatory
```
Each of these controls is weighted by a coefficient (namely, Kp, Ki, Kd). All the weighted controls summed result in the total error, which can be used to countereffect (by changing its sign) the CTE feedback. The success of the PID in controlling CTE and having the car in the simulator stay on track depends on the successful definition of the numeic values.

## Running the Program for manual tuning.

```
./pid Kp Ki Kd 
```
When ever the pid program starts new the simulator resets.

## Solution

My best result I got with following parameters:

```
Kp = 0.1
Ki = 0.005
Kd = 0.9
```
### Reflection

I started with a P Controller and tuned it that the car stays on the track as long as possible.
Than I added Derivative part and tuned it in a way that it stays on the track pretty well. It just hits the curves border at some parts.
After I added the  Integral the car stays on the track.

It also would be possible to tune the parameter automatically, for example with the [twittle algorythmus](https://martin-thoma.com/twiddle/)

By dynamically calculating the throttle values, a further improvement in driving behavior could be achieved.

|  | P                     | PD                | PID  |
|-----|---------------------------|---------------|------------|
||Kp = 0.1 Ki = 0 Kd = 0| Kp = 0.1 Ki = 0  Kd = 0.9 | Kp = 0.1 Ki = 0.005 Kd = 0.9 | 
|**Comment**| oscillates heavily in the road| can drive on the road, hits the read line in the curves|  drives  smoothly through the track a bit wobbly at low speed|
| **Video**| [![E](https://img.youtube.com/vi/51WgsbzxZmc/0.jpg)](https://youtu.be/51WgsbzxZmc "P")| [![E](https://img.youtube.com/vi/ohKL-oc1nIs/0.jpg)](https://youtu.be/ohKL-oc1nIs "PD") | [![E](https://img.youtube.com/vi/mp5im0Wfn1g/0.jpg)](https://youtu.be/mp5im0Wfn1g "PID")|

### 




