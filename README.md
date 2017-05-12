# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Reflection

### Describe the effect each of the P, I, D components had in your implementation.

`P` component affects how hard the car will steer depending on the CTE. It can be increased if car seem to understeer in the sharp turns. But it also increases chances of car wondering within the lane. For the throttle PID controller, `P` parameter determined how fast car should go depending on it's distance from the edge of the road.

`I` component controls systematic understeer. If car is moving to a certain side more then the other, this parameter should correct it. For the throttle PID controller, `I` parameter controls systematic issues of going below or above target speed.

`D` component controls how much to reduce the steering to the center of the lane when we are approaching the center of the lane. This helps to avoid oscillations around the center of the lane. For throttle PID controller `D` component determined how should throttle change depending on the rate of change of distance to the edge of the road.

### Describe how the final hyper-parameters were chosen.

Initially I've tried to choose hyper-parameters manually. For that I looked at understeering, oscillations around the center of the lane and systematic shift. That brought me to the `P = 0.1, I = 0., D = 10.` set. This also allowed me to ride at about 50 miles per hour with constant throttle of `0.45`.

Then I've implemented the twiddle algorithm with a goal to minimize the sum of squares of `CTE`. That brought me following set: `P = 2.03741, I = 0., D = 29.2802`. This set performed less good then the parameters I've chosen manually. While sum of squared errors was smaller with this set of parameters, if I increased the throttle at some points of the track car deviated too much from the center and go off the track.

To mitigate excessive deviation from the center at certain point, I've increased the penalty for such deviation using sum of `CTE` to the power of 8 instead of 2 for the error measure to optimize. That helped, and the car was able to go with higher speeds.

To further improve the parameters, I've altered twiddle algorithm to increase throttle by factor of `1.1` with every successful loop. I've defined loop as successful if there was no excessive deviation from the center of the lane during that loop. That algorithm have reached constant throttle of `0.644204` and have not been able to converge since. Hyper-parameters that worked for lower throttle have been lost.

Next I’ve tried to optimize speed at each section of the track. I’ve started by defining a target speed - value depending on the current CTE. If car is close to the edge of the road, it should slow down to not fall over. When car is below the target speed, it did full acceleration. If car is above target speed - full break. That helped quite a bit, but my targets could not be met initially because there was not enough time to break before the tight turn. Also, when departing from the edge of the road toards the center of the road, car was going slower then it could safely go.

Next I’ve implemented PID controller for throttle. Input was the speed error - difference between target speed and current actual speed. The output was a throttle value. With PID controller car started slowing down as it approached the barrier, and accelerating when it departed from the barrier, which is more optimal behavior. I've manually selected `P = .1, I = 0., D = 1.` parameters for throttle PID controller.

Later I was trying to make a video with my progress. Screen recording tool degraded performance of my system, and that resulted in car hitting the barriers. If we don’t process enough messages per second, time to make course correction is lost and car goes over the edge. So I made a change allowing to measure time since last message, and use that time to reduce target speed further from the edge of the road. This reduces overall speed on track greatly, but makes successful completion of the track more likely.

If I had more time, I would make the distance of travel the success measure for the twiddle algorithm. I can determine distance by calculating delta of time from pervious to current step, and multiplying that by the speed. Sum of these products will give the distance. Algorithm stops if car leaves the track. Given this measure, I would expect twiddle to arrive at better hyper-parameters.

[Video](https://youtu.be/Wew07ZP1Ros) showing final result is available.

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.13, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.13 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.13.0.zip).
  * If you run OSX and have homebrew installed you can just run the ./install-mac.sh script to install this
* Simulator. You can download these from the [project intro page](https://github.com/udacity/CarND-PID-Control-Project/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
