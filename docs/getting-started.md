# Getting Started with Highway Driving Path Planner

## David and Aaron Answer Questions

**Give a sort of walkthrough in order to face the project. It seems the lesson and project lacks connection. Even in the lesson import parts are overlooked & concret examples are missing. -Sebastiano, Jason, Nadia, etc**:

Get the car moving

    - use trajectory generation

Get the car to drive in its lane

    - use frenet coordinates, smoothing the path with spline

Get the car to swtich between lanes

    - prereq: get the car to drive in its lane
    - think about behavior planning and what state do you want to enter, change lanes left, change lanes right, keep your current lane.
    - you enter a world where you got your car in a lane and a few other lanes and you're looking at how do you switch between lanes
    - preferred way to swtich lanes is to build a cost function. In the walkthrough, they had the car check if it saw a car ahead of it, veer to the left. The cost function could take into account what the cost of being in each lane is and then if you work backwarrds all way to the prediction lesson, you can try to predict out into the future where all these cars are going to be at different points in the future and what's your cost for being in different states and different lanes in the future

Then predicting out into the future and trying to figure out where other vehicles are going to be would say a Gaussian Naive Bayes classifier or a different type of classifier and using that to decide where you want to be in the future

With an actual path planner, the way the data flows in is you get the data, you try and predict where things will be in the future, then you try to make your own decisions in behavior planning and then ultimately you generate a trajectory.

For this project you might almost want to work backwards and start with building a trajectory and assuming no other vehicles are there and then start to assume other vehicles are there and build a behavior plan but don't worry about the future and then worry about the future and start predicting out where other vehicles are going to be in the future and thus where do you want to go.

We were just dealing with the trajectory for the time being. But, a cost function would be great for dealing with how to change lanes and the quiz is a really good place to really get started with that. Frenet is really helpful for being able to look at all the other cars on the road and seeing where their s and d is at. Frenet gets a little tricky when we're doing just pure trajectory generation that's why we had that spline come into play as well.


**Review implementation of spline! - Tim**:

A spline is a piecewise polynomial function thats where the pieces are tied together at those anchor points or sometimes they're called Knots and some math is done in tying together those pieces so that the connections with the knot are smooth which is why the spline trajectory looks smooth. From the standpoint of this lesson, you don't need to know how to implement it. We're really just using it since the code was written in, was there. It was really easy to use. So all you really need to know is how to use it. Walkthrough the different parts of using it, there are only like 3 commands to run when using it. You just initialize it. Let's reference the code from the walkthrough video to see how we use it:

~~~cpp
// include spline header
#include "spline.h"

// Create a list of widely spaced (x, y) waypoints, evenly spaced at 30m
// Later we will integpolate these waypoints with a spline and fill in with more points that control speed
vector<double> ptsx;
vector<double> ptsy;

// code in between
...
// code in between

// create a spline
tk::spline s;

// set the anchor points to the knots around which the spline gets formed
s.set_points(ptsx, ptsy)

// s and some x value
// so you give spline an x value, you get back the corresponding y value

// our y, ask spline what's the y for that given x
double target_y = s(target_x);
~~~

One thing to be careful about with splines and anythin else about a quintic polynomial is that if the spline starts to go vertical you can wind up with multiple x values or multiple y values for the same x value and that starts to break things. That's why in the walkthrough they did that basis - double polymerization transformation. So you're shifting horizontally. Makes the math easy. So that way if you shift the spline so it runs horizontally instead of vertically, then there won't be multiple y values for an x and once you pull out the waypoints you need from the spline, you can shift those waypoints back. Everything will just work.

**Can you talk about some strategies for smoothing the trajectories between calculations? - Doug, Juan, etc**:

I'm able to generate a smooth trajectory by creating a spline based on waypoints around me. However, my jerk often exceeds the threshold when I create a new trajectory in the next calculation.

Aaron explains:

The previous path points is a great way and that really helps you avoid any discontinuities. You've got to be careful when you're purely creating a new path every single frame. And having those previous path points really helps give you an anchor or reference to work from. And I found that when I was having problems with my jerk that I definitely need to rely more on the previous path points.

**One technique** for smoothing is the `previous path points` or using the previous path points so there is more continuity between the new path and the old path. There's using the spline itself and picking out waypoints along the spline because those are guaranteed to be smooth. 

**Another technique** you could do `quintic polynomial generation` like what is covered in the trajectory generation lesson that involves a little bit more math.

**Another technique** use the spline and instead of doing this kind of trigonometry that Aaron does with DNN and in projecting from the spline onto a triangle. Once I pulled off the 50 waypoints from the spline, I went through and verified that none of them were too far apart to create a velocity that was too high and if they were, then I pulled them back in. Might be less efficient than the math Aaron did, but could be easier to map head around.

**When I used the Live Help feature, one mentor said that I should convert the waypoints map coordinates to vehicle coordinates and use this for all my calculations. Do you think this is a good approach? - Doug**

Doug couldn't figure out how to do that transformation.

Aaron thinks that is an awesome approach. If you're talking about local transformation, that is exactly what we did in our walkthrough.

Transform between vehicle coordinates to waypoints map coordinates:

Heres the math to do the shift and then rotation

~~~cpp
for(int i = 0; i < ptsx.size(); ++i)
{
    // shift car reference angle to 0 degrees
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = (shift_x * cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
    ptsy[i] = (shift_x * sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
}
~~~

Then the rotation and shift.

~~~cpp

for(int i = 1; i <= 50-previous_path_x.size(); ++i)
{
    ...

    // rotate back to normal after rotating it earlier
    // we do the inverse of what we were doing before
    // we do a shift here, then rotation
    x_point = (x_ref * cos(ref_yaw)-y_ref * sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw)+y_ref * cos(ref_yaw));

    ...
}

~~~

This transformation was helpful. We're deployiing that in MPC and that made the math alot easier. It also makes sure you don't get any functions near vertical and that could just kind of explode on you. For what aaron was trying to do to fit points to a spline that made it alot easier since then he is just working with the x axis. I don't have to try to create some separate line going at some angle and then work from that.

**How does the Finite State Machine work with Hybrid A* in this project? -Hsin-Chen**

Does hybrid A* make sense to use for this project?

No, hyrbid A* makes sense more so like in a parking lot environment. But on this kind of highway, a cost function makes more sense. The thing about the highway is it is a very sparse environment, so there are a ton of different maneuvers you could make and you just want to choose the lowest cost maneuver whereas hybrid A* tends to work better in denser environments where you have a more limited number of choices you could make. And also discretized choices. And this is more of a continuous environment. You could like map a grid to the highway and use hybridized A* and maybe it would work. But using a cost function or finite state machine seems to be a better choice here.

In general, david is not sure you would generally use a finite state machine and hybrid A* together. Maybe there's an interesting way to do it like in a research paper.