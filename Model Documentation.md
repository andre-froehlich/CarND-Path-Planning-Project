# Model documentation

The model generation in this project consists of three main parts:
1. Unpacking sensor fusion data
2. Deciding on the strategy
3. Trajectory generation

## Unpacking sensor fusion data
In this section we go through all the other cars on the road.
* First we determine which lanes a car occupies. 
* If the car is in the process of changing lanes it can occupy two lanes.
* Then we determine the distance to our car. This can be negative, if the other car is behind.
* We check, if the car is the closest car ahead in the lane occupied by our car.
* We also check, if the car is left or right to us, thus blocking that lane for lane changes.
* We also determine a travel speed for each lane, which is the slowest car in that lane close to our car.

## Decide strategy
In this section we decide whether we stay in the current lane or switch lanes. This decision is based on the information of the previous section. The following criteria decide our target lane:
1. We can't change into lanes that were marked as blocked in the previous section.
2. We prefer lanes with higher travel speed or lanes without traffic ahead.
3. If several lanes have the same travel speed we prefer to stay in the current lane.
We also remember the lane travel speed and use it as our target speed for trajectory generation. If there is another car ahead we slightly decrease the target speed to avoid crashing.

## Trajectory generation
To generate a trajectory we use spline to get a smooth path. We use the old points from the previous path provided by the simulator and then add new points on that spline until the number of points reaches 50.
* To create the spline we first need to anchor it with the previous trajectory or, if there is none like at the start, with the car's current location.
* Then we add three new points spaced 60, 90 and 120 meters ahead using frenet coordinates. The d-value is derived from our target lane. Notice that the first point does get an intermediate d-value to avoid overshooting and thus leaving the lane.
* These three points are converted to xy coordinates.
* The anchor points and the three new points are then converted to the car coordinate system to simplify deriving the new trajectory points later.
* All these points in car coordinate system are used to create a spline.
* Then we use the spline to derive the new trajectory points.
* While doing that we constantly change the current speed in the direction of the target speed from the last section. The speed change is chosen, so that we do not exceed acceleration limits.
* We also check for speed and acceleration limit violations and adjust the trajectory points accordingly.
* The last step is to rotate and shift the points back to the global coordinate system.




