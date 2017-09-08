# Model documentation

The model generation consists in this project consists of three main parts:
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

## 


