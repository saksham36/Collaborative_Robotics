# CS339R Collaborative Robotics: Collaborative Strategy using a Hierarchical Bayes Controller

This is the demo code for the collaborative strategy designed for the Collaborative Robotics class.

## Setup instructions

```bash
pip install -r requirements.txt
```

## Problem Statement

The goal of the project was to program a robot for a joint dual robot resource gathering game. Both robots in the field are Trossen Robotics's LoCoBots. The game begins with the 2 LoCoBots placed randomly in the field containing randomly scattered red, blue, green, and yellow blocks. Each robot is provided with the information of the 4 possible base stations, the configuration of the blocks to be kept at each base station, as well as the color of blocks each robot is allowed to pick up.

Given:

- Target configuration
- Permissible color of blocks that can be picked up
- Location of 4 possible base station locations
  ![Target Configuration](https://github.com/saksham36/Collaborative_Robotics/blob/collab_strategy/imgs/target_config.png?raw=true)

### Assumptions

To approach this collaboration problem, we first adopted the following assumptions:

1. The number of blocks in the world is equal to the number of blocks needed in the locations, i.e, there are no extra blocks
2. Blocks won’t move in the map, unless picked up, i.e, blocks aren’t displaced due to collision or external forces
3. Once a block has been picked and placed on a location, it will not be picked up again
4. A block picked up, will be dropped at a location, i.e, the robot will not the drop midway and leave it
5. There is no overlap in the color of blocks that the robots are allowed to pick up
6. The other LoCoBot can observe the world same as us, i.e, no partial observability
7. The time taken to pick and drop block is the proportional to distance
8. Robots will not collide with each other

### Formulation

We adopt a hierarchical Bayesian approach to solve this 2 agent problem. We formulate the problem as a Semi-Markov Decision Process with unspecified rewards.

**State:** The state space S consists of all possible combinations of blocks in the 4 possible locations. A state can be represented as a 4 × 4 matrix

![Initial State](https://github.com/saksham36/Collaborative_Robotics/blob/collab_strategy/imgs/init_state.png?raw=true)

**Action:** An action is the act of picking up a particular block and dropping it to a base station. Hence, the size of the action space |A| is equal to the number of blocks in the map. Additionally, since each robot can only pick some certain colors, which are disjoint to each other, the action space can be separated into A1, A2, where A1 ∪ A2 = A, and A1 ∩ A2 = ∅

**Transition time:** We model F to be the time taken to pick and place the block corresponding to the action to a base station. This is proportional to the distance of the block from the current location of the robot and the distance of the block to the selected location

**Belief:** Is the probability of a location being a base station. In our problem, we have 4 possible locations, and 3 possible locations. So belief at time t is a 4 × 3 matrix. The initial belief is set to be an uniform distribution.

![Initial Belief](https://github.com/saksham36/Collaborative_Robotics/blob/collab_strategy/imgs/init_belief.png?raw=true)

### Algorithm

1. Explore the world by spinning in 360 degrees in place (from initial location). Since the map, is free of any walls, rotating in place would provide complete information of the world.
2. Once the robot finishes its action, it will check if finished its action before the other robot. If so, it adopts the role of a leader, assuming that the other robot would adopt a follower strategy. Conversely, if it finishes second, it will assume the follower strategy. In the beginning, the action is the exploration, in future steps, it would be the action of picking and dropping a cube.
   1. As a leader: the robot will select a cube to put to a base station that results in maximum change in the shared belief of the world
   2. As a follower: the robot will select a cube to put to a base station resulting in minimum change in shared belief of the world
3. Repeat step 2, until the desired configuration is reached (success), or if the configuration leads to an inconsistent shared belief (failure)

When a robot has to select a cube to be placed at a location, it asks the following questions which have to be answered.

1. What's my belief to the mapping of locations to base stations?
2. Which color cube should I pick up?
3. Which cube of the color decided should I pick up?

![Algorithm Hierarchical Abstraction](https://github.com/saksham36/Collaborative_Robotics/blob/collab_strategy/imgs/hierarchical.png?raw=true)

- The high level controller answers the second question. Depending on what role the robot is adopting, using the information from the state, and current belief, to select the color from the list of allowed colored cubes that the robot can pick up, that will change the belief of the robot the most or the least, by conducting a 1-step rollout.
- The meta-controller checks that the color selected, and resultant future belief, does not contradict the configuration requirements of the task.
- The low-level controller answers the third question. Now that the high level controller has presented a valid color, the robot has to select a cube of that color. In our algorithm, the cube of the specified color that minimizes the expected time taken to pick and drop the cube is selected.

![Time Equation](https://github.com/saksham36/Collaborative_Robotics/blob/collab_strategy/imgs/time.png?raw=true)

### Bayesian Update

Bayes update can be applied as follows:

![Bayes Equation](https://github.com/saksham36/Collaborative_Robotics/blob/collab_strategy/imgs/bayes.png?raw=true)

Which simplifies to:

![Simplified Bayes Equation](https://github.com/saksham36/Collaborative_Robotics/blob/collab_strategy/imgs/bayes_simple.png?raw=true)

The belief is then normalized. Additionally, since we assume that there are no extra blocks in the map, if 3 locations have blocks placed in them, then automatically, we set the probability of the empty location to be a base station to be 0.

## Results

We show that for the case where both robots follow the above collaborative strategy, the target configuration is achieved no matter which robot begins or the placement of the block.

### Case 1: Robot 1 is the first to complete exploration, and assumes leadership role, Robot 2 assumes follower role at t=0.

**Action Sequences:**

```
T = 0
Robot_1 chooses to put cube 3 which is green to goal A which it assumes is location 0
Robot_2 chooses to put cube 5 which is blue to goal A which it assumes is location 0
T=17
Robot_2 chooses to put cube 8 which is yellow to goal A which it assumes is location 0
T=26
Robot_1 chooses to put cube 4 which is green to goal C which it assumes is location 1
T=28
Robot_2 chooses to put cube 9 which is yellow to goal B which it assumes is location 2
T=52
Robot_1 chooses to put cube 2 which is red to goal A which it assumes is location 0
T=55
Robot_2 chooses to put cube 7 which is blue to goal B which it assumes is location 2
T=65
Robot_1 chooses to put cube 0 which is red to goal A which it assumes is location 0
T=79
Robot_2 chooses to put cube 6 which is blue to goal C which it assumes is location 1
T=82
Robot_1 chooses to put cube 1 which is red to goal B which it assumes is location 2
```

** Change in the state, belief, and configuration in time**

![State](https://github.com/saksham36/Collaborative_Robotics/blob/collab_strategy/gifs/state_leader_0.gif)
![Belief](https://github.com/saksham36/Collaborative_Robotics/blob/collab_strategy/gifs/belief_leader_0.gif)
![Configuration](https://github.com/saksham36/Collaborative_Robotics/blob/collab_strategy/gifs/config_leader_0.gif)

### Case 2: Robot 2 is the first to complete exploration, and assumes leadership role, Robot 1 assumes follower role at t=0.

**Action Sequences:**

```
T = 0
Robot_1 chooses to put cube 2 which is red to goal A which it assumes is location 0
Robot_2 chooses to put cube 8 which is yellow to goal A which it assumes is location 0
T=11
Robot_2 chooses to put cube 9 which is yellow to goal B which it assumes is location 2
T=13
Robot_1 chooses to put cube 3 which is green to goal C which it assumes is location 1
T=38
Robot_2 chooses to put cube 5 which is blue to goal A which it assumes is location 0
T=39
Robot_1 chooses to put cube 0 which is red to goal A which it assumes is location 0
T=55
Robot_2 chooses to put cube 7 which is blue to goal B which it assumes is location 2
T=56
Robot_1 chooses to put cube 4 which is green to goal A which it assumes is location 0
T=79
Robot_2 chooses to put cube 6 which is blue to goal C which it assumes is location 1
T=83
Robot_1 chooses to put cube 1 which is red to goal B which it assumes is location 2
```

** Change in the state, belief, and configuration in time**

![State](https://github.com/saksham36/Collaborative_Robotics/blob/collab_strategy/gifs/state_leader_1.gif)
![Belief](https://github.com/saksham36/Collaborative_Robotics/blob/collab_strategy/gifs/belief_leader_1.gif)
![Configuration](https://github.com/saksham36/Collaborative_Robotics/blob/collab_strategy/gifs/config_leader_1.gif)

The problem devolves into a turn-based game, due to the SMDP structure of the strategy. **This behavior emerges from the algorithm itself and is not hard-coded.**

## Credits

This code has been written solely by Saksham Consul ([**@saksham36**](https://github.com/saksham36)) ([**sconsul@stanford.edu**](sconsul@stanford.edu)) with discussion with Carlota Parés-Morlans ([**@carlotapares**](https://github.com/carlotapares)) ([**cpares@stanford.edu**](cpares@stanford.edu))
