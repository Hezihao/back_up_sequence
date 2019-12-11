# neo_goal_sequence_driver

## It's a package for testing basic functionalities of navigation stack.


1. Currently tested on MP-500 under neo_track1.world, and it's working fine;

2. User can change (x, y, theta) values of each goal, also the number of goals;

3. For testing, please:
	
	a) run .../neo_simulation/simulation.launch;
	b) run .../neo_goal_sequence_driver/launch/neo_goal_sequence_driver.launch;
	   (The robot should be moving along the sequence.)
	c) manipulate goals in .../neo_goal_sequence_driver/config/goal_list.yaml;
	d) turn to b) and repeat if you want to;
