# neo_goal_sequence_driver

## It's a package for testing basic functionalities of navigation stack.


1. Currently tested on MP-500 under neo_track1.world, and it's working fine;

2. User can change (x, y, theta) values of each goal, also the number of goals;

3. For testing, please:
	
	1) run .../neo_simulation/simulation.launch;
	2) run .../neo_goal_sequence_driver/launch/neo_goal_sequence_driver_server.launch;
	   (The robot should be moving along the sequence.)
	3) run .../neo_goal_sequence_driver/launch/neo_goal_sequence_driver_client.launch
	   there're 4 arguments available, if you want to modify them, read the .srv file;
	4) manipulate goals in .../neo_goal_sequence_driver/config/goal_list.yaml;
	5) turn to ii. and repeat if you want to;
