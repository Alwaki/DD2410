# DD2410
Assignments + Project for robotics course KTH

Current Status:

ROS: Finished rudimentary part (so ready for presentation), however not implemented a more sophisticated action server. Probably not going to, as I've read all the theory and no further credit is given in the course for implementing it.

KINEMATICS: Finished kinematics problem, E and C parts are completed. Ready for turnin, but Kattis is not yet functional with additional python libraries. Supposed to be fixed at end of summer according to staff.

PLANNING: Changed algorithm from A* to RRT. Current implementation of algorithm has been able to solve 6/6 E cases, and 4/6 C cases. However, due to random nature of algorithm, results vary. Problem with consistency of C cases, often gets stuck in corridors. Idea to fix is geometrically limiting the random generation of new coordinates for certain intervals. Furthermore, need to add comments for this code.

MAPPING: Finished part E, however not yet attempted to implement part C.

