clc;
include_namespace_dq;

%% Establish Connection
vi = DQ_VrepInterface;
vi.disconnect_all();
vi.connect('127.0.0.1', 19997)
vi.start_simulation();

%% Initialize VREP Robots

franka = FEpVrepRobot('Franka', vi);

%% Load DQ Robotics kinematics
fep  = fep_vreprobot.kinematics();

% maximum joint ranges (deg): (q1..q7)
q_min = [-2.8973   -1.7628   -2.8973   -3.0718   -2.8973   -0.0175  -2.8973];

%        166.0031  101.0010  166.0031 -3.9992   166.0031   215.0024  166.0031
q_max = [ 2.8973    1.7628    2.8973  -0.0698    2.8973    3.7525    2.8973];

   
%goal = [100*pi/180, 0.004, pi/4.0, -1.57156, pi/4.0, 1.57075, 0.0];
goal = [   0.3
    1
    0.2618
   -0.8727
         0
    1.3963
         0];

% Give the joint angles, cacluate the pose of the end effector. 

newGoal = fep.fkm(goal);

%normalization for new goal
normalizedNewGoal_dq = newGoal * inv(norm(newGoal));

% check if the value of normalizedNewGoal is 1
norm(normalizedNewGoal_dq)
