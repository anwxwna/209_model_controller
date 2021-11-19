function func = kinematics_generate(theta, alpha,beta, l , d)
% KINEMATICS OF WHEELED ROBOT 
% This function seeks to generate the forward kinematics of robot given a
% struct that encapsulates sufficient data about the robot
% PSEUDOCODE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Count all the different types of Wheels in the system N_s,N_f
% Get the coordinates and Orientation data for every frame for the wheel
% and the robot body
% 

R_theta = [cos(theta) sin(theta) 0;
           -sin(theta) cos(theta) 0;
           0           0         1  ];

% Rolling constraints

% No-slip constraints

% 