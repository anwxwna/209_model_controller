function posture_kine = kinematics_generate(WMR)
% KINEMATICS OF WHEELED ROBOT 
% This function seeks to generate the forward kinematics of robot given a
% struct that encapsulates sufficient data about the robot
% PSEUDOCODE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Count all the different types of Wheels in the system N_s,N_f
% Get the coordinates and Orientation data for every frame for the wheel
% and the robot body
% 
% wheels_x : alpha,beta, l , d  WHEEL DATA in array. One row per wheel 

% Rotation matrix for the robot body 
wheels_f = WMR.wheels_f;
wheels_s = WMR.wheels_s;
% trans_input = WMR.trans_input;

R_theta = WMR.tform_robot;

% Rolling constraints
roll_f = rolling_constraints_matrix(wheels_f);  % N_f x 3
roll_s = rolling_constraints_matrix(wheels_s);  % N_s x 3
roll= [roll_f; roll_s];

% roll_c = roll*R_theta.';
% No-slip constraints
no_slip_f = no_slip_constraints_matrix(wheels_f);
no_slip_s = no_slip_constraints_matrix(wheels_s);
no_slip = [no_slip_f;no_slip_s];

% no_slip_c = no_slip*R_theta.';   % for posture kinematics

% this will be multiplied with the the velocity and steering inputs to get
% the 
posture_kine = R_theta.'*null(no_slip);   
% posture_kine = no_slip;
                        




