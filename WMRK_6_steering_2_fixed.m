function WMR = WMRK_6_steering_2_fixed
%% ======================Wheeled Mobile Robot Kinematics========================
% November 2021
%
% Setup of all the wheel and robot parameters and the relationships between
% them for hinged type robot  

%% ============================Numerical Setup=============================
WMR = struct();
WMR.Name = '6_steering_2_fixed';
WMR.Field = 'WMRK';

% Number of wheels
WMR.wheel_num = 8 ;
WMR.actuator_num = 3;

% transormation matrix of the robot body 
WMR.TR = struct();
WMR.TR.yaw_angle = 0;  % only one relevant for the current use case 
WMR.TR.pitch_angle = 0;
WMR.TR.roll_angle = 0;
WMR.TR.trans_x = 0;
WMR.TR.trans_y = 0;
WMR.TR.trans_z = 0;

% Reference frame of the robot (4x4)
WMR.tform_robot = TransformationMatrix(WMR.TR);

% Coordinates of center of wheels ROBOT FRAME
% NUMBER OF WHEELS : 4
WMR.wheels_radii = [3,3,3,3,3,3,3,3];

% This data will come from the platform used to make the robot structure
% Orientation data of wheels WRT Robot body (to get tform matrices, solver
% only gets tform matrices)
% ---------------- yaw z , pitch y , roll x ------------------------------
% WHEELS
% ------------------------------------------------------------------
% WHEEL 1
TR_wheel_1 = struct();
TR_wheel_1.yaw_angle = 0;
TR_wheel_1.pitch_angle = 0;
TR_wheel_1.roll_angle = pi/2;
TR_wheel_1.trans_x = -30;
TR_wheel_1.trans_y = 10;
TR_wheel_1.trans_z = 0;

% WHEEL 2
TR_wheel_2 = struct();
TR_wheel_2.yaw_angle = 0;
TR_wheel_2.pitch_angle = 0;
TR_wheel_2.roll_angle = -pi/2;
TR_wheel_2.trans_x = -30;
TR_wheel_2.trans_y = -10;
TR_wheel_2.trans_z = 0;

% WHEEL 3
TR_wheel_3 = struct();
TR_wheel_3.yaw_angle = 0;
TR_wheel_3.pitch_angle = 0;
TR_wheel_3.roll_angle = pi/2;
TR_wheel_3.trans_x = -20;
TR_wheel_3.trans_y = 10;
TR_wheel_3.trans_z = 0;

% WHEEL 4
TR_wheel_4 = struct();
TR_wheel_4.yaw_angle = 0;
TR_wheel_4.pitch_angle = 0;
TR_wheel_4.roll_angle = -pi/2;
TR_wheel_4.trans_x = -20;
TR_wheel_4.trans_y = -10;
TR_wheel_4.trans_z = 0;

% WHEEL 5
TR_wheel_5 = struct();
TR_wheel_5.yaw_angle = 0;
TR_wheel_5.pitch_angle = 0;
TR_wheel_5.roll_angle = pi/2;
TR_wheel_5.trans_x = 0;
TR_wheel_5.trans_y = 10;
TR_wheel_5.trans_z = 0;

% WHEEL 6
TR_wheel_6 = struct();
TR_wheel_6.yaw_angle = 0;
TR_wheel_6.pitch_angle = 0;
TR_wheel_6.roll_angle = -pi/2;
TR_wheel_6.trans_x = 0;
TR_wheel_6.trans_y = -10;
TR_wheel_6.trans_z = 0;

% WHEEL 7
TR_wheel_7 = struct();
TR_wheel_7.yaw_angle = 0;
TR_wheel_7.pitch_angle = 0;
TR_wheel_7.roll_angle = pi/2;
TR_wheel_7.trans_x = 20;
TR_wheel_7.trans_y = 10;
TR_wheel_7.trans_z = 0;

% WHEEL 8
TR_wheel_8 = struct();
TR_wheel_8.yaw_angle = 0;
TR_wheel_8.pitch_angle = 0;
TR_wheel_8.roll_angle = -pi/2;
TR_wheel_8.trans_x = 20;
TR_wheel_8.trans_y = -10;
TR_wheel_8.trans_z = 0;

% accessible to outside
WMR.wheel_tforms = [TransformationMatrix(TR_wheel_1); TransformationMatrix(TR_wheel_2);TransformationMatrix(TR_wheel_3);TransformationMatrix(TR_wheel_4);TransformationMatrix(TR_wheel_5); TransformationMatrix(TR_wheel_6);TransformationMatrix(TR_wheel_7);TransformationMatrix(TR_wheel_8)]; 


% ACTUATORS
% ------------------------------------------------------------------
% ACTUATOR 1    (for wheel)
TR_act_1 = struct();
TR_act_1.yaw_angle = 0;
TR_act_1.pitch_angle = 0;
TR_act_1.roll_angle = pi/2;
TR_act_1.trans_x = -30;
TR_act_1.trans_y = 10;
TR_act_1.trans_z = 0;

% ACTUATOR 2    (for hinge)
TR_act_2 = struct();
TR_act_2.yaw_angle = 0;
TR_act_2.pitch_angle = 0;
TR_act_2.roll_angle =0 ;
TR_act_2.trans_x = 0;
TR_act_2.trans_y = 0;
TR_act_2.trans_z = 0;


% ACTUATOR 3   (for hinge)
TR_act_3 = struct();
TR_act_3.yaw_angle = 0;
TR_act_3.pitch_angle = 0;
TR_act_3.roll_angle =0 ;
TR_act_3.trans_x = 10;
TR_act_3.trans_y = 0;
TR_act_3.trans_z = 0;

range_of_motions = [2*pi,pi/3,pi/3]; %range of motion of the actuators

WMR.actuator_tforms = [TransformationMatrix(TR_act_1); TransformationMatrix(TR_act_2);TransformationMatrix(TR_act_3)]; 
    


%% =================== COMPUTE ============================================
% Configuration wrt the robot body COMPUTE THIS
% Calling the Classify Component Function 

[wheels_f,wheels_s ,wmr_possible,wmr_type] = classify_components(WMR);
WMR.wheels_f = wheels_f;
WMR.wheels_s = wheels_s;



