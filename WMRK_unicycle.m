function WMR = WMRK_unicycle
%% ======================Wheeled Mobile Robot Kinematics========================
% November 2021
%
% Setup of all the wheel and robot parameters and the relationships between
% them for unicycle type robot  

%% ============================Numerical Setup=============================
WMR = struct();
WMR.Name = 'unicycle';
WMR.Field = 'WMRK';

% Number of wheels
WMR.wheel_num = 2;
WMR.actuator_num = 2;

% transormation matrix of the robot body 
WMR.TR = struct();
WMR.TR.roll_angle = 0;
WMR.TR.yaw_angle = 0;
WMR.TR.pitch_angle = 0;
WMR.TR.trans_x = 0;
WMR.TR.trans_y = 0;
WMR.TR.trans_z = 0;

% Reference frame of the robot (4x4)
WMR.tform_robot = TransformationMatrix(WMR.TR);

% Coordinates of center of wheels ROBOT FRAME
% NUMBER OF WHEELS : 2
WMR.wheels_radii = [3,3];

% Orientation data of wheels WRT Robot body (to get tform matrices, solver
% only gets tform matrices)
% yaw z , pitch y  , roll x
% WHEEL 1
TR_wheel_1 = struct();
TR_wheel_1.yaw_angle = 0;
TR_wheel_1.pitch_angle = 0;
TR_wheel_1.roll_angle = pi/2;
TR_wheel_1.trans_x = -2;
TR_wheel_1.trans_y = 5;
TR_wheel_1.trans_z = 0;
orientation_w1 = TransformationMatrix(TR_wheel_1);

% WHEEL 2
TR_wheel_2 = struct();
TR_wheel_2.yaw_angle = 0;
TR_wheel_2.pitch_angle = 0;
TR_wheel_2.roll_angle = -pi/2;
TR_wheel_2.trans_x = -2;
TR_wheel_2.trans_y = -5;
TR_wheel_2.trans_z = 0;
orientation_w2 = TransformationMatrix(TR_wheel_2);

% accessible to outside
WMR.wheel_tforms = [orientation_w1; orientation_w2]; 
WMR.actuator_tforms = [orientation_w1; orientation_w2]; 
    


%% =================== COMPUTE ============================================
% Configuration wrt the robot body COMPUTE THIS
% Calling the Classify Component Function 

[wheels_f,wheels_s ,wmr_possible] = classify_components(WMR);
WMR.wheels_f = wheels_f;
WMR.wheels_s = wheels_s;




%% ========================Mathematical Modeling===========================


%% =====================Create Symbolic Definitions========================
% WMR.symbs = struct();
% syms th1 th2 th3 th4 th5
% WMR.symbs.thiSym = sym(zeros(WMR.DOF,1));
% WMR.symbs.alphasSym = sym(zeros(WMR.DOF,1));
% WMR.symbs.thetasSym = sym([th1; th2; th3; th4; th5]);
% for i = 1:WMR.DOF
%     WMR.symbs.alphasSym(i) = WMR.DH.alphas(i);
%     WMR.symbs.thiSym(i) = WMR.th.thi(i);
% end

%% ===============Transform each point in the global frame=================
