function WMRK = WheeledMobileRobotKinematics_sym
%% ======================Wheeled Mobile Robot Kinematics========================
% November 2021
%
% Setup of all the wheel and robot parameters and the relationships between
% them 

%% ============================Numerical Setup=============================
WMR = struct();
WMR.Name = 'Wheeled Mobile Robot Sym';
WMR.Field = 'WMRK';

% Number of Degrees of Freedom
WMR.DOF = 3;

% transormation matrix of the robot body 
roll_angle = sym("roll_angle");
yaw_angle  = sym("yaw_angle");
pitch_angle = sym("pitch_angle");
trans_x = sym("trans_x");
trans_y = sym("trans_y");
trans_z= sym("trans_z");

WMR.TR = struct();
WMR.TR.roll_angle = roll_angle;
WMR.TR.yaw_angle = yaw_angle;
WMR.TR.pitch_angle = pitch_angle;
WMR.TR.trans_x = trans_x;
WMR.TR.trans_y = trans_y;
WMR.TR.trans_z = trans_z;


% Reference frame of the robot (4x4)
WMR.tform_robot = TransformationMatrix(WMR.TR);

% Coordinates of center of wheels 
WMR.wheels_centers = [];
WMR.actuators_centers = [];

% Orientation of wheels 
WMR.wheel_orientation = []; % one 
WMR.wheel_orientations = [];
WMR.actuator_orientations = [];
    
% Coordinates of actuators 


%% =================== COMPUTE ============================================
% Configuration wrt the robot body COMPUTE THIS
WMR.wheels_f = [];
WMR.wheels_s = [];




%% ========================Mathematical Modeling===========================


%% =====================Create Symbolic Definitions========================
WMR.symbs = struct();
syms th1 th2 th3 th4 th5
WMR.symbs.thiSym = sym(zeros(WMR.DOF,1));
WMR.symbs.alphasSym = sym(zeros(WMR.DOF,1));
WMR.symbs.thetasSym = sym([th1; th2; th3; th4; th5]);
for i = 1:WMR.DOF
    WMR.symbs.alphasSym(i) = WMR.DH.alphas(i);
    WMR.symbs.thiSym(i) = WMR.th.thi(i);
end

%% ===============Transform each point in the global frame=================
WMRK = RotateKinematicChain(KinematicSystem(WMR), zeros(WMR.DOF, 1));