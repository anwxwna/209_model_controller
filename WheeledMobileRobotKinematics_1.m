function WMRK = WheeledMobileRobotKinematics_1
%% ======================Wheeled Mobile Robot Kinematics========================
% November 2021
%
% Setup of all the wheel and robot parameters and the relationships between
% them 

%% ============================Numerical Setup=============================
WMR = struct();
WMR.Name = 'Wheeled Mobile Robot 1';
WMR.Field = 'WMRK';

% Number of Degrees of Freedom
WMR.DOF = 3;


% Coordinates of center of wheels 
WMR.wheels = [];
WMR.
% Orientation of wheels 

% Coordinates of actuators 

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