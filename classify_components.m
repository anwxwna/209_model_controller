function [wheels_f,wheels_s,wmr_possible] = classify_components(WMR)
%% AXLE MAPS ==============================================================
wheels_f =[];
wheels_s =[];
wheel_num = WMR.wheel_num ;
wheel_tforms = WMR.wheel_tforms;
actuator_num = WMR.actuator_num ;
actuator_tforms = WMR.actuator_tforms;
fixed = [];
steering = [];
active = [];
robot_x_axis = [1;0;0;0];
robot_z_axis = [0;0;1;0];
robot_y_axis = [0;1;0;0];
% if else ladder approach to generate model================================
% Check orientation of wheels add to an array 
% Add euler angles of the wheels from the tform matrices into an array
% Get directional vectors for all the wheels 
wheelsEUL = [];
wheelsTR = [];
for i=1:wheel_num
    wheelsEUL = [wheelsEUL ; tform2eul(wheel_tforms(4*(i-1)+1:4*(i-1)+1+3,1:4))];
    wheelsTR = [wheelsTR ; tform2trvec(wheel_tforms(4*(i-1)+1:4*(i-1)+1+3,1:4))];
end

same_axle_map = containers.Map;
par_axle_map = containers.Map;
actuator_axle_ortho_map = containers.Map;
actuator_axle_par_map = containers.Map;

for i=1:wheel_num
    eulZYXi = wheelsEUL(i,1:3);
    transXYZi = wheelsTR(i,1:3);

    for j=1:wheel_num
        eulZYXj = wheelsEUL(j,1:3);
        transXYZj = wheelsTR(j,1:3);

        if i~=j
            % Check if wheels on same axle add to array 
            % x angle of rotation is opposite and the wheel coordinate
            % center is same
            % Wheels are same axle -> z-axis is aligned 
            if (transXYZi(1) == transXYZj(1)) && (sign(transXYZi(2)) ~= sign(transXYZj(2))) && (transXYZi(3) == transXYZj(3)) && (eulZYXi(1) == 0) && (eulZYXi(2) == 0 ) && (eulZYXj(1) == 0) && (eulZYXj(2) == 0 )
    %       Don't include duplicates 
                inv_map = strcat(int2str(j),'_',int2str(i));
                if isKey(same_axle_map,inv_map) ~=1
                    mapping_name = strcat(int2str(i),'_',int2str(j));
                    same_axle_map(mapping_name) = [i,j];
                end
            
    %       Parallel axles  
            elseif (sign(transXYZi(2)) == sign(transXYZj(2))) && (transXYZi(3) == transXYZj(3)) && (eulZYXi(1) == 0) && (eulZYXi(2) == 0 ) && (eulZYXj(1) == 0) && (eulZYXj(2) == 0 )
                inv_map = strcat(int2str(j),'_',int2str(i));
                if isKey(par_axle_map,inv_map) ~=1
                    mapping_name = strcat(int2str(i),'_',int2str(j));
                    par_axle_map(mapping_name) = [i,j];
                end
          
            end
%       Axles are at some angles (not pi/2)
%       TODO
    % WHEEL TILTS
    %     if (eulZYX(1) == 0) && (eulerXYZ(2)) == 0 % only considering rotation about x-axis here, for wheels wrt robot body
    %         x_angle = eulZYX(3);
    %     if x_angle > 0
    %             Left side 
    %             if (x_angle == pi/2) || (x_angle == -pi/2)
    %             straight wheels
        end
    end
end

%% GROUPING ITERATIONS ====================================================
wheel_centers = [];
for i=1:wheel_num
%   Center of wheels in ROBOT FRAME
%   Distance between center points -> transform [0,0,0,1] to wheel frame mult
%   with corresponding tform for the wheel 
    tform= wheel_tforms(4*(i-1)+1:4*(i-1)+1+3,1:4);
    wheel_center = tform*[0;0;0;1];
    wheel_centers = [wheel_centers;wheel_center.'];
end
% Actuators
% actuator wrt ROBOT FRAME
actsEUL = [];
act_centers=[];
for i=actuator_num
     actsEUL = [actsEUL ; tform2eul(actuator_tforms(4*(i-1)+1:4*(i-1)+1+3,1:4))];
     act_center = actuator_tforms(4*(i-1)+1:4*(i-1)+1+3,1:4)*[0;0;0;1];
     act_center = tform*[0;0;0;1];
     act_centers = [act_centers;act_center.'];
end
%% AXLE-WISE GROUPINGS iterations =========================================
axle_vectors=[];
act_axle_angles = [];
for i=keys(same_axle_map)
%   Pair wise (Q: Can we have more than two wheels on the same axle? )
    W = same_axle_map(i{1});
    w1 = W(1);
    w2 = W(2);
    axle_vec = wheel_centers(w1,1:4)- wheel_centers(w2,1:4);
    axle_vectors = [axle_vectors;axle_vec];
%   Relative position of actuators wrt all the axles
    for k=actuator_num
%          axle_actuator_map_name = strcat(int2str(i),'_',int2str(k));
         act_center = actuator_tforms(4*(k-1)+1:4*(k-1)+1+3,1:4) *robot_z_axis;
         angle = acos(min(1,max(-1, act_center(:).' * axle_vec(:) / norm(act_center) / norm(axle_vec) )));
         % Actuator parallel to wheel axles
         if angle == 0
            if act_centers(k) == wheel_centers(w1,1:4)
                active = [active, w1];
            elseif act_centers(k) == wheel_centers(w2,1:4)
                active = [active, w2];
            end
    % Actuator located orthogonal to wheel axles 
         elseif angle == pi/2
            steering = [steering,w1,w2];
         end
         act_axle_angles = [act_axle_angles;angle];
    end

    
 
    %   Steering Wheels
    %   Others (?)
    % Passive Wheels 
    % Check radius of wheels on the same axle
    % Distance between wheels on the same axle
    % Distance between wheels on parallel axles
end
%% ICR of wheels (?) [CHECK FOR SINGULARITIES]=============================
icr = [];



%% ANGLE calculations =====================================================
% calculate alpha, beta and l for each wheel wrt P=========================
% Use the tform matrix to get the coords wrt to the ROBOT frame from the

% WHEEL frame
wheels_alphas = [];
wheels_betas = [];
for i=1:wheel_num
    % Get vector of point from P to wheel center -> WP
%     wheel_center= wheel_centers((3*(i-1))+1:(3*(i-1))+2,1);
    wheel_center = wheel_centers(i,1:4);
    wheel_center(4) = 0;
    wheel_center= wheel_center.';
%   alpha = atan2(norm(cross(wheel_center,robot_x_axis)),dot(wheel_center,robot_x_axis)); % Angle in radians
    alpha = acos(min(1,max(-1, wheel_center(:).' * robot_x_axis(:) / norm(wheel_center) / norm(robot_x_axis) )));
%   if angle alpha more than pi -> wheel behind P
%   If angle alpha less than pi -> wheel infront of P
    wheels_alphas = [wheels_alphas, alpha];

    % Angle between WP and Z vector for the wheel -> beta
    wheel_z_vec = tform*[0;0;1;0]; % z-vector of the wheel in robot frame 
%     beta = atan2(norm(cross(wheel_center,wheel_z_vec)),dot(wheel_center,wheel_z_vec)); % Angle in radians
    beta = acos(min(1,max(-1, wheel_center(:).' * wheel_z_vec(:) / norm(wheel_center) / norm(wheel_z_vec) )));
    wheels_betas = [wheels_betas, beta];
end

% These should be output of the above lines
fixed = [1,2];
% steering = [];

for i=fixed
    wheels_f = [wheels_f; wheels_alphas(i),wheels_betas(i),norm(wheel_centers(i))];
end
wmr_possible = 1;
end