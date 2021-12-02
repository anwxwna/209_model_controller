function [wheels_f,wheels_s,wmr_possible] = classify_components(WMR)
%% AXLE MAPS ==============================================================
wheels_f =[];
wheels_s =[];
wheel_num = WMR.wheel_num ;
wheel_tforms = WMR.wheel_tforms;
actuator_num = WMR.actuator_num ;
actuator_tforms = WMR.actuator_tforms;

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
                    same_axle_map(mapping_name) = [wheel_tforms(4*(i-1)+1:4*(i-1)+1+3,1:4);wheel_tforms(4*(j-1)+1:4*(j-1)+1+3,1:4)];
                end
            
    %       Parallel axles  
            elseif (sign(transXYZi(2)) == sign(transXYZj(2))) && (transXYZi(3) == transXYZj(3)) && (eulZYXi(1) == 0) && (eulZYXi(2) == 0 ) && (eulZYXj(1) == 0) && (eulZYXj(2) == 0 )
                inv_map = strcat(int2str(j),'_',int2str(i));
                if isKey(par_axle_map,inv_map) ~=1
                    mapping_name = strcat(int2str(i),'_',int2str(j));
                    par_axle_map(mapping_name) = [wheel_tforms(4*(i-1)+1:4*(i-1)+1+3,1:4);wheel_tforms(4*(j-1)+1:4*(j-1)+1+3,1:4)];
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
    wheel_centers = [wheel_centers;wheel_center];
end
%% AXLE-WISE GROUPINGS iterations
for i=keys(same_axle_map)
%   Pair wise (Q: Can we have more than two wheels on the same axle? )
%     sa = same_axle_map{i};
%     sa = strsplit(sa{1},'_');
%     w1 = str2int(sa{1});
%     w2 = str2int(sa{2});
%     axle_vec = wheel_centers(w1:w1+3,1)- wheel_centers(w2:w2+3,1);
%   Relative position of actuators wrt all the wheels
    for k=actuator_num
         actEUL = tform2eul(actuator_tforms(4*(k-1)+1:4*(k-1)+1+3,1:4));
%         actTR = tform2trvec(wheel_tforms(4*(i-1)+1:4*(i-1)+1+3,1:4));
         act_center = actuator_tforms(4*(k-1)+1:4*(k-1)+1+3,1:4)*[0;0;0;1];
    end
    % Actuator parallel to wheel axles
    % Actuator located orthogonal to wheel axles 
    %   Steering Wheels
    %   Others (?)
    % Passive Wheels 
    % Check radius of wheels on the same axle
    % Distance between wheels on the same axle
    % Distance between wheels on parallel axles
end
%% ICR of wheels (?) [CHECK FOR SINGULARITIES]=============================

%% ANGLE calculations =====================================================
% calculate alpha, beta and l for each wheel wrt P=========================
% Use the tform matrix to get the coords wrt to the ROBOT frame from the
robot_x_axis = [1;0;0;0];
% WHEEL frame
wheels_alphas = [];
wheels_betas = [];
for i=1:wheel_num
    % Get vector of point from P to wheel center -> WP
    wheel_center= wheel_centers((3*(i-1))+1:(3*(i-1))+2,1);
    wheel_center(4) = 0;
%   wheel_center_vec = wheel_center
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

fixed = [1,2];
steering = [];
for i=fixed
    wheels_f = [wheels_f; wheels_alphas(i),wheels_betas(i),norm(wheel_centers(i))];
end
wmr_possible = 1;
end