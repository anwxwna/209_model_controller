function func = classify_components(WMR)
wheel_num = WMR.wheel_num ;
wheel_tforms = WMR.wheel_tforms;

% if else ladder approach to generate model


% Check orientation of wheels add to an array 
% Add euler angles of the wheels from the tform matrices into an array
% eulerXYZs = [];

% Get directional vectors for all the wheels 
wheels = [];
for i=1:wheel_num:4

    eulZYX = tform2eul(wheel_tforms(i:i+3,1:4));
    transXYZ = tform2trvec(wheel_tforms(i:i+3,1:4));
    wheelx = [eulZYX,transXYZ];
    wheels = [wheels ; wheelx];
end

same_axle_map = containers.Map;
par_axle_map = containers.Map;
for i=1:wheel_num
    eulZYXi = wheels(i:i+3,1);
    transXYZi = wheels(i:i+3,2);

    for j=1:wheel_num
        eulZYXj = wheels(i:i+3,1);
        transXYZj = wheels(i:i+3,2);

% Check if wheels on same axle add to array 
% x angle of rotation is opposite and the wheel coordinate center is the
% same
%       Wheels are same axle -> z-axis is aligned 
        if (transXYZi(1) == transXYZj(1)) && (sign(transXYZi(2)) ~= sign(transXYZj(2))) && (transXYZi(3) == transXYZj(3)) && (eulZYXi(1) == 0) && (eulerXYZi(2) == 0 ) && (eulZYXj(1) == 0) && (eulerXYZj(2) == 0 )
            mapping_name = strcat(int2str(i),'_',int2str(j));
            same_axle_map(mapping_name) = [eulZYXi,transXYZi,wheel_tforms(4*(i-1)+1:4*(i-1)+1+3,1:4); eulZYXj,transXYZj,wheel_tforms(4*(j-1)+1:4*(j-1)+1+3,1:4)];
        
%       Parallel axles  
        elseif (sign(transXYZi(2)) == sign(transXYZj(2))) && (transXYZi(3) == transXYZj(3)) && (eulZYXi(1) == 0) && (eulerXYZi(2) == 0 ) && (eulZYXj(1) == 0) && (eulerXYZj(2) == 0 )
            mapping_name = strcat(int2str(i),'_',int2str(j));
            par_axle_map(mapping_name) = [eulZYXi,transXYZi,wheel_tforms(4*(i-1)+1:4*(i-1)+1+3,1:4); eulZYXj,transXYZj,wheel_tforms(4*(j-1)+1:4*(j-1)+1+3,1:4)];

        end
%         Axles are at some angles


% OLD
%     if (eulZYX(1) == 0) && (eulerXYZ(2)) == 0 % only considering rotation about x-axis here, for wheels wrt robot body
%         x_angle = eulZYX(3);
%     if x_angle > 0
%             Left side 
%             if (x_angle == pi/2) || (x_angle == -pi/2)
%             straight wheels
            

    end
end


% AXLE-WISE GROUPINGS iterations
% Check radius of wheels on the same axle

% distance between wheels on the same axle

% Center of wheels in ROBOT FRAME
for i=1:wheel_num
    tform= wheel_tforms(4*(i-1)+1:4*(i-1)+1+3,1:4);
    wheel_center = [0;0;0]*tform;
    wheel_centers = [wheel_centers;wheel_center];
end

for i=1:length(same_axle_map)
end
    
% distance between wheels on parallel axles

% ICR of wheels (?) [CHECK FOR SINGULARITIES]==============================

% calculate alpha, beta and l for each wheel wrt P=========================
% Use the tform matrix to get the coords wrt to the ROBOT frame from the
robot_x_axis = [1;0;0];
% WHEEL frame
wheels_alphas = [];
for i=1:wheel_num
    % Get vector of point from P to wheel center -> WP
%   Distance between center points -> transform [0,0,0] to wheel frame mult
%   with corresponding tform for the wheel 
    tform= wheel_tforms(4*(i-1)+1:4*(i-1)+1+3,1:4);
    wheel_center = [0;0;0]*tform;
    wheel_centers = [wheel_centers;wheel_center];

%     center= wheel_centers((3*(i-1))+1,(3*(i-1))+2);
    alpha = atan2(norm(cross(wheel_center,robot_x_axis)),dot(wheel_center,robot_x_axis)); % Angle in radians
%   if angle alpha more than pi -> wheel behind P
%   If angle alpha less than pi -> wheel infront of P
    wheels_alphas = [wheels_alphas, alpha];

    % Angle between WP and Z vector for the wheel -> beta
    wheel_z_vec = [0;0;1]*tform; % z-vector of the wheel in robot frame 
    beta = atan2(norm(cross(wheel_center,wheel_z_vec)),dot(wheel_center,wheel_z_vec)); % Angle in radians
    wheels_betas = [wheels_betas, beta];

end



% Relative position of actuators wrt all the wheels 
% Actuator parallel to wheel axles
% Actuator located orthogonal to wheel axles 
%   Steering Wheels
%   Others (?)

% Passive Wheels 

% 

% kinematic tree approach -> that would be more general but I'm also trying
% to wrap my head around it 
end
