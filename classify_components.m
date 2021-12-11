function [wheels_f,wheels_s,wmr_possible,wmr_type] = classify_components(WMR)
%% AXLE MAPS ==============================================================
wmr_type = '';
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
wmr_possible =0;
% if else ladder approach to generate model
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
                % Don't include duplicates 
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
% Actuators =============================================================
% actuator wrt ROBOT FRAME
actsEUL = [];
act_centers=[];
for i=1:actuator_num
     actsEUL = [actsEUL ; tform2eul(actuator_tforms(4*(i-1)+1:4*(i-1)+1+3,1:4))];
     act_center = actuator_tforms(4*(i-1)+1:4*(i-1)+1+3,1:4)*[0;0;0;1];
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
    axle_vectors = [axle_vectors;axle_vec];  % same axle wheels

%   Relative position of actuators wrt all the axles
    for k=1:actuator_num
%   Going through each actuator in loop 
         act_vector_z = actuator_tforms(4*(k-1)+1:4*(k-1)+1+3,1:4) *robot_z_axis;
         angle = acos(min(1,max(-1, act_vector_z(:).' * axle_vec(:) / norm(act_vector_z) / norm(axle_vec) )));
       
         % Actuator parallel to wheel axles---------
%        Adding the ACTIVE wheels to the active set (fixed wheels)
         if (angle == 0) || (angle == pi) 
             
            if isequal( act_centers(k,1:4), wheel_centers(w1,1:4) ) && isequal(act_centers(k,1:4), wheel_centers(w2,1:4))
                active = [active, w1,w2];
            elseif act_centers(k,1:4) == wheel_centers(w1,1:4)
                active = [active, w1];
                passive_but_same_axle_as_active = w2;
            elseif act_centers(k,1:4) == wheel_centers(w2,1:4)
                active = [active, w2];
                passive_but_same_axle_as_active = w1;
            
            end
%  Check if actuated wheels share same axes
%  If steered wheels on same axes as fixed wheels, consider as fixed
            % Actuator located orthogonal to wheel axles-------------------
%           Center steerable wheels are the passive wheels with a hinged
%           actuator orthogonal to it 
%             This gives steering wheels that are co-aligned and oriented
%             together fulfilling requirement that 
         elseif angle == pi/2
            steering = [steering,w1,w2];
         end
         act_axle_angles = [act_axle_angles;angle];
    end
end
steering = unique(steering);
% Only passive wheel axles are considered steering wheels
for i=1:length(steering)-1
    if ismember(steering(i), active)
        steering(i) = [];
    end
    
end
if ismember( passive_but_same_axle_as_active,steering)
   steering(steering == passive_but_same_axle_as_active) = [];
end
%% [CHECK FOR SINGULARITIES]=============================
icrs = [];
% ICR of wheels
for i=1:length(steering)
    for j=2:length(steering)
        if i ~= length(steering)
            shift_i =i+1;
        else
            shift_i =1;
        end
        if j ~= length(steering)
            shift_j =j+1;
        else
            shift_j =1;
        end
        
        x = [wheel_centers(i,1:3); wheel_centers(shift_i,1:3)];
        y = [wheel_centers(j,1:3); wheel_centers(shift_j,1:3)];
        [intersection_point ,dist]= lineIntersect3D(x,y);
        icrs = [icrs;intersection_point];
    end
end
% This will set the wmr_possible variable for the particular robotic
% configuration reference: https://ms.copernicus.org/articles/7/93/2016/ms-7-93-2016.pdf

%  TYPE A --------------------------
% wheel axes of all wheels of a robot with coaligned center-steerable wheels (HW-type Vb) coincide
% ≥ 2 cs wheels , 0 fixed wheels
if (length(steering) >=2) && (isempty(active))
    for i=length(steering)
        for j=length(steering)
            if isKey(same_axle_map, strcat(int2str(i),'_',int2str(j)))
                wmr_possible = 0;
            end
        end
    end
end

%  TYPE B --------------------------
% A singularity of type A is only present for robots with coaligned wheel 
% contact points (HW-type Vb). When the wheel contact points are not on a
% straight line (HW-type Va), then one or more (but not all) wheel axes may be coaligned.
% ≥ 3 cs wheels , 0 fixed wheels
if (length(steering) >=3) && (isempty(active))
    for i=length(steering)
        for j=length(steering)
            if i ~=j
                if isKey(same_axle_map, strcat(int2str(i),'_',int2str(j)))
                    wmr_possible = 0;
                end
            end
        end
    end
end
    
%  TYPE C --------------------------
% A robot is in a type C singularity when the ICR coincides with the contact point of a center-steerable standard wheel
if (length(steering) >=2) && (isempty(active))
   for i= 1:length(icrs)
       for j=1:length(steering)
        if isequal(icrs(i),wheel_centers(steering(j),1:3)) ==1
            wmr_possible =0;
        end
       end
   end

end
% ACTIVE CHECK Check if there are active wheels that are not on the same axle -------------
for i=1:length(active)
    for j=1:length(active)
        if (i~=j) && (i<j)
            if isKey(same_axle_map, strcat(int2str(i),'_',int2str(j))) ~= 1
                wmr_possible=0;
            end
        end
    end
end
%% ANGLE calculations =====================================================
% calculate alpha, beta and l for each wheel wrt P=========================
% Use the tform matrix to get the coords wrt to the ROBOT frame from the
% WHEEL frame
wheels_alphas = [];
wheels_betas = [];
for i=1:wheel_num
    % Get vector of point from P to wheel center -> WP
%   Wheel_center= wheel_centers((3*(i-1))+1:(3*(i-1))+2,1);
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
%% Classification==========================================================
% After counting all the actuated wheels of differnt types 
% if the number of fixed wheels is == 0
if isempty(active) ==0
%     If number of c-steerable wheels is 0
    if isempty(steering)
        wmr_type = '(3,0)';
    %     If number of c-steerable wheels is 1
    elseif length(steering) == 1
        wmr_type = '(2,1)';
    %     If number of c-steerable wheels is >=3
    elseif length(steering) >=3
        wmr_type = '(1,2)';
    %     If number of c-steerable wheels is >=2
    elseif length(steering) >=2
        wmr_type = '(2,0)';
        wmr_type = '(1,2)';
    end
    
end
% if number of fixed wheels is > =1
%     If number of c-steerable wheels is 0
%     If number of c-steerable wheels is >=1
if length(active) >=1
    if isempty(steering)
        wmr_type = '(2,0)';
    elseif length(steering) >=1
            wmr_type = '(1,1)';
    end
end
% These should be output of the above lines

for i=fixed
    wheels_f = [wheels_f; wheels_alphas(i),wheels_betas(i),norm(wheel_centers(i))];
end
% wmr_possible = 1;
        
        