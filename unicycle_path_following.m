%path following a circle 
%unicycle-like type robot 
%controller that generates the control inputs for the cartesian system and
%the frenet frame combined
function unicycle_path_following(task_details)
c_s = task_details(1,1);
initial_position = task_details(:,2); %[0;1;0; 1; 0; 0]
duration = task_details(1,3);
[t,y] = ode45(@myODE1,[0 duration],initial_position);
plot(y(:,4),y(:,5),'LineWidth',2)
xlabel('x','FontSize',14);
ylabel('y','FontSize',14);


    function dy = myODE1(t,y)
        k1 = 1.6;
        k2 = 5;
        k3 = 10;
        dy = zeros(6,1);
        R = 2;
        %longitudinal velocity defined here for now should be an input
        %later
        u1 = 1;
        v1 = u1*cos(y(3))/(1-y(2)*c_s);
        %frenet frame dynamics
        dy(1) = v1; %s
        dy(2) = u1*sin(y(3)); %d
        dy(3) = (-v1*k2*y(2)-abs(v1)*k3*(1-y(2)*c_s)*tan(y(3))+dy(2)*c_s*tan(y(3)))/((1-y(2)*c_s)*(1+(tan(y(3)))^2)); %theta_e
        %cartesian coordinates dynamics
        dy(4) = u1*cos(y(6));
        dy(5) = u1*sin(y(6));
        dy(6) = (-v1*k2*y(2)-abs(v1)*k3*(1-y(2)*c_s)*tan(y(3))+dy(2)*c_s*tan(y(3)))/((1-y(2)*c_s)*(1+(tan(y(3)))^2))+c_s*dy(1);
    end

end
