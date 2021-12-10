%path following a circle 
%controller that generates the control inputs for the cartesian system and
%the frenet frame conbines

function carlike_pathfollowing(task_details)
c_s = task_details(1,1);
initial_position = task_details(:,2); %[0;1;0; 1; 0; 0]
duration = task_details(1,3);
[t,y] = ode45(@myODE1,[0 duration],initial_position);
plot(y(:,5),y(:,6),'LineWidth',2)
xlabel('x','FontSize',14);
ylabel('y','FontSize',14);


    function dy = myODE1(t,y)
        k1 = 1.6;
        k2 = 1;
        k3 = 3;
        k4 = 3;
        L = 1;
        dy = zeros(8,1);
        R = 4;
        %the longitudinal velocity defined here should be an input later
        if t<=5
            u1 = 1;
        else 
            u1=-1;
        end
        z1 = y(1);
        z2 = y(2);
        z3 = (1-y(2)*c_s)*tan(y(3));
        z4 = -c_s*(1-y(2)*c_s)*(1+2*tan(y(3))^2)+(1-y(2)*c_s)^2*tan(y(4))/L*(1+2*tan(y(3))^2)/cos(y(3));
        v1 = u1*cos(y(3))/(1-y(2)*c_s);
        v2 = -abs(v1)*k2*z2-v1*k3*z3-abs(v1)*k4*z4;
        %frener frame dynamics
        dy(1) = v1;
        dy(2) = u1*sin(y(3));
        dy(3) = u1*tan(y(4))/L-dy(1)*c_s;
        dy(4) = v2-dy(2)*c_s^2*(1+2*tan(y(3))^2)+dy(3)*c_s*(1-y(2)*c_s)*4*tan(y(3))/(cos(y(4)))^2+2*(1-y(2)*c_s)*dy(2)*c_s*tan(y(4))/L*(1+2*(tan(y(3)))^2)/cos(y(3))-(1-y(2)*c_s)^2*tan(y(4))/L*dy(3)*(2*tan(y(3))/(cos(y(3)))^3+(1+2*(tan(y(3)))^2)*sin(y(3))/(cos(y(3)))^2);
        %cartesian frame dynamics
        dy(5) = u1*cos(y(7));
        dy(6) = u1*sin(y(7));
        dy(7) = u1*tan(y(8))/L;
        dy(8) = dy(4);
    end

end
