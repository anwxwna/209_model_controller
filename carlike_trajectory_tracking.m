%trajetctory generation and tracking error control
%carlike-like type robot 
function carlike_trajectory_tracking(task_details)
    %u1 = task_details(1,1);
    %u2 = task_details(1,2);
    initial_position = task_details(:,2);
    duration = task_details(1,3);
    [t,y] = ode45(@myODE1,[0 duration],initial_position);
    plot(t,y(:,5),t,y(:,6),t,y(:,7),t,y(:,8),'LineWidth',2)
    xlabel('Time t','FontSize',14);
    ylabel('Solution (x_e, y_e, \theta_e,\phi_e)','FontSize',14);
    legend('x_e','y_e','\theta_e','\phi_e','FontSize',14)


    function dy = myODE1(t,y)
        k1=1;
        k2=3;
        k3=9;
        k4=6;
        L=1;
        %reference inputs
        if t<10
            u1=1;
            u2=0;
        elseif t>=10 && t<20
            u1=-1;
            u2 = 0.5*cos(2*pi*(t-10)/5);
        else 
            u1 = 1;
            u2=0;
        end 
%reference dynamics generation
        dy = zeros(8,1);    
        dy(1) = u1*cos(y(3));
        dy(2) = u1*sin(y(3));
        dy(3) = u1/L*tan(y(4));  
        dy(4) = u2;
        z3 = tan(y(7));
        z4 = (tan(y(8))-cos(y(7))*tan(y(4)))/L/(cos(y(7)))^3+k2*y(6);
%tracking error control
        dy(5) = dy(3)*y(6)-k1*abs(u1)*(y(5)+z3/k3*(z4+(1+z3^2*tan(y(4))/L)));
        dy(6) = -dy(3)*y(5)+(-k1*abs(u1)*(y(5)+z3/k3*(z4+(1+z3^2*tan(y(4))/L)))+u1)*tan(y(7));
        dy(7) = (-k1*abs(u1)*(y(5)+z3/k3*(z4+(1+z3^2*tan(y(4))/L)))+u1)/cos(y(7))/L*tan(y(8))-dy(3);
        dy(8) = (-k3*u1*z3-k4*abs(u1)*z4-k2*dy(6)-(3*tan(y(8))/cos(y(7))-2*tan(y(4)))*sin(y(7))*dy(7)/L/(cos(y(7)))^3+u2/(L*(cos(y(4)))^2*(cos(y(7)))^2))*L*(cos(y(8)))^2*(cos(y(7)))^3-u2;
    end
end
