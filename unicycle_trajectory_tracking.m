%trajetctory generation and tracking error control
%reference inputs are defined here and not given as inputs
%should be changed in the future
%unicycle-like type robot 
function unicycle_trajectory_tracking(task_details)
    %u1 = task_details(1,1);
    %u2 = task_details(1,1);
    initial_position = task_details(:,2);
    duration = task_details(1,3);
    [t,y] = ode45(@myODE1,[0 duration],initial_position);
    plot(t,y(:,4),t,y(:,5),t,y(:,6),'LineWidth',2)
    xlabel('Time t','FontSize',14);
    ylabel('Solution (x_e, y_e, \theta_e)','FontSize',14);
    legend('x_e','y_e','\theta_e','FontSize',14)




    function dy = myODE1(t,y)
        k1=3;
        k2=10;
        k3=8;
        %reference inputs
        if t<5
            u1=1;
            u2=0;
        elseif t>=5 && t<10
            u1=-1;
            u2 = cos(2*pi*(t-10)/5);
        else 
            u1 = 1;
            u2=0;
        end 
%reference dynamics
        dy = zeros(6,1);    
        dy(1) = u1*cos(y(3));
        dy(2) = u1*sin(y(3));
        dy(3) = u2;  
%the tracking error dynamics
        dy(4) = u2*y(5)-k1*abs(u1)*(y(4)+y(5)*tan(y(6)));
        dy(5) = -u2*y(4)+(-k1*abs(u1)*(y(4)+y(5)*tan(y(6)))+u1)*tan(y(6));
        dy(6) = (-k2*u1*y(5)-k3*abs(u1)*tan(y(6)))*(cos(y(6)))^2;
    end

end