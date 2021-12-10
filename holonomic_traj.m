%holonomic robot 
%trajectory tracking
%feedback linearization

function holonomic_traj(task_details)
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
        k1=2;
        k2=1.2;
        k3=0.2;
        if t<5
            u1=1;
            u2=-0.5;
            u3=0;
        elseif t>=5 && t<10
            u1=-1;
            u2=1;
            u3 = cos(2*pi*(t-10)/5);
        else 
            u1 = 1;
            u2 = -0.5;
            u3 = 0;
        end 
%reference dynamics
        dy = zeros(6,1);    
        dy(1) = u1*sin(y(3))+u2*cos(y(3));
        dy(2) = -u1*cos(y(3))+u2*sin(y(3));
        dy(3) = u3;  
%tracking error dynamics
        A = [sin(y(6)) cos(y(6)) 0;-cos(y(6)) sin(y(6)) 0; 0 0 1];
        dy(4:6)=[u3*y(5);-u3*y(4);-u3]+A*inv(A)*([-k1 -u3 0;u3 -k2 0;0 0 -k3]*y(4:6)+[0 0 u3]'); 
    end

end
