function holonomic_point(task_details)
    desired_location = task_details(:,1);
    initial_position = task_details(:,2);
    duration = task_details(1,3);
    [t,y] = ode45(@myODE1,[0 duration],initial_position);
    plot(t,y(:,1),t,y(:,2),t,y(:,3),'LineWidth',2)
    %title('Solution of van der Pol Equation (\mu = 1) with ODE45');
    xlabel('Time t','FontSize',14);
    ylabel('Solution (x, y, \theta)','FontSize',14);
    legend('x','y','\theta','FontSize',14)

    function dy = myODE1(t,y)
        k1 = 2;
        k2 = 2;
        k3 = 2;
        dy = zeros(3,1);
        A = [sin(y(3)) cos(y(3)) 0;-cos(y(3)) sin(y(3)) 0; 0 0 1];
        dy = A*inv(A)*[-k1 0 0;0 -k2 0;0 0 -k3]*(y-desired_location);
    end
end
