%controller that generates the control inputs for the cartesian system


function unicylce_point_expon(task_details)
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
        alpha = 0.1;
        q = 2;
        p = 5;
        k1 = 2;
        k2 = 10;
        k3 = 5;
        dy = zeros(3,1);

        rho = (abs(y(2)-desired_location(2))^(p/(q+1))+abs(tan(y(3)-desired_location(3))^(p/q)))^(1/p);
        dy(1) = -k1*((y(1)-desired_location(1))*sin(t)-abs(y(1)-desired_location(1)))*sin(t)+alpha*rho*sin(t);
        %print(dy(1))
        dy(2) = dy(1)*tan(y(3));
        dy(3) = (-dy(1)*k2*(y(2)-desired_location(2))/rho^2-abs(dy(1)-desired_location(1))*k3*tan(y(3)-desired_location(3))/rho)/(1+(tan(y(3)-desired_location(3)))^2);
    end

end

