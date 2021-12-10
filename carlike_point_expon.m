%stabilization to a point 
%exponential no lipschitz controller 
%Car-like type robot 
function carlike_point_expon(task_details)
    desired_location = task_details(:,1);
    initial_position = task_details(:,2);
    duration = task_details(1,3);
    [t,y] = ode45(@myODE1,[0 duration],initial_position);
    plot(t,y(:,1),t,y(:,2),t,y(:,3),t,y(:,4),'LineWidth',2)
    xlabel('Time t','FontSize',14);
    ylabel('Solution (x, y, \theta,\phi)','FontSize',14);
    legend('x','y','\theta','\phi','FontSize',14)
    function dy = myODE1(t,y)
        alpha = 0.6;
        q = 2;
        p = 5;
        k1 = 1.6;
        k2 = 10;
        k3 = 18;
        k4 = 10;
        dy = zeros(4,1);
        L = 1;
        y = y - desired_location;
        rho = (abs(y(2))^(p/(q+2))+abs(tan(y(3)))^(p/(q+1))+abs(tan(y(4))/L*(1+(tan(y(3)))^2)/cos(y(3)))^(p/q))^(1/p);
        dy(1) = -k1*(y(1)*sin(t)-abs(y(1)))*sin(t)+alpha*rho*sin(t);
        dy(2) = (-k1*(y(1)*sin(t)-abs(y(1)))*sin(t)+alpha*rho*sin(t))*tan(y(3));
        dy(3) = (-k1*(y(1)*sin(t)-abs(y(1)))*sin(t)+alpha*rho*sin(t))*tan(y(4))/(L*cos(y(3)));
        dy(4) = L*(-abs(dy(1))*k2*y(2)/rho^3-dy(1)*k3*tan(y(3))/rho^2-abs(dy(1))*k4*tan(y(4))/L*(1+(tan(y(3)))^2)/cos(y(3))/rho)*(cos(y(4)))^2*(cos(y(3)))^3-3*(sin(y(4)))^2*tan(y(3))*dy(1)/(L*cos(y(3)));

    end


end
