%stabilization to a point 
%type21
function type2_1point_expon(task_details)
    desired_location = task_details(:,1);
    initial_position = task_details(:,2);
    duration = task_details(1,3);
    [t,y] = ode45(@myODE1,[0 duration],initial_position);
    plot(t,y(:,1),t,y(:,2),t,y(:,3),t,y(:,4),'LineWidth',2)
    xlabel('Time t','FontSize',14);
    ylabel('Solution (x, y, \theta,\beta)','FontSize',14);
    legend('x','y','\theta','\beta','FontSize',14)

    function dy = myODE1(t,y)
        alpha = 0.6;
        q = 2;
        p = 5;
        k1 = 1.6;
        k2 = 10;
        k3 = 9;
        dy = zeros(4,1);
        y=y-desired_location;
        rho = (abs(y(2))^(p/(q+1))+abs(tan(y(3)+y(4)))^(p/q))^(1/p);
        dy(1) = -k1*(y(1)*sin(t)-abs(y(1)))*sin(t)+alpha*rho*sin(t);
        dy(2) = (-k1*(y(1)*sin(t)-abs(y(1)))*sin(t)+alpha*rho*sin(t))*tan(y(3)+y(4));
        dy(3) = (-dy(1)*k2*y(2)/rho^2)/(1+(tan(y(3)+y(4)))^2)-y(3)/0.1;
        dy(4) = (-abs(dy(1))*k3*tan(y(4)+y(3))/rho)/(1+(tan(y(4)+y(3)))^2)-y(4)/0.1;

    end


end
