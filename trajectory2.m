function [time, data, impact] = trajectory2(time_of_DART_impact, t, state, dt)

%constants
G  = 6.67e-11; 
r_earth = 6378e3; %m
m_earth = 5.972e24; %kg


remain = time_of_DART_impact - t;
%define evaluation time rage
tspan = [0:dt:3600];

%run ODE45
options = odeset('Events', @collision);
[time, data, impact] = ode45(@rate_func, tspan, state, options);


    %rate function
    function r = rate_func(~, X)

    P = X(1:2);
    V = X(3:4);

    accel = (G * m_earth / norm(P)^2) * -(P / norm(P));

    dPdt = V;
    dVdt = accel;

    r = [dPdt ; dVdt];

    end

    %event function
    function [value, isterminal, direction] = collision(~, X)

    P = norm(X(1:2));
    %stop if distace to earth is small
        %use the half the rough distace between data points as an additional
        %tolerance
    value      =  P - r_earth - 1;%(dt * norm(v_0) / 2);
    isterminal = 1;
    direction  = 0;


    end


end