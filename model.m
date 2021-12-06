function [time, data, impact] = model(time_of_DART_impact, dt, chic_data)

%constants
G  = 6.67e-11; %km^3/mh^2
r_earth = 6378e3; %km
m_earth = 5.972e24; %kg

%inital conditions
index = time_of_DART_impact;
r_0 = chic_data(index,2:3);
v_0 = -1 * chic_data(index,4:5);


%combine initial position and velocity
x_0 = [r_0' ; v_0'];

%define evaluation time rage
tspan = [0:dt:time_of_DART_impact];

%run ODE45
options = odeset('Events', @collision);
[time, data, impact] = ode45(@rate_func, tspan, x_0);


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