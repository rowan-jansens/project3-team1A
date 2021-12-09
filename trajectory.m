function [time, data, impact, angle] = trajectory(time_of_DART_impact, dt, chic_data)

%constants
G  = 6.67e-11; 
r_earth = 6378e3; %m
m_earth = 5.972e24; %kg

%inital conditions
chic_dt = chic_data(2,1) - chic_data(1,1)
index = int32((time_of_DART_impact + chic_dt)/ chic_dt);
r_0 = chic_data(index,2:3);
v_chic = -1 * chic_data(index,4:5);

%find DART impact velocity
theta = 90; %DART impact will always be normal to current asteriod velocity
R = [cosd(theta) -sind(theta) ; sind(theta) cosd(theta)];


%calculate new inital velocity using conservation of momentum
s_dart = 17e3; %km/h
v_dart = s_dart * (R* (v_chic / norm(v_chic))')



m_dart = 1e11; %kg
m_chic = 6.82e15; %kg


v_0 = vpa(((m_dart * v_dart') + (m_chic * v_chic)) / (m_dart + m_chic));
%v_0 = norm(v_chic) * v_0 / norm(v_0);  %there is a problem here

angle = vpa(acosd(dot(v_0, v_chic) / (norm(v_0) * norm(v_chic))));
angle = double(angle);

v_0 = double(v_0)

%metrics
da = norm(v_0 - v_chic);
p = norm(v_chic) * m_chic;
val = (da  *1000) * m_chic;


%combine initial position and velocity
x_0 = [r_0' ; v_0'];

%define evaluation time rage
tspan = [0:dt:time_of_DART_impact + 10*dt];

%run ODE45
options = odeset('Events', @collision);
[time, data, impact] = ode45(@rate_func, tspan, x_0, options);


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