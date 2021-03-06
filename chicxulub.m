function [res] = chicxulub(hours, dt)

%constants
G  = 6.67e-11;%8.6496e-11; %km^3/mh^2
r_earth = 6378e3; %m
m_earth = 5.972e24; %kg

%"final" conditions; we're calcluating in reverse
theta = 60;  %impact angle
v = 12e3;  %impact velocity (m/s)
v_0 = [cosd(theta) * v ;  sind(theta) * v];
%r_0 = [0 ; r_earth];
r_0 = [r_earth ; 0];

%initial state
x_0 = [r_0 ; v_0];

tspan = [0:dt:hours];

[time, data] = ode45(@rate_func, tspan, x_0);
res = [time data];

    function r = rate_func(~, X)

    P = X(1:2);
    V = X(3:4);

    accel = (G * m_earth / norm(P)^2) * -(P / norm(P));

    dPdt = V;
    dVdt = accel;

    r = [dPdt ; dVdt];

    end


end