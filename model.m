function [T, X] = model()%years in advance

%constants
G  = 8.6496e-11; %km^3/mh^2
r_earth = 6000; %km
m_earth = 5.972e24; %kg

%"final" condition
theta = 40;  %impact angle
v = 12*3600;  %impact velocity
v_0 = [cosd(theta) * v ;  sind(theta) * v];
r_0 = [r_earth ; 0];

%initial state
x_0 = [r_0 ; v_0];



[T, X] = ode45(@rate_func, [0:0.001:(365*1*24)], x_0);

function r = rate_func(~, W)

P = W(1:2);
V = W(3:4);

accel = (G * m_earth / norm(P)^2) * (P / norm(P));

dPdt = V;
dVdt = accel;

r = [dPdt ; dVdt];

end


end