function [T, W] = model(rx_0) % km

%constants
G  = 8.6496e-11; %km^3/mh^2
r_earth = 6000; %km
m_earth = 5.972e24; %kg
r_impactor = 40; %km

theta = 40;
v = 12*3600;
v_0 = [cosd(theta) * v ;  sind(theta) * v];
r_0 = [rx_0 ; 0];
x_0 = [r_0 ; v_0];

options = odeset('Events', @collision);
[T, W] = ode45(@rate_func, [0, (365*1*24)], x_0, options);

function r = rate_func(~, W)

P = W(1:2);
V = W(3:4);

accel = (G * m_earth / norm(P)^2) * (P / norm(P));

dPdt = V;
dVdt = accel;

r = [dPdt ; dVdt];

end


end