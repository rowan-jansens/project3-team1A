function [T, W] = model()%years in advance

G  = 8.6496e-11; %km^3/mh^2
r_earth = 6000; %km
m_earth = 5.972e24; %kg
r_impactor = 40; %km

%v_0 = [-12*3600 ; 0];
%d = -1 * v_0(1) * 365 * 24
%theta = deg2rad(0.009);
%r_0 = [cos(theta) * d ; sin(theta)*d]

% impactor [x pos. ; y pos. ; x vel. ; y vel.]

theta = 40
v = 12*3600;
v_0 = [cosd(theta) * v ;  sind(theta) * v]

r_0 = [r_earth ; 0];


x_0 = [r_0 ; v_0];

options = odeset('Events', @collision);


[T, W] = ode45(@rate_func, [0, (365*1*24)], x_0);

function r = rate_func(~, W)

P = W(1:2);
V = W(3:4);

accel = (G * m_earth / norm(P)^2) * (P / norm(P));

dPdt = V;
dVdt = accel;

r = [dPdt ; dVdt];

end

function [value, isterminal, direction] = collision(~, W)

P = W(1:2);

value      = ~(norm(P) < (r_earth + r_impactor));
isterminal = 1;
direction  = 0;

end

end