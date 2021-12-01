function [T, W] = model()

G  = 8.6496e-11; %km^3/mh^2
r_earth = 6000; %km
m_earth = 5.972e24; %kg
r_impactor = 40; %km

% impactor [x pos. ; y pos. ; x vel. ; y vel.]
W = [2.67e8 ; 2.67e8 ; -12 ; 0];

options = odeset('Events', @collision);
[T, W] = ode45(@rate_func, [0 600000], W, options);

function r = rate_func(~, W)

P = W(1:2);
V = W(3:4);

accel = (G * m_earth / norm(P)^2) * -(P / norm(P));

dPdt = V;
dVdt = accel;

r = [dPdt ; dVdt];

end

function [value, isterminal, direction] = collision(~, W)

P = W(1:2);

value      = ~(norm(P) < (r_earth));
isterminal = 1;
direction  = 0;

end

end