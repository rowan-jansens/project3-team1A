function [T, W] = model()

G  = 6.67408e-11;
r_earth = 6.37e6;
m_earth = 5.972e24;
r_impactor = 1e3;

% impactor [x pos. ; y pos. ; x vel. ; y vel.]
W = [1.5e11 ; 0 ; 0 ; 0];

options = odeset('Events', @collision);
[T, W] = ode45(@rate_func, [0 10000], W, options);

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

value      = norm(P) - (r_earth + r_impactor);
isterminal = 1;
direction  = -1;

end

end