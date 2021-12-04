function [time, data] = trajectory(time_of_DART_impact, chic_data)

%constants
G  = 8.6496e-11; %km^3/mh^2
r_earth = 60000; %km
m_earth = 5.972e24; %kg

P_old = 1e20;

%inital conditions
index = time_of_DART_impact;
r_0 = chic_data(index,2:3);
v_0 = -1 * chic_data(index,4:5);
x_0 = [r_0' ; v_0'];

tspan = [0:0.1:index*2];


options = odeset('Events', @collision);
[time, data] = ode45(@rate_func, tspan, x_0, options);

function r = rate_func(t, X)

P = X(1:2);
V = X(3:4);

accel = (G * m_earth / norm(P)^2) * -(P / norm(P));

dPdt = V;
dVdt = accel;

r = [dPdt ; dVdt];

end


function [value, isterminal, direction] = collision(t, X)

P = norm(X(1:2));
value      =  P - r_earth - 100
isterminal = 1;
direction  = 0;


end


end