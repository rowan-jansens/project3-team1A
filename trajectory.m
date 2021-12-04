function [time, data] = trajectory(time_of_DART_impact, dt, chic_data)

%constants
G  = 8.6496e-11; %km^3/mh^2
r_earth = 6000; %km
m_earth = 5.972e24; %kg



%inital conditions
index = time_of_DART_impact / dt;
r_0 = chic_data(index,2:3);
v_chic = -1 * chic_data(index,4:5);


%find DART impact velocity
theta = -90; %DART impact will always be normal to current asteriod velocity
R = [cosd(theta) -sind(theta) ; sind(theta) cosd(theta)];


s_dart = 6.6 * 3600; %km/h
v_dart = s_dart * (R* (v_chic / norm(v_chic))');



m_dart = 50000000; %kg
m_chic = 6.28e15; %kg


v_new = ((m_dart * v_dart') + (m_chic * v_chic)) / (m_dart + m_chic);





v_0 = norm(v_chic) * v_new / norm(v_new);



da = norm(v_0 - v_chic) * (3600^2 * 1000^2)



acos(dot(v_0, v_chic) / (norm(v_0) * norm(v_chic))) * 180 / pi







x_0 = [r_0' ; v_0'];

tspan = [0:dt:time_of_DART_impact];


options = odeset('Events', @collision);
[time, data] = ode45(@rate_func, tspan, x_0, options);

function r = rate_func(~, X)

P = X(1:2);
V = X(3:4);

accel = (G * m_earth / norm(P)^2) * -(P / norm(P));

dPdt = V;
dVdt = accel;

r = [dPdt ; dVdt];

end


function [value, isterminal, direction] = collision(~, X)

P = norm(X(1:2));
value      =  P - r_earth - 4;
isterminal = 1;
direction  = 0;


end


end