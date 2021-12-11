function [time, data] = trajectory1(time_of_DART_impact, dt1, dt2, chic_data)

%constants
G  = 6.67e-11; 
r_earth = 6378e3; %m
m_earth = 5.972e24; %kg

%inital conditions
chic_dt = chic_data(2,1) - chic_data(1,1);
index = int32((time_of_DART_impact + chic_dt)/ chic_dt);
r_0 = chic_data(index,2:3);
v_chic = -1 * chic_data(index,4:5);

%find DART impact velocity
theta = 90; %DART impact will always be normal to current asteriod velocity
R = [cosd(theta) -sind(theta) ; sind(theta) cosd(theta)];


%calculate new inital velocity using conservation of momentum
s_dart = 17e3; %km/h
v_dart = s_dart * (R* (v_chic / norm(v_chic))')';


m_dart = 420000; %kg
m_chic = 6.82e15; %kg


v_0 = vpa(((m_dart * v_dart) + (m_chic * v_chic)) / (m_dart + m_chic));


v_0 = double(v_0);



%combine initial position and velocity
x_0 = [r_0' ; v_0'];

%define evaluation time rage


tspan1 = 0:dt1:time_of_DART_impact;



%run ODE45
options = odeset('Events', @proximity);
[time1, data1] = ode45(@rate_func, tspan1, x_0, options);


tspan2 = time1(end):dt2:time_of_DART_impact + (3600 * 24 * 10);


options = odeset('Events', @collision);
[time2, data2] = ode45(@rate_func, tspan2, data1(end,1:4)', options);

time = [time1 ; time2];
data = [data1 ; data2];

    %rate function
    function r = rate_func(~, X)

    P = X(1:2);
    V = X(3:4);

    accel = (G * m_earth / norm(P)^2) * -(P / norm(P));

    dPdt = V;
    dVdt = accel;

    r = [dPdt ; dVdt];

    end


    function [value, isterminal, direction] = proximity(~, X)

    P = norm(X(1:2));
    %stop if distace to earth is small
        %use the half the rough distace between data points as an additional
        %tolerance
    value      =  P - (12 * r_earth);%(dt * norm(v_0) / 2);
    isterminal = 1;
    direction  = 0;


    end

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