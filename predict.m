v_chic = -1 * chic_data(end,4:5)'


m_chic = 6.82e15;
m_dart = 500;

theta = 90; %DART impact will always be normal to current asteriod velocity
R = [cosd(theta) -sind(theta) ; sind(theta) cosd(theta)];


%calculate new inital velocity using conservation of momentum
s_dart = 6.6e3; %km/h
v_dart = s_dart * (R* (v_chic / norm(v_chic)))


v_0 = vpa(((v_chic * m_chic) + (v_dart * m_dart)) / (m_chic + m_dart))

angle = vpa(norm(acosd(dot(v_0, v_chic) / (norm(v_0) * norm(v_chic)))))

angle = double(angle)

dist = ((6 * 6e6) / (tand(angle)));

time = dist / norm(v_dart) / (3600 * 24 * 364.25)