function res = plot_traj(data)


figure
clf
x = linspace(0,2*pi,100);
y = linspace(0,2*pi,100);
x = 6378*sin(x);
y = 6378*cos(y);

hold on

plot(data(:, 2)/1000, data(:, 3)/1000, ".-")
patch(x, y, "green")

xlabel("km")
ylabel("km")
title("Trajectory")
axis equal
axis([-10000 60000 -10000 30000])






hold off
end