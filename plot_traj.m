function res = plot_traj(data)


figure
clf
x = linspace(0,2*pi,100);
y = linspace(0,2*pi,100);
x = 6000*sin(x);
y = 6000*cos(y);

hold on

plot(data(:, 2), data(:, 3), ".-")
patch(x, y, "green")

xlabel("km")
ylabel("km")
title("Trajectory")
axis equal
axis([-10000 60000 -20000 20000])






hold off
end