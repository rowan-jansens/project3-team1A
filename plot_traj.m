function res = plot_traj(T, X)


figure
clf
x = linspace(0,2*pi,100);
y = linspace(0,2*pi,100);
x = 6000*sin(x);
y = 6000*cos(y);

hold on

plot(X(:, 1), X(:, 2))
patch(x, y, "green")

xlabel("km")
ylabel("km")
title("Trajectory")
axis equal




hold off
end