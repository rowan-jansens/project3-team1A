dt = 100;
t_range = year2sec * 0.1 : year2sec * 0.5 : year2sec * 6;
impacts = zeros(2, length(t_range));



figure(1)
clf
set(figure(1), "Position", [0 0 1200 500])
subplot(1,2,1)
    x = linspace(0,2*pi,100);
    y = linspace(0,2*pi,100);
    x = 6378*sin(x);
    y = 6378*cos(y);
    
    
    
    hold on
    patch(x, y, "green")
    

for i=1:length(t_range)
    [t, X, impact] = trajectory(t_range(i), dt, chic_data); % if DART hits at max_time-i, will the impactor impact?
    impacts(1,i) = ~isempty(impact);
    impacts(2,i) = min(vecnorm(X(:,1:2)'));
    if isempty(impact)
        plot(X(:, 1)/1000, X(:, 2)/1000, ".-", "Color", [0.4660 0.6740 0.1880], "LineWidth", 0.5, "MarkerSize", 5)
    else
        plot(X(:, 1)/1000, X(:, 2)/1000, ".-r", "LineWidth", 0.5, "MarkerSize", 5)
    end
end
    
    xlabel("km")
    ylabel("km")
    title("Trajectory")
    axis equal
    axis([-20000 20000 -20000 30000])
    hold off
    
    
subplot(1,2,2)
yyaxis right
plot(t_range / year2sec, impacts(2,:) / 1000)
ylabel("Minimum Distance (km)")

yyaxis left
plot(t_range / year2sec, impacts, ".-", "Markersize", 20)
xlabel("Time of DART impact (years before potential earth impact)")
ylabel("Earth Impact")
ylim([-0.5 1.5])
yticks([0 1])