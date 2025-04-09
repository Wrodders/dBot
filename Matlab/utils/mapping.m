function [x, y, theta, linvel, stats] = mapping(data, dt, config)
    % Extract linear and angular velocities from the data
    linvel = data(config.start:config.stop, 15);
    angvel = data(config.start:config.stop, 18) * 2 * pi;  % Convert to rad/s
    stats.trgtVel = data(1, 16);
    stats.vel_dev = std(linvel);
    stats.vel_var = var(linvel);
    n = length(linvel);
    x = zeros(n, 1);
    y = zeros(n, 1);
    theta = zeros(n, 1);
    theta(1) = config.theta0;
    x(1) = config.x0;
    y(1) = config.y0;
    
    % Forward Euler integration for dead reckoning kinematics
    for i = 2:n
        theta(i) = theta(i-1) + angvel(i-1) * dt;
        x(i) = x(i-1) + linvel(i-1) * cos(theta(i-1)) * dt;
        y(i) = y(i-1) + linvel(i-1) * sin(theta(i-1)) * dt;
    end
    stats.steer_dev = std(data(:,19)*2*pi);
    stats.steer_var = var(data(:,19)*2*pi);
    stats.time = round((config.stop - config.start) * dt, 2);
end
