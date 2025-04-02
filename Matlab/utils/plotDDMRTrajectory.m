function plotDDMRTrajectory(ax, x, y, theta, linvel, config)
    % Plot the path trajectory
    plot(ax, x, y, config.path_line_style, 'LineWidth', 2, ...
         'Color', config.colors.path, 'DisplayName', 'Path');
    hold on;
    % Plot heading arrows at subsampled intervals
    idx = 1:config.heading_subsample:length(x);
    quiver(ax, x(idx), y(idx), cos(theta(idx)) .* linvel(idx), ...
           sin(theta(idx)) .* linvel(idx), config.heading_scale, ...
           'Color', config.colors.heading, 'DisplayName', 'Heading');
    hold off
    grid minor;

  
end


