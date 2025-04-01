
function animateTrajectory(x, y, theta, linvel, dt, config, varargin)
    % Parse inputs for the animation
    p = inputParser;
    addRequired(p, 'x');
    addRequired(p, 'y');
    addRequired(p, 'theta');
    addRequired(p, 'linvel');
    addRequired(p, 'dt');
    addRequired(p, 'config');
    addParameter(p, 'PlaybackSpeed', 1, @isnumeric);
    addParameter(p, 'FigureVisible', 'on', @ischar);
    addParameter(p, 'PathStyle', '-.', @ischar);
    addParameter(p, 'Title', 'Robot Trajectory Animation', @ischar);
    parse(p, x, y, theta, linvel, dt, config, varargin{:});
    
    % Update configuration for path style if needed
    config.plotStyle = p.Results.PathStyle;
    frameDelay = dt / p.Results.PlaybackSpeed;
    
    % Create figure for animation
    figAnim = figure('Visible', p.Results.FigureVisible, 'Color', 'w');
    axAnim = axes(figAnim);
   
    
    % Plot static trajectory elements on the animation figure
    plotDDMRTrajectory(axAnim, x, y, theta, linvel, config);
    plotStartEndMarkers(axAnim, x, y);
    hold(axAnim, 'on');
    % Set fixed axis limits with a 10% margin to prevent dynamic resizing
    x_range = max(x) - min(x);
    y_range = max(y) - min(y);
    x_margin = 0.1 * x_range;
    y_margin = 0.1 * y_range;
    xlim(axAnim, [min(x)-x_margin, max(x)+x_margin]);
    ylim(axAnim, [min(y)-y_margin, max(y)+y_margin]);
    axis(axAnim, 'equal');
    axis(axAnim, 'manual');  % Lock the axis limits
    
    % Initialize animated plot elements (robot marker and heading arrow)
    h_robot = plot(axAnim, x(1), y(1), 'bo', 'MarkerSize', 10, ...
                   'MarkerFaceColor', 'b', 'LineWidth', 2, 'DisplayName', 'Current Position');
    h_arrow = quiver(axAnim, x(1), y(1), cos(theta(1))*linvel(1), sin(theta(1))*linvel(1), ...
                    'AutoScale', 'off', 'Color', 'k', 'LineWidth', 1.5, ...
                    'MaxHeadSize', 0.5, 'DisplayName', 'Current Heading');
    
    % Configure axes appearance and title
    title(axAnim, p.Results.Title, 'FontSize', 20); 
    set(axAnim, 'FontSize', 14, 'XGrid', 'on', 'YGrid', 'on', 'GridAlpha', 0.3, 'LineWidth', 1.2);
    
    % Animation loop: update the robot's position and heading arrow
    for i = 1:length(x)
        set(h_robot, 'XData', x(i), 'YData', y(i));
        set(h_arrow, 'XData', x(i), 'YData', y(i), ...
            'UData', cos(theta(i)) * linvel(i), 'VData', sin(theta(i)) * linvel(i));
        drawnow;
        pause(frameDelay);
    end
    hold(axAnim, 'off');
end

