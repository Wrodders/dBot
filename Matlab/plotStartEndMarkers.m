function plotStartEndMarkers(ax, x, y)
    % Plot the start marker
    plot(ax, x(1), y(1), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'g', ...
         'Color', 'g', 'DisplayName', 'Start');
    
    % Plot the end marker
    plot(ax, x(end), y(end), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'r', ...
         'Color', 'r', 'DisplayName', 'End');
    
end
