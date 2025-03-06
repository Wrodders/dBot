% Data extraction function
function [timestamps, measurement, reference, controlaction] = extractMotorData(filename)
    % Open Loop Ramp 0 - 16 over 50 Seconds
    [timestamps, timedelta, data] = parseDataSet(filename);
    measurement = data(:, 9);
    reference = data(:, 10);
    controlaction = data(:, 11);
end