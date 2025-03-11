% Left Motor Data extraction function
function [timestamps, measurement, reference, controlaction] = extractMotorData(filename)
    [timestamps, timedelta, data] = parseDataSet(filename);
    measurement = data(:, 9);

    reference = data(:, 10);
    reference = [reference(2:end); reference(end)]; % Reference at time k is evaluated by MCU at time k+1
    controlaction = data(:, 11);
end