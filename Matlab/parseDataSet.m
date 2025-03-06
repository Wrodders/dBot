function [timestamps, timedelta, sensorData] = parseDataSet(csvFile)
T = readtable(csvFile, VariableNamingRule="preserve");

% Check that there are at least two columns
if width(T) < 2
    error('CSV file must contain a timestamp column and at least one sensor data column.');
end

%% 2. Process Timestamps and Sensor Data
timestamps = T{:,1};      % numeric timestamps as strings or numbers
sensorData = T{:,2:end};  % sensor measurements

for i=1:1:length(timestamps)-1  
    timedelta(i) = timestamps(i+1) - timestamps(i); 
end 
end