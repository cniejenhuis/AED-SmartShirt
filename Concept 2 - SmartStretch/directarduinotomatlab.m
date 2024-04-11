% Create Arduino object
a = arduino('COM3', 'Uno'); % Change COM3 to the port your Arduino is connected to

% Initialize variables
numSamples = 1000; % Adjust as needed
data = zeros(1, numSamples);

% Read data from Arduino
for i = 1:numSamples
    data(i) = readVoltage(a, 'A0'); % Read from analog pin A0
    pause(0.1); % Adjust pause as needed
end

% Plot data
plot(data);
xlabel('Sample');
ylabel('Voltage');
title('Arduino Data');
