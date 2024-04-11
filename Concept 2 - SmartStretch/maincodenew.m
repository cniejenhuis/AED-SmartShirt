% Establish a Connection to the Arduino
arduinoObj = serialport("COM3", 9600);

% Prepare the serialport Object
configureTerminator(arduinoObj, "CR/LF");
flush(arduinoObj);
arduinoObj.UserData = struct("Data", [], "Count", 1);
configureCallback(arduinoObj, "terminator", @readSensorData);

% Set up Arduino to read analog input
a = arduino();
sensorPin = 'A1';
Resistor = 10000.0;
calibrateMaxButtonPin = 7;
calibrateMinButtonPin = 3;
maxVoltage = 0.0;
minVoltage = 5.0;
maxButtonPressed = false;
minButtonPressed = false;
configurePin(a, calibrateMaxButtonPin, 'pullup');
configurePin(a, calibrateMinButtonPin, 'pullup');

% Initialize variables for plotting
time = [];
angles = [];

% Create figure and axes for plotting
figure;
plotHandle = plot(0, 0);
title('Real-time Angle Plot');
xlabel('Time (s)');
ylabel('Angle (degrees)');
ylim([0, 300]);

% Start timer for 10 seconds
t = timer('ExecutionMode', 'fixedRate', 'Period', 0.1, 'TimerFcn', @updatePlot, 'StopTime', 10);
start(t);

% Callback function to update plot
function updatePlot(~, ~)
    % Read angle value from Arduino
    sensorValue = readVoltage(a, sensorPin) * (5.0 / 1023.0);
    
    % Check for max and min calibration button press
    if readDigitalPin(a, calibrateMaxButtonPin) == 1 && ~maxButtonPressed
        maxVoltage = sensorValue;
        maxButtonPressed = true;
        pause(1); % Debouncing delay
    elseif readDigitalPin(a, calibrateMaxButtonPin) == 0
        maxButtonPressed = false;
    end
    
    if readDigitalPin(a, calibrateMinButtonPin) == 1 && ~minButtonPressed
        minVoltage = sensorValue;
        minButtonPressed = true;
        pause(1); % Debouncing delay
    elseif readDigitalPin(a, calibrateMinButtonPin) == 0
        minButtonPressed = false;
    end
    
    % Calculate the angle based on calibrated min and max voltages
    angle = map(sensorValue, minVoltage / (5.0 / 1023.0), maxVoltage / (5.0 / 1023.0), 0, 300);
    
    % Update time and angles arrays
    nonlocal time angles
    time = [time, toc];
    angles = [angles, angle];
    
    % Update plot
    set(plotHandle, 'XData', time, 'YData', angles);
    drawnow;
end

% Callback function to read streaming data from Arduino
function readSensorData(src, ~)
    % Read the ASCII data from the serialport object
    data = readline(src);
    
    % Convert the string data to numeric
    % type and save it in the UserData
    % property of the serialport object
    src.UserData.Data(end+1) = str2double(data);
    
    % Update the Count value of the serialport object
    src.UserData.Count = src.UserData.Count + 1;
end
