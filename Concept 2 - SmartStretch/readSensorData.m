function readSensorData(src, ~)
    % Read the ASCII data from the serialport object
    data = readline(src);
    
    % Convert the string data to numeric type and save it in the UserData
    % property of the serialport object
    src.UserData.Data(end+1) = str2double(data);
    
    % Update the Count value of the serialport object
    src.UserData.Count = src.UserData.Count + 1;
    
    % Display the data
    disp(data);
    
    % If desired number of data points have been collected from the Arduino,
    % switch off the callbacks
    if src.UserData.Count > 1001
        configureCallback(src, "off");
    end
end
