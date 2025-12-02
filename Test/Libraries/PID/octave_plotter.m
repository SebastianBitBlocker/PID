clc
fid = fopen('Test/Libraries/PID/Export/pid_control_response.dat', 'r');
if (fid == -1)
  error('Could not open file.');
end

% Read the header line (optional, but good practice)
header = fgetl(fid);
disp(header); % Display the header

% Read the numerical data, specifying the format as floating-point numbers
data = textscan(fid, '%f%f%f%f', 'Delimiter', '\t');

fclose(fid);

% Assign the data to separate variables
time = data{1};
reference = data{2};
control_signal = data{3};
plant_output = data{4};

% Now you can proceed with plotting as described before
plot(time, reference, 'b-', 'linewidth', 1.5, 'DisplayName', 'Reference');
hold on;
plot(time, plant_output, 'r-', 'linewidth', 1.5, 'DisplayName', 'Plant Output');
plot(time, control_signal, 'g--', 'linewidth', 1.5, 'DisplayName', 'Control Signal');

xlabel('Time');
ylabel('Amplitude');
title('PID Control System Response');
legend('show');
grid on;
