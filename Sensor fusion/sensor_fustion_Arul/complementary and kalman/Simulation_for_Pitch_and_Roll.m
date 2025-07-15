% Simple Sensor Fusion Simulation for Pitch and Roll

% Simulation time (seconds)
dt = 0.01;  % time step (100 Hz)
T = 5;     % total simulation time
time = 0:dt:T;

% True angles (simulate some motion)
true_pitch = 10 * sin(2*pi*0.1*time);  % pitch changes slowly between -10 to 10 degrees
true_roll = 5 * cos(2*pi*0.1*time);    % roll changes slowly between -5 to 5 degrees

% Simulate accelerometer reading from pitch and roll (gravity projection)
accel_x = -sin(deg2rad(true_pitch));
accel_y = sin(deg2rad(true_roll)) .* cos(deg2rad(true_pitch));
accel_z = cos(deg2rad(true_roll)) .* cos(deg2rad(true_pitch));

% Calculate pitch and roll from accel (using atan2 formulas)
pitch_acc = atan2(-accel_x, sqrt(accel_y.^2 + accel_z.^2)) * (180/pi);
roll_acc = atan2(accel_y, accel_z) * (180/pi);

% Add small noise to accel pitch and roll (simulate real-world imperfection)
pitch_acc = pitch_acc + randn(size(time))*1.5;   % ±1.5 degree noise
roll_acc  = roll_acc  + randn(size(time))*1.5;

% Simulate gyro angular velocity by differentiating true angles + some noise
gyro_pitch_rate = [0, diff(true_pitch)/dt] + randn(size(time))*0.1; % deg/s
gyro_roll_rate = [0, diff(true_roll)/dt] + randn(size(time))*0.1;   % deg/s

% Initialize fused angle estimates
pitch_fused = zeros(size(time));
roll_fused = zeros(size(time));

% Complementary filter constants
alpha = 0.95;

% Initial angles from accelerometer
pitch_fused(1) = pitch_acc(1);
roll_fused(1) = roll_acc(1);

% Apply complementary filter
for i = 2:length(time)
    % Integrate gyro rates to get angle
    pitch_gyro = pitch_fused(i-1) + gyro_pitch_rate(i)*dt;
    roll_gyro = roll_fused(i-1) + gyro_roll_rate(i)*dt;
    
    % Complementary filter combining accel and gyro
    pitch_fused(i) = alpha * pitch_gyro + (1 - alpha) * pitch_acc(i);
    roll_fused(i) = alpha * roll_gyro + (1 - alpha) * roll_acc(i);
end

% Plot results
figure;

subplot(2,1,1);
plot(time, true_pitch, 'k-', 'LineWidth', 2); hold on;
plot(time, pitch_acc, 'r--o', 'LineWidth', 1);  % Red dashed with circle marker
plot(time, pitch_fused, 'b-', 'LineWidth', 2); 
legend('True Pitch', 'Accel Pitch', 'Fused Pitch', 'Location', 'best');
xlabel('Time (s)');
ylabel('Pitch (deg)');
title('Pitch Estimation');
grid on;

subplot(2,1,2);
plot(time, true_roll, 'k-', 'LineWidth', 2); hold on;
plot(time, roll_acc, 'r--o', 'LineWidth', 1);
plot(time, roll_fused, 'b-', 'LineWidth', 2);
legend('True Roll', 'Accel Roll', 'Fused Roll', 'Location', 'best');
xlabel('Time (s)');
ylabel('Roll (deg)');
title('Roll Estimation');
grid on;
