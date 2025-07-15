clc; clear; close all;

% Simulated time
dt = 0.01;
time = 0:dt:10;

% Simulated true motion (degrees)
true_pitch = 10 * sin(2*pi*0.1*time);     % oscillating pitch
true_roll  = 5 * cos(2*pi*0.05*time);     % oscillating roll
true_yaw   = 45 * sin(2*pi*0.02*time);    % slow yaw turn

% Simulate gyroscope data (derivatives + noise)
gyro_pitch_rate = [0, diff(true_pitch)/dt] + randn(size(time))*0.5;
gyro_roll_rate  = [0, diff(true_roll)/dt] + randn(size(time))*0.5;
gyro_yaw_rate   = [0, diff(true_yaw)/dt] + randn(size(time))*1.5;

% Simulate accelerometer-based pitch and roll
pitch_acc = true_pitch + randn(size(time))*1.5;
roll_acc  = true_roll + randn(size(time))*1.5;

% Simulate magnetometer readings
mag_x = cosd(true_yaw) + randn(size(time))*0.05;
mag_y = -sind(true_yaw) + randn(size(time))*0.05;
yaw_mag = atan2(-mag_y, mag_x) * (180/pi);  % in degrees
yaw_mag = mod(yaw_mag + 180, 360) - 180;

% Initialize fusion variables
pitch_fused = zeros(size(time));
roll_fused  = zeros(size(time));
yaw_fused   = zeros(size(time));

% Initial values
pitch_fused(1) = pitch_acc(1);
roll_fused(1)  = roll_acc(1);
yaw_fused(1)   = yaw_mag(1);

% Complementary filter alpha
alpha = 0.95;

% Fusion loop
for i = 2:length(time)
    % Gyro integration
    pitch_gyro = pitch_fused(i-1) + gyro_pitch_rate(i) * dt;
    roll_gyro  = roll_fused(i-1)  + gyro_roll_rate(i)  * dt;
    yaw_gyro   = yaw_fused(i-1)   + gyro_yaw_rate(i)   * dt;

    % Wrap yaw_gyro between -180 to 180
    yaw_gyro = mod(yaw_gyro + 180, 360) - 180;

    % Complementary filter
    pitch_fused(i) = alpha * pitch_gyro + (1 - alpha) * pitch_acc(i);
    roll_fused(i)  = alpha * roll_gyro  + (1 - alpha) * roll_acc(i);
    yaw_fused(i)   = alpha * yaw_gyro   + (1 - alpha) * yaw_mag(i);
end

% --- Plot Results ---

figure;
subplot(3,1,1);
plot(time, true_pitch, 'k-', time, pitch_acc, 'r--', time, pitch_fused, 'b-', 'LineWidth', 1.5);
legend('True Pitch', 'Accel Pitch', 'Fused Pitch'); ylabel('Pitch (°)'); grid on;

subplot(3,1,2);
plot(time, true_roll, 'k-', time, roll_acc, 'r--', time, roll_fused, 'b-', 'LineWidth', 1.5);
legend('True Roll', 'Accel Roll', 'Fused Roll'); ylabel('Roll (°)'); grid on;

subplot(3,1,3);
plot(time, true_yaw, 'k-', time, yaw_mag, 'r--', time, yaw_fused, 'b-', 'LineWidth', 1.5);
legend('True Yaw', 'Mag Yaw', 'Fused Yaw'); ylabel('Yaw (°)'); xlabel('Time (s)'); grid on;




