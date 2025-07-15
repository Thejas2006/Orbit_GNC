clc;
clear;

% Simulation parameters
fs = 100;            % Sampling frequency (Hz)
T = 50;              % Duration (seconds)
t = linspace(0, T, fs*T);
dt = 1/fs;

% Simulated Gyroscope data (angular rate in deg/s)
gyro_pitch = 5 * sin(2 * pi * 0.1 * t);   % Pitch rate
gyro_roll  = 3 * sin(2 * pi * 0.2 * t);   % Roll rate
gyro_yaw   = 10 * sin(2 * pi * 0.05 * t); % Yaw rate

% Simulated Accelerometer data (used to estimate pitch and roll)
acc_pitch = 10 * sin(2 * pi * 0.1 * t);   % Pitch from acc
acc_roll  = 6 * sin(2 * pi * 0.2 * t);    % Roll from acc

% Simulated Magnetometer data (used to estimate yaw)
mag_yaw = 8 * sin(2 * pi * 0.05 * t);     % Yaw from mag

% Complementary filter
alpha = 0.98;

pitch_fused = zeros(size(t));
roll_fused  = zeros(size(t));
yaw_fused   = zeros(size(t));

for i = 2:length(t)
    pitch_fused(i) = alpha * (pitch_fused(i-1) + gyro_pitch(i) * dt) + (1 - alpha) * acc_pitch(i);
    roll_fused(i)  = alpha * (roll_fused(i-1)  + gyro_roll(i)  * dt) + (1 - alpha) * acc_roll(i);
    yaw_fused(i)   = alpha * (yaw_fused(i-1)   + gyro_yaw(i)   * dt) + (1 - alpha) * mag_yaw(i);
end

% 3D Cube Visualization

% Define a cube
L = 1;
cube_pts = L/2 * [
    -1 -1 -1;
    -1 -1  1;
    -1  1 -1;
    -1  1  1;
     1 -1 -1;
     1 -1  1;
     1  1 -1;
     1  1  1;
];

faces = [
    1 2 4 3;
    5 6 8 7;
    1 2 6 5;
    3 4 8 7;
    1 3 7 5;
    2 4 8 6;
];

% Create figure
figure;
axis equal;
xlim([-2 2]); ylim([-2 2]); zlim([-2 2]);
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3);
grid on;

% Draw initial cube
h = patch('Vertices', cube_pts, 'Faces', faces, ...
    'FaceColor', [0.1 0.5 1], 'FaceAlpha', 0.5);

% Animation loop
for i = 1:5:length(t)
    % Convert angles to radians
    pitch = deg2rad(pitch_fused(i));
    roll  = deg2rad(roll_fused(i));
    yaw   = deg2rad(yaw_fused(i));

    % Rotation matrices (ZYX)
    Rz = [cos(yaw) -sin(yaw) 0;
          sin(yaw)  cos(yaw) 0;
               0         0   1];
    Ry = [cos(pitch) 0 sin(pitch);
                  0   1     0;
         -sin(pitch) 0 cos(pitch)];
    Rx = [1     0           0;
          0 cos(roll) -sin(roll);
          0 sin(roll)  cos(roll)];

    % Final rotation matrix-
    R = Rz * Ry * Rx;

    % Rotate cube
    rotated_pts = (R * cube_pts')';

    % Update cube
    set(h, 'Vertices', rotated_pts);
    title(sprintf("Time: %.2f sec", t(i)));
    drawnow;
end
