function x_next = stateFcn(x, u)
    dt = 0.01; % Time step (s)

    % States
    px = x(1); 
    vx = x(2);
    angle = x(3);
    py = x(4);

    % Inputs (measured)
    ax = u(1);        % acceleration in x (from accelerometer)
    omega = u(2);     % angular rate (from gyro)
    ay = u(3);        % vertical acceleration

    % Simple dynamics
    px_next = px + vx * dt;
    vx_next = vx + ax * dt;
    angle_next = angle + omega * dt;
    py_next = py + ay * dt;

    x_next = [px_next; vx_next; angle_next; py_next];
end
