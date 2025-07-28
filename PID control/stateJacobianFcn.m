function F = stateJacobianFcn(x, u)
    % Jacobian of the state function w.r.t. state x
    n = numel(x);
    F = eye(n);  % Since x only adds gyro*dt
end