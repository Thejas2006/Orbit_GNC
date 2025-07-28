function H = measJacobianFcn(x)
    % Measurement is directly the state, so Jacobian is identity
    n = numel(x);
    H = eye(n);
end