function A = stateJacobianFcn(x, u)
    A = eye(3);  % df/dx is identity if linear motion model
end