function y = measFcn(x)
    vx = x(2);
    angle = x(3);
    y = [vx; angle];  % Only measuring velocity and angle
end
