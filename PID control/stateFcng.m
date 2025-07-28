function x_next = stateFcng(x, u)
    dt = 0.01; % or parameterize this
    x_next = x + u * dt;  % simple constant acceleration model
end