function [x, P] = unscentedKalmanFilter(x, P, u, z, f, h, R, Q)
    % Unscented Kalman filter algorithm
    %
    % Estimates the state of a nonlinear system at time t given observations up
    % to and including time t, and inputs and system dynamics up to but not
    % including time t
    %
    % Inputs:
    % - x: the state of the system at time t-1
    % - P: the covariance matrix of the state estimate at time t-1
    % - u: the input to the system at time t
    % - z: the observation of the system at time t
    % - f: the state transition function
    % - h: the observation function
    % - R: the covariance matrix of the observation noise
    % - Q: the covariance matrix of the process noise
    %
    % Outputs:
    % - x: the state of the system at time t
    % - P: the covariance matrix of the state estimate at time t

    % define the parameters of the unscented transform
    alpha = 1e-3;
    kappa = 0;
    beta = 2;
    n = length(x);

    % compute the sigma points
    lambda = alpha^2 * (n + kappa) - n;
    Wm = [lambda / (n + lambda) 0.5 / (n + lambda) * ones(1, 2 * n)];
    Wc = [lambda / (n + lambda) + (1 - alpha^2 + beta) 0.5 / (n + lambda) * ones(1, 2 * n)];
    X = sqrtm(P) * [zeros(n, 1) eye(n) -eye(n)];
    X = [x X];

    % prediction step
    x_pred = f(X) * Wm';
    P_pred = zeros(n, n);
    for i = 1:2 * n + 1
        P_pred = P_pred + Wc(i) * (f(X(:, i)) - x_pred) * (f(X(:, i)) - x_pred)';
    end
    P_pred = P_pred + Q;

    % measurement update step
    y = z - h(x_pred);
    P_yy = zeros(length(z), length(z));
    P_xy = zeros(n, length(z));
    for i = 1:2 * n + 1
        P_yy = P_yy + Wc(i) * (h(X(:, i)) - h(x_pred)) * (h(X(:, i)) - h(x_pred))';
        P_xy = P_xy + Wc(i) * (f(X(:, i)) - x_pred) * (h(X(:, i)) - h(x_pred))';
    end
    P_yy = P_yy + R;
    K = P_xy * inv(P_yy);
    x = x_pred + K * y;
    P = P_pred - K * P_yy * K';
end