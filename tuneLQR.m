% Function to optimize LQR parameters using a genetic algorithm.
function bestParams = tuneLQR()
    % Define system constants.
    g = 9.81;                 % Acceleration due to gravity
    l_initial = 9.144;        % Initial length of the pendulum
    m = .1;                   % Mass of the pendulum
    b = 0.015;                % Damping coefficient
    
    % State-space representation of the linearized system.
    A = [0 1 0; 
         g/l_initial^2 0 -g/l_initial^2; 
         0 0 0];
    B = [0; -1; 1];
    
    % Define simulation parameters.
    tspan = [0 30];
    initial_conditions = [pi + deg2rad(30); 0; l_initial];
    
    % Genetic algorithm settings.
    options = optimoptions('ga', 'MaxGenerations', 100, 'Display', 'off', 'PlotFcn', @gaplotbestf);

    % Parameter bounds for optimization.
    lb = [0.1, 0.1, 0.1, 0.1];
    ub = [10, 10, 10, 10];

    % Use the genetic algorithm to find the best LQR parameters.
    [bestParams, ~] = ga(@(params) objectiveFunction(params, A, B, tspan, initial_conditions, b, m, l_initial), 4, [], [], [], [], lb, ub, [], options);
    
    % Print the optimized parameters.
    fprintf('Q = diag([%.8f, %.8f, %.8f]);\n', bestParams(1), bestParams(2), bestParams(3));
    fprintf('R = %.8f;\n', bestParams(4));
end

% Objective function to evaluate the performance of given LQR parameters.
function score = objectiveFunction(params, A, B, tspan, initial_conditions, b, m, l_initial)
    % Extract the Q and R parameters for the LQR.
    Q_diag = params(1:3);
    R = params(4);
    Q = diag(Q_diag);
    
    % Compute the LQR gain matrix.
    [K, ~, ~] = lqr(A, B, Q, R);
    
    % Simulate the controlled system.
    [~, x_controlled] = simulatePendulum(tspan, initial_conditions, K, true, b, m, l_initial);

    % Compute relevant metrics for performance evaluation.
    finalCableLength = x_controlled(end, 3);
    maxAngle = max(abs(rad2deg(x_controlled(:,1)) - 180));
    maxAngularVelocity = max(abs(x_controlled(:,2) * (180/pi)));

    % Calculate the score based on the system performance.
    score = 0;
    if finalCableLength > 0
        score = score + finalCableLength;
    end
    if maxAngle > 5
        score = score + (maxAngle - 5)*10;
    end
    if maxAngularVelocity > 25  
        score = score + (maxAngularVelocity - 30) * 10;  
    end
end

% Simulate the pendulum dynamics.
function [t, x] = simulatePendulum(tspan, initial_conditions, K, useControl, b, m, l_initial)
    dt = 0.01; % Simulation time step
    [t, x] = ode4(@(t,x) pendulumDynamics(t, x, K, useControl, b, m, l_initial), tspan, initial_conditions, dt);
end

% Dynamics of the pendulum.
function dx = pendulumDynamics(t, x, K, useControl, b, m, l_initial)
    g = 9.81;  % Acceleration due to gravity
    
    % Define desired (reference) state.
    desired_state = [pi; 0; l_initial];
    
    % Calculate state error.
    error = x - desired_state;
    error(1) = wrapToPi(x(1) - pi);  
    
    % Determine control input.
    u = controlInput(error, K, useControl, x);
    
    % Compute the state derivatives.
    theta_ddot = (g / x(3)) * sin(x(1)) - b * x(2) / m - (u * x(2)) / x(3);
    omega = x(2);
    l_dot = u;

    dx = [omega; theta_ddot; l_dot];
end

% Determine control input based on error and system limits.
function u = controlInput(error, K, useControl, x)
    if useControl
        u_raw = -K * error;
        u = max(min(u_raw, 5), -5); 
    else
        u = -.305;
    end
    
    % Ensure the control respects system constraints.
    if x(3) <= 0 && u < 0
        u = 0;
    elseif x(3) >= 9.144 && u > 0
        u = 0;
    end
end

% Fourth-order Runge-Kutta method (ODE4) for solving ordinary differential equations.
% Inputs:
%   - func: Function handle specifying the system dynamics (e.g., dy/dt = func(t,y))
%   - tspan: 2-element vector specifying the start and end times
%   - y0: Initial conditions (column vector)
%   - dt: Time step size
% Outputs:
%   - T: Time vector
%   - Y: Solution matrix with each row being the system state at corresponding time
function [T, Y] = ode4(func, tspan, y0, dt)
    % Initialize time vector from tspan(1) to tspan(2) with step size dt
    t0 = tspan(1);
    tfinal = tspan(2);
    T = (t0:dt:tfinal)';  % Transposed to make it a column vector
    num_steps = length(T);

    % Initialize solution matrix Y
    Y = zeros(num_steps, length(y0));
    Y(1,:) = y0';  % Set the first row to the initial conditions

    % Iterate through each time step to compute the solution
    for i = 1:num_steps-1
        ti = T(i);             % Current time step
        yi = Y(i,:)';          % Current state

        % Compute the four increments (k1, k2, k3, k4) for the ODE4 method
        k1 = func(ti, yi);
        k2 = func(ti + 0.5*dt, yi + 0.5*dt*k1);
        k3 = func(ti + 0.5*dt, yi + 0.5*dt*k2);
        k4 = func(ti + dt, yi + dt*k3);

        % Update the next state based on the increments
        Y(i+1,:) = yi + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
    end
end
