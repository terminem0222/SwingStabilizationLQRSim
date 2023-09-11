function LQRpendulum()

% ----------- SYSTEM PARAMETERS -----------
% Define the physical parameters of the pendulum and environment.
g = 9.81;          % Acceleration due to gravity in m/s^2
l_initial = 9.144; % Initial length of the pendulum in meters
m = .1;            % Mass of the pendulum in kilograms
b = 0.015;         % Damping coefficient representing frictional effects

% ----------- STATE-SPACE REPRESENTATION -----------
% Define the state-space representation matrices for the pendulum.

% A is the system matrix:
% It defines the inherent dynamics of the system without any control.
% In state-space representation, the system's evolution is described by dx/dt = Ax + Bu.
% So, matrix A defines how each state influences the rate of change of other states.
% For the pendulum, the states are: angle, angular velocity, and length.
A = [0 1 0; 
     g/l_initial^2 0 -g/l_initial^2; 
     0 0 0];

% B is the input matrix:
% It defines how the control inputs affect the rate of change of the states.
% In our system's equation dx/dt = Ax + Bu, matrix B determines how the control inputs (u) influence the states.
B = [0; -1; 1];

% Cost matrices for the LQR controller.
Q = diag([1.31629761, 0.10001671, 3.54819301]); % State cost matrix
R = 9.58971006;                                % Control input cost matrix

% Ensure that the system is controllable with given A and B matrices.
assertControllability(A, B);

% Compute the LQR controller gain using the lqr function.
[K, ~, ~] = lqr(A, B, Q, R);

% ----------- INITIAL CONDITIONS -----------
% Define the initial conditions of the pendulum: angle, angular velocity, and length.
initial_conditions = [pi + deg2rad(30); 0; l_initial];

% ----------- SIMULATE PENDULUM -----------
% Define the time span over which the pendulum is simulated.
tspan = [0 30];  % Simulation time from 0 to 30 seconds
opts = odeset('RelTol',1e-4,'AbsTol',1e-4);  % Set options for the ODE solver to ensure accuracy.

% Simulate the pendulum dynamics without and with control.
[t_uncontrolled, x_uncontrolled] = simulatePendulum(tspan, initial_conditions, [], false, b, m, l_initial);
[t_controlled, x_controlled] = simulatePendulum(tspan, initial_conditions, K, true, b, m, l_initial);

% Plot the results of the two simulations (controlled vs uncontrolled).
plotResults(t_uncontrolled, x_uncontrolled, t_controlled, x_controlled);

end

% Helper function to check system controllability
function assertControllability(A, B)
    % Ensure that the control matrix has full rank. If not, the system is uncontrollable.
    if rank(ctrb(A,B)) < 3
        error('System is not controllable');
    end
end

function [t, x] = simulatePendulum(tspan, initial_conditions, K, useControl, b, m, l_initial)
    % Simulate the pendulum dynamics over a specified time span.

    dt = 0.01; % Define the time step for the simulation.

    % Use a fourth-order ODE solver (ode4) to simulate the system dynamics.
    [t, x] = ode4(@(t,x) pendulumDynamics(t, x, K, useControl, b, m, l_initial), tspan, initial_conditions, dt);

    % Calculate zero crossings to determine the oscillation frequency.
    zero_crossings = sum(diff(sign(x(:,1)-pi)) ~= 0);
    time_period = (tspan(2) - tspan(1)) / (zero_crossings / 2);  % Calculate period based on zero crossings.
    frequency_mHz = (1 / time_period) * 1000; % Convert the frequency to mHz.

    % Determine the maximum angular deviation during the simulation.
    max_angle_degrees = getMaxAngle(t, x);

    % Display the results for the user.
    if useControl
        fprintf('[Controlled System]\n- Oscillation Frequency: %.2f mHz\n- Max Angle Deviation: %.2f°\n\n', frequency_mHz, max_angle_degrees);
    else
        fprintf('[Uncontrolled System]\n- Oscillation Frequency: %.2f mHz\n- Max Angle Deviation: %.2f°\n\n', frequency_mHz, max_angle_degrees);
    end
end

function max_angle = getMaxAngle(t, x)
    % Determine the maximum angular deviation during a specific time window (24 to 29 seconds).

    relevant_time = (t >= 24) & (t < 29);  % Create a logical mask for the desired time range.
    max_angle_rad = max(abs(x(relevant_time, 1) - pi)); % Find the maximum deviation from the downward vertical.
    max_angle = rad2deg(max_angle_rad);  % Convert the result to degrees.
end

% Helper function to visualize the simulation results.
function plotResults(t_uncontrolled, x_uncontrolled, t_controlled, x_controlled)
    % Create a new figure for plotting.
    figure;

    % Plot the pendulum's angular displacement.
    subplot(3,1,1);
    plotData(t_uncontrolled, rad2deg(x_uncontrolled(:,1)) - 180, 'r', 'Uncontrolled', [-30 30]);
    plotData(t_controlled, rad2deg(x_controlled(:,1)) - 180, 'b', 'Controlled', [-30 30]);
    xlabel('Time (s)');
    ylabel('Theta (degrees)');
    title('Pendulum Angle');
    legend('Uncontrolled', 'Controlled', 'Location', 'northwest');  % Add a legend to the plot.

    % Plot the pendulum's angular velocity.
    subplot(3,1,2);
    plotData(t_uncontrolled, x_uncontrolled(:,2) * (180/pi), 'r', 'Uncontrolled', [-60 60]);
    plotData(t_controlled, x_controlled(:,2) * (180/pi), 'b', 'Controlled', [-60 60]);
    xlabel('Time (s)');
    ylabel('Angular Velocity (deg/s)');
    title('Angular Velocity');
    legend('Uncontrolled', 'Controlled', 'Location', 'northwest');  % Add a legend to the plot.

    % Plot the pendulum's length over time.
    subplot(3,1,3);
    plotData(t_uncontrolled, x_uncontrolled(:,3), 'r', 'Uncontrolled');
    plotData(t_controlled, x_controlled(:,3), 'b', 'Controlled');
    xlabel('Time (s)');
    ylabel('Length (m)');
    title('Pendulum Length');
    legend('Uncontrolled', 'Controlled', 'Location', 'southwest');  % Add a legend to the plot.

    % Formatting: Apply consistent appearance to all subplots.
    for i = 1:3
        subplot(3,1,i);

        ax = gca; 
        ax.YLabel.Color = 'k';  % Set the color of y-axis labels to black.
        ax.XLabel.Color = 'k';  % Set the color of x-axis labels to black.
        ax.Title.Color = 'k';   % Set the color of the title to black.

        grid on;                % Turn the grid on.
        xlim([0 30]);           % Set the x-axis limits to [0, 30] seconds.

        set(gca, 'FontName', 'Arial');  % Set the font to Arial.
    end

end

% Helper function for consistent plotting
function plotData(t, data, color, label, ylimits)
    % Plot the given data with specified parameters.
    plot(t, data, color, 'DisplayName', label);
    hold on;
    
    % If y-axis limits are provided, set them.
    if exist('ylimits','var')
        ylim(ylimits);
        % Set the y-ticks for the left y-axis
        yticks(ylimits(1):10:ylimits(2));
        % Set up a right y-axis with the same limits for visual clarity.
        yyaxis right;
        ylim(ylimits);
        yticks(ylimits(1):10:ylimits(2));
    end
    % Display the legend.
    legend;
end

function dx = pendulumDynamics(t, x, K, useControl, b, m, l_initial)
    g = 9.81;  % Acceleration due to gravity
    
    % Define the desired (target) state of the pendulum.
    desired_state = [pi; 0; l_initial];
    
    % Calculate the error between current and desired state.
    error = x - desired_state;
    
    % Adjust the error for the angle to consider downward as 0 degrees.
    error(1) = wrapToPi(x(1) - pi);  
    
    % Calculate the control input based on the state error.
    u = controlInput(error, K, useControl, x);
    
    % Calculate the dynamics based on the control input and current state.
    theta_ddot = (g / x(3)) * sin(x(1)) - b * x(2) / m - (u * x(2)) / x(3);
    omega = x(2);
    l_dot = u;

    % Return the rate of change of the system's state.
    dx = [omega; theta_ddot; l_dot];
end

function u = controlInput(error, K, useControl, x)
    if useControl
        % Calculate raw control input using LQR gain matrix.
        u_raw = -K * error;
        
        % Clip the control input to a maximum magnitude of 5.
        u = max(min(u_raw, 5), -5); 
    else
        % Default control input when not using LQR control.
        u = -.305;
    end
    
    % If the length of the pendulum is at its minimum and the control input 
    % tries to decrease it further, set control input to zero.
    if x(3) <= 0 && u < 0
        u = 0;
    % If the length of the pendulum is at its maximum and the control input 
    % tries to increase it further, set control input to zero.
    elseif x(3) >= 9.144 && u > 0
        u = 0;
    end
end

function [T, Y] = ode4(func, tspan, y0, dt)
    % Fourth-order Runge-Kutta method for solving ordinary differential equations.
    % 
    % Parameters:
    % - func: Function handle representing the system's dynamics.
    % - tspan: 2-element vector specifying the start and end simulation times.
    % - y0: Initial conditions (column vector).
    % - dt: Time step for the simulation.
    
    % Define the time points for the simulation.
    t0 = tspan(1);
    tfinal = tspan(2);
    T = (t0:dt:tfinal)';
    num_steps = length(T);

    % Initialize the output matrix with zeros.
    Y = zeros(num_steps, length(y0));
    Y(1,:) = y0';

    % Loop through each time step and calculate the state.
    for i = 1:num_steps-1
        ti = T(i);
        yi = Y(i,:)';

        % Calculate the four k-values for the Runge-Kutta method.
        k1 = func(ti, yi);
        k2 = func(ti + 0.5*dt, yi + 0.5*dt*k1);
        k3 = func(ti + 0.5*dt, yi + 0.5*dt*k2);
        k4 = func(ti + dt, yi + dt*k3);

        % Update the state using the calculated k-values.
        Y(i+1,:) = yi + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
    end
end
