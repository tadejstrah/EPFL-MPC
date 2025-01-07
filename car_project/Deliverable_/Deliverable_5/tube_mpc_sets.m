clearvars;
close all;
clc;

%% Vehicle Dynamics Initialization
% Set up a model to control the longitudinal motion of a car. The goal is to
% maintain a safe distance from a leading vehicle while tracking a desired speed.

sampling_period = 0.1; % Sampling interval (10 Hz)
vehicle = Car(sampling_period); % Create the vehicle model

target_speed_mps = 120 / 3.6; % Convert target speed from km/h to m/s

% Determine steady-state values for the desired speed
[equilibrium_state, equilibrium_input] = vehicle.steady_state(target_speed_mps);
linear_system = vehicle.linearize(equilibrium_state, equilibrium_input);
[longitudinal_dynamics, ~] = vehicle.decompose(linear_system);

discrete_dynamics = c2d(longitudinal_dynamics, sampling_period); % Discretize the system
[A_matrix, B_matrix, C_matrix, D_matrix] = ssdata(discrete_dynamics); % Extract system matrices

% Adjust B matrix to reflect relative motion dynamics
B_matrix = -B_matrix;

%% Design of Reference Tracking Controller
% Construct an LQR controller to maintain the desired position and speed.

tracking_weights = diag([25, 50]); % Penalize position and velocity errors
control_penalty = 2; % Penalize control effort

% Compute LQR gains for discrete-time system
lqr_gain = -dlqr(A_matrix, B_matrix, tracking_weights, control_penalty);
closed_loop_matrix = A_matrix + B_matrix * lqr_gain;

%% Compute Invariant Set for Disturbance Rejection
% Determine the smallest set bounding the effects of external disturbances.
nominal_throttle = equilibrium_input(2);
throttle_variation = Polyhedron('lb', nominal_throttle - 0.5, 'ub', nominal_throttle + 0.5);
disturbance_set = -B_matrix * throttle_variation; % Map disturbance to state space

% Visualization setup
figure('Name', 'Convergence of Invariant Set');
hold on; grid on; axis equal;

% Initialize the invariant set at the origin
invariant_sets{1} = Polyhedron('lb', [0; 0], 'ub', [0; 0]);

tolerance = 1e-3; % Convergence threshold
iteration = 1;
max_iterations = 100;
colors = lines(max_iterations); % Use line color scheme for visualization

while true
    % Compute the next set in the sequence
    invariant_sets{iteration + 1} = invariant_sets{iteration} + closed_loop_matrix^iteration * disturbance_set;
    invariant_sets{iteration + 1}.minHRep();

    % Plot the current set
    plot(invariant_sets{iteration + 1}, 'Color', colors(iteration, :), 'Alpha', 0.3);
    title(sprintf('Invariant Set After %d Iterations', iteration));

    % Check for convergence
    if norm(closed_loop_matrix^iteration, 'fro') < tolerance
        fprintf('Invariant set converged after %d iterations.\n', iteration);
        break;
    end

    % Update iteration counter
    iteration = iteration + 1;
    if iteration > max_iterations
        warning('Maximum iterations reached without convergence.');
        break;
    end
    drawnow;
end

final_invariant_set = invariant_sets{end};
final_invariant_set.minHRep();

%% Define System Constraints
% Establish constraints for safe operation of the vehicle.

safe_position_margin = 8; % Safety buffer distance (meters)
minimum_gap = 6; % Minimum acceptable distance (meters)

% State constraints (e.g., relative position >= minimum_gap)
state_constraints = Polyhedron('A', [-1, 0], 'b', -(minimum_gap - safe_position_margin));

% Control input constraints (e.g., throttle limits)
input_min = -1; % Maximum braking
input_max = 1; % Maximum acceleration
input_constraints = Polyhedron('lb', input_min, 'ub', input_max);

% Tighten constraints by accounting for the invariant set
state_constraints_tightened = state_constraints - final_invariant_set;
state_constraints_tightened.minHRep();

control_effect = lqr_gain * final_invariant_set;
input_constraints_tightened = input_constraints - control_effect;
input_constraints_tightened.minHRep();

%% Terminal Controller Design
% Create a conservative controller for use in the terminal set definition.

terminal_weights = tracking_weights / 2; % Reduce weights for terminal control
input_penalty_terminal = control_penalty * 2; % Increase penalty on control effort

[terminal_lqr_gain, terminal_cost] = dlqr(A_matrix, B_matrix, terminal_weights, input_penalty_terminal);
terminal_lqr_gain = -terminal_lqr_gain;
terminal_closed_loop_matrix = A_matrix + B_matrix * terminal_lqr_gain;

%% Compute Terminal Set for Stability
% Define the terminal set to ensure MPC stability.

[Fx, fx] = deal(state_constraints_tightened.A, state_constraints_tightened.b);
[Fu, fu] = deal(input_constraints_tightened.A, input_constraints_tightened.b);

combined_constraints = [Fx; Fu * terminal_lqr_gain];
combined_bounds = [fx; fu];

terminal_set = Polyhedron('A', combined_constraints, 'b', combined_bounds);

iteration = 0;
while iteration < max_iterations
    previous_set = terminal_set;
    pre_set = Polyhedron('A', [combined_constraints; combined_constraints * terminal_closed_loop_matrix], ...
                         'b', [combined_bounds; combined_bounds]);
    pre_set.minHRep();
    terminal_set = intersect(terminal_set, pre_set);
    terminal_set.minHRep();

    if terminal_set == previous_set
        fprintf('Terminal set computation converged after %d iterations.\n', iteration);
        break;
    end

    iteration = iteration + 1;
    if iteration == max_iterations
        warning('Terminal set computation did not converge.');
    end
end

if terminal_set.isEmptySet()
    error('Terminal set is empty. Adjust safety margins or controller weights.');
end

%% Visualization of Key Results
% Plot the constraints and sets for verification.

figure('Name', 'Constraint Tightening Comparison');
hold on;
plot(input_constraints, 'alpha', 0.3, 'color', 'blue', 'DisplayName', 'Original Input Constraints');
plot(input_constraints_tightened, 'alpha', 0.3, 'color', 'red', 'DisplayName', 'Tightened Input Constraints');
legend('Location', 'best');
title('Tightened Constraints for Robust Control');
xlabel('Throttle Input');

figure('Name', 'Final Terminal Set');
hold on;
plot(terminal_set, 'alpha', 0.3, 'color', 'red', 'DisplayName', 'Terminal Set');
title('Computed Terminal Set for Stability');
xlabel('Position Error (m)');
ylabel('Velocity Error (m/s)');
grid on;

%% Save Results for MPC
% Get the folder where the script is located
scriptFolder = fileparts(mfilename('fullpath'));

% Construct the full path for the .mat file
savePath = fullfile(scriptFolder, 'tube_mpc_data.mat');

% Save the data to the constructed path
save(savePath, ...
    'lqr_gain', 'terminal_lqr_gain', 'terminal_cost', ...
    'tracking_weights', 'control_penalty', ...
    'terminal_weights', 'input_penalty_terminal', ...
    'final_invariant_set', 'terminal_set', ...
    'state_constraints_tightened', 'input_constraints_tightened', ...
    'minimum_gap', 'safe_position_margin');
