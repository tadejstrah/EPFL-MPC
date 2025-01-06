classdef MpcControl_lon < MpcControlBase
    
    methods
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   x0           - initial state (estimate)
            %   V_ref, u_ref - reference state/input
            %   d_est        - disturbance estimate
            %   x0other      - initial state of other car
            % OUTPUTS
            %   u0           - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N_segs = ceil(mpc.H/mpc.Ts); % Horizon steps
            N = N_segs + 1;              % Last index in 1-based Matlab indexing

            [nx, nu] = size(mpc.B);
            
            % Targets
            V_ref = sdpvar(1);
            u_ref = sdpvar(1);

            % Disturbance estimate (Ignore this before Todo 4.1)
            d_est = sdpvar(1);

            % Initial states
            x0 = sdpvar(nx, 1);
            x0other = sdpvar(nx, 1); % (Ignore this before Todo 5.1)

            % Input to apply to the system
            u0 = sdpvar(nu, 1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D
            %       are the DISCRETE-TIME MODEL of your system.
            %       You can find the linearization steady-state
            %       in mpc.xs and mpc.us.
            
            % Define weights for tracking objectives
            x_error_penalty = 0; % Weight of position error (not penalized in longitudinal control)
            speed_error_penalty = 2; % Weight of velocity error
            Q_tracking_mat = diag([x_error_penalty, speed_error_penalty]);
            R_tracking_mat = 1; % Weight of throttle effort
            
            input_constraint = [1; 1]; % Throttle constraints: -100% ≤ u ≤ 100%
            input_constraint_sign = [1; -1]; %
            
            % No LQR for terminal cost anymore, using Q_tracking_mat directly
            Q_terminal_mat = Q_tracking_mat; % Terminal cost matrix (same as tracking cost)
            R_terminal_mat = R_tracking_mat; % Input cost matrix (same as tracking cost)
            
            % Initialize optimization variables
            state_trajectory = sdpvar(nx, N); % State trajectory prediction [position; velocity]
            control_input_trajectory = sdpvar(nu, N-1); % Control/input trajectory prediction [throttle]
            
            % Initial constraints and terminal cost
            initial_state_deviation = x0 - mpc.xs;
            constraint = (state_trajectory(:,1) == initial_state_deviation);
            
            % Terminal cost (using Q_terminal_mat directly)
            error_state = state_trajectory(:,N) + mpc.xs - [0; V_ref];
            objective = error_state' * Q_terminal_mat * error_state; % Terminal cost
            
            % Iterate over prediction horizon
            for horizonindex = 1:N-1
                % Error in state and input
                error_input = control_input_trajectory(:,horizonindex) + mpc.us - u_ref;
                error_state = state_trajectory(:,horizonindex) + mpc.xs - [0; V_ref];
                
                % Dynamics, state, and input constraints
                constraint = constraint ...
                             + (state_trajectory(:,horizonindex+1) == mpc.B * control_input_trajectory(:,horizonindex) + mpc.A * state_trajectory(:,horizonindex)) ...
                             + (input_constraint_sign * (control_input_trajectory(:,horizonindex) + mpc.us) <= input_constraint);
                
                % Objective function (tracking cost)
                objective = objective ...
                           + error_state' * Q_tracking_mat * error_state ...
                           + error_input' * R_tracking_mat * error_input;
            end
            
            % Extract the first input (convert to absolute coordinates)
            constraint = constraint + (u0 == control_input_trajectory(:,1) + mpc.us);
            
            % Debugging variables
            debugVars = {state_trajectory, control_input_trajectory};
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(constraint, objective, sdpsettings('solver','gurobi'), ...
                {x0, V_ref, u_ref, d_est, x0other}, {u0, debugVars{:}});
        end
        
        % Computes the steady state target which is passed to the
        % controller
        function [Vs_ref, us_ref] = compute_steady_state_target(mpc, ref, d_est)

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            %   d_est  - disturbance estimate (Ignore before Todo 4.1)
            % OUTPUTS
            %   Vs_ref, us_ref - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Steady-state subsystem
            A = mpc.A(2, 2);
            B = mpc.B(2, 1);

            % Subsystem linearization steady-state
            xs = mpc.xs(2);
            us = mpc.us;

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE

            % Store the target velocity as our reference
            % This is the absolute velocity we want to achieve
            Vs_ref = ref;
            % compute steady state throttle and sensure its within bound.
            us_ref = min(max(us - (A*(Vs_ref-xs))/B, -1),1);
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end
