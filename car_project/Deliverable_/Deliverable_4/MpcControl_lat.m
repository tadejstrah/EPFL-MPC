classdef MpcControl_lat < MpcControlBase
    
    methods
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   x0           - initial state (estimate)
            %   x_ref, u_ref - reference state/input
            % OUTPUTS
            %   u0           - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N_segs = ceil(mpc.H/mpc.Ts); % Horizon steps
            N = N_segs + 1;              % Last index in 1-based Matlab indexing

            [nx, nu] = size(mpc.B);
            
            % Targets
            x_ref = sdpvar(nx, 1);
            u_ref = sdpvar(nu, 1);

            % Initial states
            x0 = sdpvar(nx, 1);
            x0other = sdpvar(nx, 1); % (Ignore this, not used)

            % Input to apply to the system
            u0 = sdpvar(nu, 1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D
            %       are the DISCRETE-TIME MODEL of your system
            %       You can find the linearization steady-state
            %       in mpc.xs and mpc.us.
            
            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            y_error_penalty = 1 ;% weight of lateral position error
            theta_error_penalty = 1 ;% weight of heading angle error
            Q_tracking_mat = diag([y_error_penalty, theta_error_penalty]);
            R_tracking_mat = 1; % weight of steering effort
            state_constraint = [3.5;          % Maximum allowed y
                                0.5;          % Minimum allowed y (-)
                                5*pi/180;     % Maximum allowed theta
                                5*pi/180];    % Minimum allowed theta (-)
            state_constraint_sign = [1  0;    % 
                                    -1  0;    %
                                     0  1;    % 
                                     0 -1];   % 
            input_constraint = [30*pi/180;    % Max steering angle δ ≤ 30°
                                30*pi/180];   % Min steering angle (-)
            input_constraint_sign = [1; -1];  % 
            
            % LQR to compute terminal cost
            Q_terminal_mat = Q_tracking_mat;
            R_terminal_mat = R_tracking_mat; % tune me if needed
            [state_feedback_gain_mat, terminal_cost_mat, ~] = dlqr(mpc.A, mpc.B, Q_terminal_mat, R_terminal_mat);
            % closed loop dynamic
            A_closed_loop = mpc.A - mpc.B*state_feedback_gain_mat;
            % terminal set
            terminal_set = polytope([state_constraint_sign; -input_constraint_sign*state_feedback_gain_mat], [state_constraint; input_constraint]);

            % compute the maximal invariant set Iteratively as explained in
            % the course
            previous_terminal_set = terminal_set;
            max_iterations = 1000; % Set a limit to prevent infinite loop
            
            for i = 1:max_iterations
                [constraint_sign, constraint] = double(previous_terminal_set);
                pre_terminal_set = polytope(constraint_sign * A_closed_loop, constraint);  % One-step backward reachable set
                terminal_set = intersect(previous_terminal_set, pre_terminal_set);  % Intersect with current set
            
                % Check that we reached convergence
                if isequal(previous_terminal_set, terminal_set)
                    break;
                end
                previous_terminal_set = terminal_set;
            end
            [terminal_set_sign, terminal_set_constraint] = double(terminal_set);
            
            figure
            hold on; grid on;
            plot(polytope(state_constraint_sign,state_constraint), 'c');
            plot(terminal_set, 'lime');
            xlabel('lateral position [m]');
            ylabel('steering angle [rad]');

            state_trajectory = sdpvar(nx, N);     % State trajectory [y; theta]
            control_input_trajectory = sdpvar(nu, N-1);   % Input trajectory [steering_angle]
            % ensure the solver dont "cheat"
            initial_state_deviation = x0 - mpc.xs;
            constraint = (state_trajectory(:,1) == initial_state_deviation);
            constraint = constraint + (terminal_set_sign*(state_trajectory(:,N) + mpc.xs) <= terminal_set_constraint);  % add the terminal cost
            error_state = state_trajectory(:,N) + mpc.xs - x_ref ;
            objectif = error_state'*terminal_cost_mat*error_state;    % Terminal cost from LQR
            % Itterrate over prediction horizon
            for horizonindex = 1:N-1
                error_input = control_input_trajectory(:,horizonindex) + mpc.us - u_ref;
                error_state = state_trajectory(:,horizonindex) + mpc.xs - x_ref;
                % Dynamic + State + Input constraints
                constraint = constraint ...
                               + (state_trajectory(:,horizonindex+1) == mpc.B*control_input_trajectory(:,horizonindex) + mpc.A*state_trajectory(:,horizonindex)) ...
                               + (state_constraint_sign*(state_trajectory(:,horizonindex) + mpc.xs) <= state_constraint) ...
                               + (input_constraint_sign*(control_input_trajectory(:,horizonindex) + mpc.us) <= input_constraint);
                objectif = objectif + error_state'*Q_tracking_mat*error_state + error_input'*R_tracking_mat*error_input;
            end
           
            
            % Extract first input (convert back to absolute coordinates)
            constraint = constraint + (u0 == control_input_trajectory(:,1) + mpc.us);
            
            % Store debugging variables
            debugVars = {state_trajectory, control_input_trajectory};

            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(constraint, objectif, sdpsettings('solver','gurobi'), ...
                {x0, x_ref, u_ref, x0other}, {u0, debugVars{:}});
        end
        
        % Computes the steady state target which is passed to the
        % controller
        function [xs_ref, us_ref] = compute_steady_state_target(mpc, ref)

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            % OUTPUTS
            %   xs_ref, us_ref - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % Steady-state system
            A = mpc.A;
            B = mpc.B;

            % Linearization steady-state
            xs = mpc.xs;
            us = mpc.us;

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE

            % Set reference state (xs_ref = (y, theta)):
            % desired lateral position (y=ref) and zero heading angle (theta=0).
            % This ensures the car is parallel to the road at the target position.
            xs_ref = [ref; 0];
            
            % Set reference input (us_ref) for zero steering angle at steady state.
            % This ensures straight driving when the target is reached with zero heading angle.
            us_ref = 0;
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end
