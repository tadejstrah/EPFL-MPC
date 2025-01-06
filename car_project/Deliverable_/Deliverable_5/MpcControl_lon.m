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
            
            %% Load pre-computed tube MPC components for robust control
            % Get the folder where the script is located
            scriptFolder = fileparts(mfilename('fullpath'));
            
            % Construct the full path for the .mat file
            loadPath = fullfile(scriptFolder, 'tube_mpc_data.mat');
            
            % Load the data from the specified path
            data = load(loadPath);
            
            % Extract data from the loaded file
            xsafe = [data.safe_position_margin; 0];
            
            % restore data from the matrix
            Q_tracking_mat = data.tracking_weights;    
            R_tracking_mat = data.control_penalty;     
            
            % Initialize optimization variables
            X = sdpvar(nx, N);     % Nominal state trajectory [position; velocity]
            U = sdpvar(nu, N-1);   % Nominal input trajectory [acceleration]
            
            % Initialize optimization problem
            objective = X(:,N)'*data.terminal_weights*X(:,N);    % Terminal cost
            constraint = [];   % Constraint list
 
            % convert to delta
            constraint = constraint + (X(:,1) == x0other - xsafe - x0);
            
            % Iterate over prediction horizon
            for horizonindex = 1:N-1
                % Dynamics, state, and input constraints
                constraint = constraint + (X(:,horizonindex+1) == mpc.A*X(:,horizonindex) - mpc.B*U(:,horizonindex)) ...
                                        + (data.state_constraints_tightened.A*X(:,horizonindex) <= data.state_constraints_tightened.b) ... % State constraints
                                        + (data.input_constraints_tightened.A*U(:,horizonindex) <= data.input_constraints_tightened.b);    % Input constraints
                % Objective function (tracking cost)
                objective = objective + X(:,horizonindex)'*Q_tracking_mat*X(:,horizonindex) + U(:,horizonindex)'*R_tracking_mat*U(:,horizonindex);
            end
                        
            % Terminal set constraint for recursive feasibility
            constraint = constraint + (data.terminal_set.A*X(:,N) <= data.terminal_set.b);
            
            constraint = constraint + (u0 == data.terminal_lqr_gain*(x0other - xsafe - x0 - X(:,1)) + U(:,1));  % Tube controller
            
            % Store variables for debugging
            debugVars = {X, U};

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
            % and the control action come from the tube so we dont use this
            us_ref = 0;
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end
