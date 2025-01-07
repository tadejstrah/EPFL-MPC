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

            x = sdpvar(nx, N);
            u = sdpvar(nu, N-1);

            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D
            %       are the DISCRETE-TIME MODEL of your system.
            %       You can find the linearization steady-state
            %       in mpc.xs and mpc.us.
            


            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            % obj = 0;
            % con = [];


            % % Weight matrices for cost function
            Q = diag([1, 10]);            % Penalize velocity deviation heavily
            R = 10;                        % Penalize throttle usage
       
            A = mpc.A;
            B = mpc.B;

            [~,Qf,~] = dlqr(A,B,Q,R);

            
            % K = -K; 
            
            % Qf = dlyap(A,Q);

            % con = [];
            % obj = 0;

            x_ref = [0; V_ref];
            % U_ref = u_ref;
            u_min = -1;                   % Minimum throttle
            u_max = 1;  
 
            obj = 0;
            con = [];
           
             % Initial state constraint
            con = [con, x(:, 1) == x0];

            % Add constraints and objective for all prediction steps
            for k = 1:N_segs
                % Dynamics constraint
                con = [con, x(:, k+1) == mpc.A * x(:, k) + mpc.B * u(:, k)];
                % State constraints
                % con = [con, F * x(:, k) <= f];
                % Input constraints
                con = con + (u_min <= u(:,1));
                con = con + (u(:,1) <= u_max);
                % con = [con, M * u(:, k) <= m];
                % Add stage cost to objective
                obj = obj + (x(:, k) - x_ref)' * Q * (x(:, k) - x_ref) + ...
                            (u(:, k) - u_ref)' * R * (u(:, k) - u_ref);
            end

            % Add terminal state cost
            obj = obj + (x(:, N) - x_ref)' * Qf * (x(:, N) - x_ref);
            % Add terminal constraint to enforce terminal set
            % con = [con, Ff * x(:, N) <= ff];




            % 
            % con = [con, X(:, 1) == x0];
            % 
            % con = con + (X(:,2) == A*X(:,1) + B*U(:,1)) ;
            % con = con + (u_min <= U(:,1));
            % con = con + (U(:,1) <= u_max);
            % obj = U(:,1)'*R*U(:,1);
            % 
            % for i = 2:N-1
            %      con = con + (X(:,i+1) == A*X(:,i) + B*U(:,i));
            %      con = con + (u_min <= U(:,i));
            %      con = con + (U(:,i) <= u_max);
            %      obj = obj + (X(:,i)-Xs)'*Q*(X(:,i)-Xs) + (U(:,i)-Us)'*R*(U(:,i)-Us);
            % end


            % obj = obj + X(:,N)'*Qf*X(:,N);
% 

            % Replace this line and set u0 to be the input that you
            % want applied to the system. Note that u0 is applied directly
            % to the nonlinear system. You need to take care of any 
            % offsets resulting from the linearization.
            % If you want to use the delta formulation make sure to
            % substract mpc.xs/mpc.us accordingly.
            % con = con + ( u0 == 0 );
            
            con = con + ( u0 == u(:,1) );


            % Pass here YALMIP sdpvars which you want to debug. You can
            % then access them when calling your mpc controller like
            % [u, X, U] = mpc_lon.get_u(x0, ref);
            % with debugVars = {X_var, U_var};
            debugVars = {};
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {x0, V_ref, u_ref, d_est, x0other}, {u0, debugVars{:}});
        end
        

        % Computes the steady state target which is passed to the
        % controller
        function [Vs_ref, us_ref] = compute_steady_state_target(mpc, ref, d_est)
            
            % ref
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
            xs = mpc.xs(2); %velocity
            us = mpc.us; %

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
    

            % Compute steady-state targets
            % Solve A * Vs_ref + B * us_ref + d_est = ref (velocity reference)
            % us_ref = (ref - A * xs) / B;
            us_ref = 0;
            % us_ref = (ref - A * ref) / B; % Steady-state throttle
            Vs_ref = ref;                        % Steady-state velocity
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end
