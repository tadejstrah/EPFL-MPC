classdef NmpcControl_overtake < handle

    properties
        % The NMPC problem
        opti

        % Problem parameters
        x0, ref, x0other

        % Most recent problem solution
        sol

        % The input that you want to apply to the system
        u0

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Add any variables you would like to read to debug here
        % and then store them in the NmpcControl function below.
        % e.g., you could place X here and then add obj.X = X
        % in the NmpcControl function below.
        % 
        % After solving the problem, you can then read these variables 
        % to debug via
        %   nmpc.sol.value(nmpc.X)
        % 
        X, U
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    end

    methods
        function obj = NmpcControl_overtake(car, H)

            import casadi.*

            N_segs = ceil(H/car.Ts); % Horizon steps
            N = N_segs + 1;          % Last index in 1-based Matlab indexing

            nx = 4;
            nu = 2;

            % Define the NMPC optimization problem
            opti = casadi.Opti();
            
            % Parameters (symbolic)
            obj.x0 = opti.parameter(nx, 1);       % initial state
            obj.ref = opti.parameter(2, 1);       % target y, velocity
            obj.x0other = opti.parameter(nx, 1);  % initial state of other car

            % SET THIS VALUE TO BE YOUR CONTROL INPUT
            obj.u0 = opti.variable(nu, 1);

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            f_discrete = @(x,u) RK4(x,u,car.Ts,@car.f);

      
            state_constraint = [0;
                                0;
                                3.5;          % Maximum allowed y
                                0.5;          % Minimum allowed y (-)
                                5*pi/180;     % Maximum allowed theta
                                5*pi/180      % Minimum allowed theta (-)
                                0;
                                0];



            state_constraint_sign = [0  0  0  0;     
                                     0  0  0  0;    
                                     0  1  0  0;    %  
                                     0 -1  0  0;    % 
                                     0  0  1  0;    % 
                                     0  0 -1  0;    % 
                                     0  0  0  0;
                                     0  0  0  0];    
            
            input_constraint = [30*pi/180;    % Max steering angle δ ≤ 30°
                                30*pi/180;
                                1;
                                1];   % Min steering angle (-)

            input_constraint_sign = [1 0; 
                                    -1 0;
                                     0 1;
                                     0 -1];  

            X = opti.variable(nx,N+1); % state trajectory variables
            U = opti.variable(nu,N);   % control trajectory (throttle, brake)

            Q = diag([0, 1,100,300]); % weights for [x, y, theta, v]
            R = diag([200,100]);  % weights for [delta, u]

            % Inital condition
            opti.subject_to(X(:,1) == obj.x0);

            cost = 0;
            
            car_l = 12;
            car_w = 3;
            H = diag([1/(car_l^2),1/(car_w^2)]);
            
            
            for k=1:N
                state_delta = X(:,k) - [0 obj.ref(1) 0 obj.ref(2)]';

                opti.subject_to(X(:,k+1) == f_discrete(X(:,k), U(:,k)));
                cost =  cost +  state_delta'*Q*state_delta + U(:,k)'*R*U(:,k);

                opti.subject_to(input_constraint_sign*(U(:,k)) <= input_constraint);
                opti.subject_to(state_constraint_sign*(X(:,k)) <= state_constraint);
    
                % other car position: x0_other + k*Ts.obj.x0other(4)
                
                lon_other = obj.x0other(1) + k*car.Ts*obj.x0other(4) ;
                lat_other = obj.x0other(2);
                p_l = [lon_other lat_other]';

                lon = X(1,k); 
                lat = X(2,k);
                p = [lon lat]';
             
                opti.subject_to((p-p_l)' * H * (p-p_l) >= 1);

            end
    
      

            opti.subject_to( obj.u0 == U(:,1) );

            opti.minimize(cost);

            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % Store the defined problem to solve in get_u
            obj.opti = opti;

            % Setup solver
            options = struct;
            options.ipopt.print_level = 0;
            options.print_time = 0;
            options.expand = true;
            obj.opti.solver('ipopt', options);
        end

        function u = get_u(obj, x0, ref, x0other)

            if nargin < 4
                x0other = zeros(4, 1);
            end


            % obj.opti.debug.value(X)
            % Compute solution from x0
            obj.solve(x0(1:4), ref, x0other(1:4));


            u = obj.sol.value(obj.u0);

        end

        function solve(obj, x0, ref, x0other)

            % Pass parameter values
            obj.opti.set_value(obj.x0, x0);
            obj.opti.set_value(obj.ref, ref);
            obj.opti.set_value(obj.x0other, x0other);

            obj.sol = obj.opti.solve();   % actual solve
            
            % obj.opti.debug.show_infeasibilities()
            % obj.opti.callback(@(i) plot(obj.opti.debug.value(obj.X),'DisplayName',num2str(i)))
            % obj.opti.debug.value(obj.x0)

            % Set warm start for next solve
            obj.opti.set_initial(obj.sol.value_variables());
            obj.opti.set_initial(obj.opti.lam_g, obj.sol.value(obj.opti.lam_g));
        end
    end
end
