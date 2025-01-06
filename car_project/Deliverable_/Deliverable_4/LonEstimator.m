classdef LonEstimator
    properties
        % continous sub-system
        sys
        % Extended linearization points
        xs_hat, us_hat
        % Extended system matrices
        A_hat, B_hat, C_hat
        % Observer gain matrix
        L
    end
    
    methods
        % Setup the longitudinal disturbance estimator
        function est = LonEstimator(sys, Ts)

            xs = sys.UserData.xs;
            us = sys.UserData.us;
            
            % Discretize the system and extract the A,B,C,D matrices
            [~, Ad, Bd, Cd, ~] = Car.c2d_with_offset(sys, Ts);

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE

            % Define the velocity index for the longitudinal subsystem
            velocity_index = 2; % Position is 1, velocity is 2
            
            % Initial extended state vector (velocity and disturbance)
            initial_velocity = xs(velocity_index); % Steady-state velocity from linearization
            initial_disturbance = 0;  % Initial guess for disturbance is zero
            est.xs_hat = [initial_velocity; initial_disturbance];  % [velocity; disturbance]
            
            % Steady-state input for reference
            est.us_hat = us;  % Reference input for coordinate transformations
            
            % Augmented system matrix (A_hat) for velocity and disturbance dynamics
            velocity_dynamics = Ad(velocity_index, velocity_index);  % Evolution of velocity
            disturbance_effect = Bd(velocity_index);                % How disturbance affects velocity
            est.A_hat = [velocity_dynamics, disturbance_effect; 0, 1];  % Disturbance persistence modeled as a constant
            
            % Augmented input matrix (B_hat)
            est.B_hat = [disturbance_effect; 0];  % Input affects velocity but not disturbance
            
            % Augmented output matrix (C_hat) for velocity measurement
            velocity_measurement = Cd(velocity_index, velocity_index);  % Velocity measurement matrix
            est.C_hat = [velocity_measurement, 0];  % Only velocity is measured, no disturbance measurement
            
            % Define observer poles for desired estimation dynamics (faster or slower estimation)
            observer_poles = [0.6; 0.7];  % Moderately fast response
            
            % Calculate observer gain matrix L using pole placement
            % Transpose augmented system and output matrices for the 'place' function
            est.L = place(est.A_hat', est.C_hat', observer_poles)';  % Transpose the result to match original orientation
            
            % Store the original continuous-time system model in the estimator object
            est.sys = sys;  % Original system model reference for potential use
            
            % Additional information for the observer, if needed, can be stored here


            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
        
        % This function takes in the the estimate, input, and measurement
        % at timestep i and predicts the next (extended) state estimate at
        % the next timestep i + 1.
        function z_hat_next = estimate(est, z_hat, u, y)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   z_hat      - current estimate (V, dist)
            %   u          - longitudinal input (u_T)
            %   y          - longitudinal velocity measurement (V)
            % OUTPUTS
            %   z_hat_next - next time step estimate
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % Convert absolute coordinate to delta
            measurement_deviation = y - est.xs_hat(1); % Measurement deviation error from steady state
            input_deviation = u - est.us_hat; % Input deviation error from steady state
            
            % Apply the observer equation in deviation coordinates
            state_estimate_deviation_next = est.A_hat * (z_hat - est.xs_hat) + est.B_hat * input_deviation + ...
                                            est.L * (measurement_deviation - est.C_hat * (z_hat - est.xs_hat));
            
            % Convert delta back to absolute coordinates
            z_hat_next = state_estimate_deviation_next + est.xs_hat;

            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end
