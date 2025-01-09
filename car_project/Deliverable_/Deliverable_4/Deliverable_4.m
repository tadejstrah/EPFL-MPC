clear all
close all
clc

%% Initialize MPC Controllers

Ts = 1/25;                               % Sample time of 40 ms seconds
H = 3;                                  % Prediction horizon of 3 seconds

% Setup car and get linearized model
car = Car(Ts);
[xs, us] = car.steady_state(120/3.6);    % Linearize at 120 km/h
sys = car.linearize(xs, us);             % Get linearized system
[sys_lon, sys_lat] = car.decompose(sys); % Split into longitudinal and lateral

% Create Estimator
estimator = LonEstimator(sys_lon, Ts);

% Create MPC controllers
mpc_lon = MpcControl_lon(sys_lon, Ts, H);
mpc_lat = MpcControl_lat(sys_lat, Ts, H);
mpc = car.merge_lin_controllers(mpc_lon, mpc_lat);

%% Simulating Closed-Loop Combined MPC with Longitudinal Estimator

x0 = [0 0 0 80/3.6]'; % (x, y, theta, V)
ref1 = [0 80/3.6]';   % (y_ref, V_ref)
ref2 = [3 50/3.6]';   % (y_ref, V_ref)

params = {};
params.Tf = 15;
params.myCar.model = car;
params.myCar.x0 = x0;
params.myCar.est_fcn = @estimator.estimate;
params.myCar.est_dist0 = 0;
params.myCar.u = @mpc.get_u;
params.myCar.ref = car.ref_step(ref1, ref2, 2);
result = simulate(params);
visualization(car, result);
