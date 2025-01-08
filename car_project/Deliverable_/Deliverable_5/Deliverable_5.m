clear all
close all
clc

%% MPC Controller Setup
Ts = 0.1;                                % Sampling interval: 0.1 seconds
H = 6;                                   % Prediction horizon: 2 seconds

% Instantiate the vehicle system and linearize its dynamics
car = Car(Ts);
[xs, us] = car.steady_state(120 / 3.6);  % Find equilibrium point at 120 km/h
sys = car.linearize(xs, us);             % Obtain linearized model
[sys_lon, sys_lat] = car.decompose(sys); % Separate longitudinal and lateral dynamics

% Create predictive controllers for longitudinal and lateral dynamics
mpc_lon = MpcControl_lon(sys_lon, Ts, H);
mpc_lat = MpcControl_lat(sys_lat, Ts, H);
mpc = car.merge_lin_controllers(mpc_lon, mpc_lat);

%% Simulating Robust Tube MPC in a Closed-Loop System

% Define simulation parameters
ref = [0; 120 / 3.6]; % Target states: [lateral position; longitudinal velocity]
params = {};
params.Tf = 25;                           % Simulation duration: 25 seconds
params.myCar.model = car;                 % Ego vehicle dynamics
params.myCar.x0 = [0; 0; 0; 115 / 3.6];   % Initial state: slightly slower than target
params.myCar.u = @mpc.get_u;              % Control input using MPC
params.myCar.ref = ref;                   % Tracking reference for ego vehicle

% Define behavior of the other vehicle (e.g., lead vehicle)
params.otherCar.model = car;              % Lead vehicle dynamics
params.otherCar.x0 = [8; 0; 0; 120 / 3.6];% Initial state: ahead of ego vehicle
params.otherCar.u = car.u_fwd_ref();      % Forward reference for other car
params.otherCar.ref = car.ref_robust();   % Robust reference to simulate variability

% Run simulation and generate visualization
result = simulate(params);                % Execute closed-loop simulation
visualization(car, result);               % Visualize simulation outcome