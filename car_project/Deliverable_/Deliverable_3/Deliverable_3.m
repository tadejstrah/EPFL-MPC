clear all
close all
clc

%% Configure and Initialize MPC Controllers

Ts = 0.1;                                % Time step for discretization: 0.1 seconds
H = 3;                                   % Prediction horizon in seconds

% Set up vehicle model and linearize at a given speed
car = Car(Ts);
[xs, us] = car.steady_state(120 / 3.6);  % Find equilibrium point at 120 km/h
[sys_lon, sys_lat] = car.decompose(car.linearize(xs, us)); % Decompose into longitudinal and lateral subsystems

% Generate predictive controllers for both dynamics
mpc_lat = MpcControl_lat(sys_lat, Ts, H);
mpc_lon = MpcControl_lon(sys_lon, Ts, H);
mpc = car.merge_lin_controllers(mpc_lat, mpc_lon);

%% Open-Loop Longitudinal MPC Simulation

% Initialize state and reference for longitudinal control
x_lon = [0; 80 / 3.6];                   % Start with an initial speed of 80 km/h
ref_lon = 120 / 3.6;                     % Target speed: 120 km/h

% Compute control inputs and predicted trajectory
[u_lon, X_lon, U_lon] = mpc_lon.get_u(x_lon, ref_lon);

%% Open-Loop Lateral MPC Simulation

% Initialize state and reference for lateral control
x_lat = [0; 0];                          % Begin at the lane center
ref_lat = 3;                             % Target lateral displacement: 3 m

% Compute control inputs and predicted trajectory
[u_lat, X_lat, U_lat] = mpc_lat.get_u(x_lat, ref_lat);

%% Visualize All Open-Loop Results in a Single Window

figure;

% Longitudinal Speed Prediction
subplot(3, 2, 1);
predicted_speed = X_lon(2,:) + mpc_lon.xs(2);
plot(0:Ts:(size(X_lon,2)-1)*Ts, predicted_speed * 3.6, 'b');
hold on;
plot([0, (size(X_lon,2)-1)*Ts], [ref_lon, ref_lon] * 3.6, 'r--');
ylabel('Speed [km/h]');
xlabel('Time [s]');
title('Longitudinal Velocity Prediction');
legend('Predicted Speed', 'Reference Speed');
grid on;

% Longitudinal Throttle Input
subplot(3, 2, 2);
predicted_throttle = U_lon + mpc_lon.us;
plot(0:Ts:(size(U_lon,2)-1)*Ts, predicted_throttle, 'g');
ylabel('Throttle Input [-]');
xlabel('Time [s]');
title('Longitudinal Control Input');
grid on;

% Lateral Position Prediction
subplot(3, 2, 3);
predicted_position = X_lat(1,:) + mpc_lat.xs(1);
plot(0:Ts:(size(X_lat,2)-1)*Ts, predicted_position, 'b');
hold on;
plot([0, (size(X_lat,2)-1)*Ts], [ref_lat, ref_lat], 'r--');
ylabel('Lateral Position [m]');
xlabel('Time [s]');
title('Lateral Position Prediction');
legend('Predicted Position', 'Reference Position');
grid on;

% Lateral Heading Angle Prediction
subplot(3, 2, 4);
predicted_heading = X_lat(2,:) + mpc_lat.xs(2);
plot(0:Ts:(size(X_lat,2)-1)*Ts, predicted_heading * 180 / pi, 'g');
ylabel('Heading Angle [°]');
xlabel('Time [s]');
title('Predicted Heading Angle');
grid on;

% Lateral Steering Input
subplot(3, 2, 5);
predicted_steering = U_lat + mpc_lat.us;
plot(0:Ts:(size(U_lat,2)-1)*Ts, predicted_steering * 180 / pi, 'm');
ylabel('Steering Angle [°]');
xlabel('Time [s]');
title('Steering Input Prediction');
grid on;

mpc_lon = MpcControl_lon(sys_lon, Ts, H);
mpc_lat = MpcControl_lat(sys_lat, Ts, H);
mpc = car.merge_lin_controllers(mpc_lon, mpc_lat);
x0 = [0 0 0 80/3.6]'; % (x, y, theta, V)
ref1 = [0 80/3.6]'; % (y ref, V ref)
ref2 = [3 120/3.6]'; % (y ref, V ref)
params = {};
params.Tf = 15;
params.myCar.model = car;
params.myCar.x0 = x0;
params.myCar.u = @mpc.get_u;
params.myCar.ref = car.ref_step(ref1, ref2, 5); % delay reference step by 5s
result = simulate(params);
visualization(car, result);