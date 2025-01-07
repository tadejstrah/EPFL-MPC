Ts = 1/10; % Sample time
car = Car(Ts);
[xs, us] = car.steady_state(120 / 3.6);
sys = car.linearize(xs, us);
[sys_lon, sys_lat] = car.decompose(sys);
% Design MPC controller
H_lon = 1; % Horizon length in seconds
% mpc_lon = MpcControl_lon(sys_lon, Ts, H_lon);
% Get control input for longitudinal subsystem
% u_lon = mpc_lon.get_u(0, 1.29);


mpc_lon = MpcControl_lon(sys_lon, Ts, H_lon);
mpc_lat = MpcControl_lat(sys_lat, Ts, H_lon);
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

[V_ref, u_ref] = mpc_lon.compute_steady_state_target(120/3.6, 0)
% xs(4) == V_ref 
% us(2) == u_ref

% mpc_lon.xs(2)
mpc_lon.us

result = simulate(params);
visualization(car, result);