Ts = 0.1;                                % Time step for discretization: 0.1 seconds
H = 2;                                   % Prediction horizon in seconds

car = Car(Ts);

mpc = NmpcControl(car, H);

x0 = [0 0 0 80/3.6];
ref = [3 100/3.6];

u = mpc.get_u(x0,ref);

u
% 
params = {};
params.Tf = 15;
params.myCar.model = car;
params.myCar.x0 = [0 0 0 80/3.6]';
params.myCar.u = @mpc.get_u;
ref1 = [0 80/3.6]';
ref2 = [3 100/3.6]';

params.myCar.ref = car.ref_step(ref1, ref2, 2);

result = simulate(params);
visualization(car, result);