Ts = 0.1;                                % Time step for discretization: 0.1 seconds
H = 2.2;                                   % Prediction horizon in seconds

car = Car(Ts);

mpc = NmpcControl_overtake(car, H);

x0_ego = [0 0 0 80/3.6];
x0_other = [20 0 0 80/3.6];
ref1 = [0 80/3.6];
ref2 = [0 100/3.6];


params = {};
params.Tf = 15;
params.myCar.model = car;
params.myCar.x0 = [0 0 0 80/3.6]';
params.myCar.u = @mpc.get_u;
params.myCar.ref = car.ref_step(ref1, ref2, 1);

params.otherCar.model = car;
params.otherCar.x0 = x0_other;
params.otherCar.u = car.u_const(80/3.6);

result = simulate(params);
visualization(car, result);