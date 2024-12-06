
Ts = 1/10; %sampling time
car = Car(Ts);

car = Car(Ts);
Tf = 2.0; % Simulation end time
x0 = [0, 0, deg2rad(-2), 20/3.6]'; % (x, y, theta, V) Initial state
u = [deg2rad(1), 0.7]'; % (delta, u T) Constant input
params = {}; % Setup simulation parameter struct
params.Tf = Tf;
params.myCar.model = car;
params.myCar.x0 = x0;
params.myCar.u = u;
result = simulate(params); % Simulate nonlinear model
visualization(car, result);

