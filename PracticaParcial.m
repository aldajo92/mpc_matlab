clc; clear; close all;

format short;

syms x1 x2 x3 x4 u1

s = 9;
delta = 0.009;
betha = 4e-6;
mu = 0.3;
w50 = 89.6;
k = 80;
c = 0.6;
Ku = 8.4;

F1 = s - delta * x1 - betha * x1 * x3;
F2 = betha * x1 * x3 - mu * x2;
F3 = (1-(x4/(x4+w50)))*k*x2 - c*x3;
F4 = -Ku * x4 + u1;

U = [u1];
X = [x1;x2;x3;x4];
F = [F1; F2; F3; F4];

A_J = jacobian(F, X);
B_J = jacobian(F, U);

C = [1 0 0 0];

%% calculando los puntos de equilibrio
u1 = 0;
F1 = eval(F);
F_equi = [0;0;0;0] ==  F1;

[a1,a2,a3,a4] = solve(F_equi, X);

S = solve(F_equi, X);

X_equ1 = [ S.x1(1); S.x2(1); S.x3(1); S.x4(1) ]
X_equ2 = [ S.x1(2); S.x2(2); S.x3(2); S.x4(2) ]
%%  evaluando punto de equilibrio 1 y calculando el Jacobiano 1
% x1 =  S.x1(1);
% x2 =  S.x2(1);
% x3 =  S.x3(1);
% x4 =  S.x4(1);
% 
% u1 = 0;

% A = double(eval(A_J))
% B = double(eval(B_J))

%%  evaluando punto de equilibrio 2 y calculando el Jacobiano 1
% x1 =  S.x1(2);
% x2 =  S.x2(2);
% x3 =  S.x3(2);
% x4 =  S.x4(2);
% 
% u1 = 0;

% A = double(eval(A_J))
% B = double(eval(B_J))

%% punto de equilibrio 3

ueq = 400;

x1 = ((ueq + w50 * Ku)*mu*c)/(betha*k*w50*Ku);
x2 = (s-delta*x1)/mu;
x3 = w50*Ku*k*x2/(c*(ueq+w50*Ku));
x4 = ueq/Ku;

u1 = ueq;

f_eval = eval(F)

A = double(eval(A_J))
B = double(eval(B_J))

%% Observabilidad
obsv(A,C)

%% Generando el sistema de variables de estado
D = zeros(size(C,1),1);
ss1 = ss(A,B,C,D);
ssDis = c2d(ss1, 0.1);

%% create MPC controller object with sample time
mpc1 = mpc(ssDis, 0.1);
% specify prediction horizon
mpc1.PredictionHorizon = 100;
% specify control horizon
mpc1.ControlHorizon = 2;
% specify nominal values for inputs and outputs
mpc1.Model.Nominal.U = 0;
mpc1.Model.Nominal.Y = 0;
% specify constraints for MV and MV Rate
mpc1.MV(1).Min = -400;
mpc1.MV(1).Max = 300;
mpc1.MV(1).RateMin = -100;
mpc1.MV(1).RateMax = 200;
% specify weights
mpc1.Weights.MV = 0.0001;
mpc1.Weights.MVRate = 0.1;
mpc1.Weights.OV = 1;
mpc1.Weights.ECR = 100000;

% specify simulation options
options = mpcsimopt();
options.RefLookAhead = 'off';
options.MDLookAhead = 'off';
options.Constraints = 'on';
options.OpenLoop = 'off';

mpc_RefSignal = 138.5500* ones(2001, 1);

time = 0:0.1:200;

% close all
% [output, time, Ux] = sim(mpc1, 2001, mpc_RefSignal, [], options);
% plot(time, output)
% figure()
% plot(time, Ux)

ym = 0;
x_state = mpcstate(mpc1);

X_0 = [0, 0, 0, 0];

u_input = zeros(size(time));

x_plant = zeros(size(time,2),4);
y_output = zeros(size(time,2),4);

y_linear = zeros(size(time,2),4);
%%
for i = 1:size(time, 2)
    % simulated plant and predictive model are identical
    y_linear(i,:) = ssDis.C*X_0';
    xmpc.Plant = X_0';
    u_input(i) = mpcmove(mpc1,x_state,[],1);
end

stairs(y_linear)

%% using for
for i = 1:size(time, 2)
    x_plant(i,:) = X_0;
    y_output(i) = ym;
    u_input(i) = mpcmove(mpc1,x_state,ym,mpc_RefSignal(i),[]);
    [t,x] = plant(u_input(i), X_0, time(i), 0.1);
    X_0 = x(size(x,1), :);
    ym = C*X_0';
end

stairs(time, y_output)
hold on