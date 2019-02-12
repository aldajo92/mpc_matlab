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

X_equ3 = [ x1; x2; x3; x4 ]

f_eval = eval(F)

A = double(eval(A_J))
B = double(eval(B_J))

%% Observabilidad
obsv(A,C)

%% Generando el sistema de variables de estado
Ts = 0.1;
Tf = 600;

D = zeros(size(C,1),1);
ss1 = ss(A,B,C,D);
ssDis = c2d(ss1, Ts);

%% create MPC controller object with sample time
mpc1 = mpc(ssDis, Ts);
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

%%

time = 0:Ts:Tf;
Samples = size(time, 2);

xmpc = mpcstate(mpc1);

% Lineal plant
xsys = [0;0;0;0];
YY = zeros(Samples,size(C,1));
UU = zeros(Samples,1);

% Real plant
xmpc_plant = mpcstate(mpc1);

X_0 = X_equ3';
% X_0 = [0,0,0,0];
X_PLANT = X_0;
Y_PLANT = C*X_0';
Y_0_PLANT = C*X_0';
YY_PLANT = zeros(Samples,size(C,1));
U_PLANT = zeros(Samples,1);

mpc_RefSignal = 138.5500 * ones(Samples, 1);

for k = 1:Samples
     % System
     ysys = ssDis.C*xsys;
     xmpc.Plant = xsys;
     
     % Plant
     Y_PLANT = C*X_PLANT';
     xmpc_plant.Plant = (X_PLANT-X_0)';
%      d = ysys - (Y_PLANT - Y_0_PLANT);
     
     % Control action System
     u = mpcmove(mpc1,xmpc,ysys,mpc_RefSignal(k),[]);
     UU(k) = u;

     % Control action Plant
     u_plant = mpcmove(mpc1,xmpc_plant,Y_PLANT-Y_0_PLANT,mpc_RefSignal(k),[]);
     U_PLANT(k) = u_plant+ueq;
     
     % Save System
     YY(k,:) = ysys';
     % Save Plant
     YY_PLANT(k,:) = Y_PLANT';
     
     % System Response
     xsys = ssDis.A*xsys + ssDis.B*u;
     
     % Plant Response
     [t_emulation,x_plant] = plant(u_plant+ueq, X_PLANT, time(k), Ts);
     X_PLANT = x_plant(size(x_plant,1), :);
end

subplot(2,2,1);
stairs(time,YY)
title('y_{system}');

subplot(2,2,2);
stairs(time,YY_PLANT)
title('y_{plant}');

subplot(2,2,3)
stairs(time,UU);
title('u');

subplot(2,2,4)
stairs(time,U_PLANT);
title('u');