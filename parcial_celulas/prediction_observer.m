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

Aexp = [ssDis.A zeros(size(A,1),size(C,1)); zeros(size(C,1),size(A,2)) eye(size(C,1))];
Bexp = [ssDis.B; zeros(size(C,1),size(B,2))];

Cexp = [ssDis.C eye(size(C,1))];

VdI = .1*eye(size(Aexp));     % disturbance covariance
Vn = 1;

[Kf,P,E] = dlqe(Aexp,VdI,Cexp,VdI,Vn)

Aest = Aexp - Kf*Cexp;

sysExpD = ss(Aexp - Kf*Cexp, [Bexp Kf], eye(size(Aexp)), 0*[Bexp Kf],Ts);
% sysD = c2d(sysC,Ts)


% ssExp = ss(Aexp, Bexp, Cexp, 0)