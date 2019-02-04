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

A = jacobian(F, X)
B = jacobian(F, U)

%% calculando los puntos de equilibrio
u1 = 0;
F1 = eval(F);
F_equi = F1 == [0;0;0;0];

[a1,a2,a3,a4] = solve(F_equi, X);

X_equ1 = double([ a1(1); a2(1); a3(1); a4(1) ])
X_equ2 = double([ a1(2); a2(2); a3(2); a4(2) ])
%%  evaluando punto de equilibrio 1 y calculando el Jacobiano 1
x1 = a1(1);
x2 = a2(1); 
x3 = a3(1);
x4 = a4(1);

u1 = 0;

A_linear = double(eval(A))
B_linear = double(eval(B))

%%  evaluando punto de equilibrio 2
x1 = a1(2);
x2 = a2(2); 
x3 = a3(2);
x4 = a4(2);

u1 = 0;

A_linear = double(eval(A))
B_linear = double(eval(B))

%% punto de equilibrio 3

ueq = 400;

x1 = ((ueq + w50 * Ku)*mu*c)/(betha*k*w50*Ku);
x2 = (s-delta*x1)/mu;
x3 = w50*Ku*k*x2/(c*(ueq+w50*Ku));
x4 = ueq/Ku;

u1 = ueq;

f_eval = eval(F)

A_linear = double(eval(A))
B_linear = double(eval(B))

%% Generando el sistema
D = zeros(size(C,1),1);
ss1 = ss(A_linear,B_linear,C,D);
ssDis = c2d(ss1, 0.5);