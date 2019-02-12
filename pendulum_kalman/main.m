clc; clear; close all;

m = 1;
M = 5;
L = 2;
g = -10;
d = 1;

s = -1;

A = [0 1 0 0;
     0 -d/M -m*g/M 0;
     0 0 0 1;
     0 -s*d/(M*L) -s*(m+M)*g/(M*L) 0];

B = [0; 1/M; 0; s*1/(M*L)];

C = [1 0 0 0];

D = zeros(size(C,1), size(B,2));

%% Augment system with disturbances and noise
Vd = .1*eye(4); % disturbance covariance
Vn = 1;

BF = [B Vd 0*B];

sysC = ss(A,BF,C,[0 0 0 0 0 Vn]);       % build state space system

sysFullOutput = ss(A,BF,eye(4),zeros(4,size(BF,2)));

%% Build Kalman filter
[Kf,P,E] = lqe(A,Vd,C,Vd,Vn);
Kf = (lqr(A',C',Vd,Vn))';

sysKF = ss(A-Kf*C,[B Kf],eye(4),0*[B Kf]);
 
%% Estimate linearized system in "down" position (Gantry crane)
dt = .01;
t = dt:dt:50;

uDIST = randn(4,size(t,2));

uNOISE = randn(size(t));
u = 0*t;
u(100:200) = 20;       % Impulse
u(1500:1520) = -100;    % Impulse

uAUG = [u; Vd*Vd*uDIST; uNOISE];

[y, t] = lsim(sysC,uAUG,t);
plot(t,y);

[xtrue,t] = lsim(sysFullOutput,uAUG,t);
hold on
plot(t,xtrue(:,1),'r','LineWidth', 2.0);

%% Kalman filter estimate
[x,t] = lsim(sysKF, [u; y'], t);
plot(t,x(:,1),'k--','LineWidth', 2.0);

figure
plot(t,xtrue, '-',t,x,'--','LineWidth', 2.0);





