clc; clear; close all;

Ts = 0.05;
Tf = 50;

% Systema continuo
A = [0 1;0 0];
B = [0; 1];
C = [1 0];
D = [0];

ssC = ss(A,B,C,D);
ssD = c2d(ssC, 0.05);

%% Observabilidad
OBS = obsv(A,C);
disp('Matriz de Observabilidad');
disp(OBS);
%% Controlabilidad
CTR = ctrb(A,B);
disp('Matriz de Controlabilidad');
disp(CTR);

%% create MPC controller object with sample time
mpc1 = mpc(ssD, Ts);
%% specify prediction horizon
mpc1.PredictionHorizon = 10;
%% specify control horizon
mpc1.ControlHorizon = 10;
%% specify constraints for MV and MV Rate
mpc1.MV(1).Min = -1;
mpc1.MV(1).Max = 1;
%% specify weights
mpc1.Weights.MV = 0.01;
mpc1.Weights.MVRate = 0.01;
mpc1.Weights.OV = 0.00001;
mpc1.Weights.ECR = 100000;
%% specify simulation options
options = mpcsimopt();
options.RefLookAhead = 'off';
options.MDLookAhead = 'off';
options.Constraints = 'on';
options.OpenLoop = 'off';

time = 0:Ts:Tf;
Samples = size(time, 2);

xmpc = mpcstate(mpc1);

% Lineal plant
xsys = [1;0];
YY = zeros(Samples,size(C,1));
UU = zeros(Samples,1);

mpc_RefSignal = ones(Samples, 1);

mpc_RefSignal(5/Ts:Samples) = 0;

for k = 1:Samples
     % Output System
     ysys = ssD.C*xsys;
     xmpc.Plant = xsys;
     
     % Control action System
     u = mpcmove(mpc1,xmpc,ysys,mpc_RefSignal(k),[]);
     UU(k) = u;
     
     % Save System
     YY(k,:) = ysys';
     
     % System Response
     xsys = ssD.A*xsys + ssD.B*u;
end

subplot(2,1,1);
stairs(time,YY, 'r--','LineWidth', 3.0)
hold on
plot(time,mpc_RefSignal,'k--','LineWidth', 3.0);
title('y_{system}');

subplot(2,1,2)
stairs(time,UU);
title('u');
