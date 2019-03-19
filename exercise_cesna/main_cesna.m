clc; clear; close all

Ts = 0.5;
Tf = 10;

A = [-1.2822 0 0.98 0; 0 0 1 0; -5.4293 0 -1.8366 0; -128.2 128.2 0 0];
B = [-0.3;0;-17;0];
C = [0 1 0 0; 0 0 0 1; -128.2 128.2 0 0];
D = [0;0;0];

ss1 = ss(A,B,C,D);
ssDis = c2d(ss1, Ts);

%% create MPC controller object with sample time
mpc1 = mpc(ssDis, Ts);
% specify prediction horizon
mpc1.PredictionHorizon = 10;
% specify control horizon
mpc1.ControlHorizon = 2;
% specify nominal values for inputs and outputs
mpc1.Model.Nominal.U = 0;
mpc1.Model.Nominal.Y = [0;0;0];
% specify constraints for MV and MV Rate
mpc1.MV(1).Min = -0.262;
mpc1.MV(1).Max = 0.262;
mpc1.MV(1).RateMin = -0.524;
mpc1.MV(1).RateMax = 0.524;
% specify constraints for OV
mpc1.OV(1).Min = -0.349;
mpc1.OV(1).Max = 0.349;
% specify weights
mpc1.Weights.MV = 0.0001;
mpc1.Weights.MVRate = 0.01;
mpc1.Weights.OV = [1 1 1];
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

mpc_RefSignal = 40 * [zeros(Samples, 1), ones(Samples, 1), zeros(Samples, 1)];

for k = 1:Samples
     % System
     ysys = ssDis.C*xsys;
     xmpc.Plant = xsys;
     
     % Control action System
     u = mpcmove(mpc1,xmpc,ysys,mpc_RefSignal(k,:),[]);
     UU(k) = u;
     
     % Save System
     YY(k,:) = ysys';
     
     % System Response
     xsys = ssDis.A*xsys + ssDis.B*u;
end

subplot(2,1,1);
stairs(time,YY)
title('y_{system}');

subplot(2,1,2)
stairs(time,UU);
title('u');