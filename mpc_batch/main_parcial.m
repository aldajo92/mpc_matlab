clc; clear; close all;

Ts = 0.05;
Tf = 2;

% Systema continuo
A = [0 1;0 0];
B = [0; 1];
C = [1 0; 0 1];
D = [0; 0];

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

time = 0:Ts:Tf;
Samples = size(time, 2);

% ============ Using mpc batch ========== %
xbatch_0 = [-1;0];
xsysbatch = xbatch_0;
YY_batch = zeros(Samples,size(C,1));
UU_batch = zeros(Samples,1);

R = 0.001*eye(size(B,2));
Q = 1*eye(size(A));
S = 1*eye(size(A));
N = 100;

% ============= Ref signal ============= %
mpc_RefSignal = ones(Samples, 1);
mpc_RefSignal(5/Ts:Samples) = 0;

for k = 1:Samples
     % Output System
%      ysys = ssD.C*xsys;
%      xmpc.Plant = xsys;
     
     % Control action System
     %      u = mpcmove(mpc1,xmpc,ysys,mpc_RefSignal(k),[]);
     %      UU(k) = u;
     
     % Save System
     %      YY(k,:) = ysys';
     
     % ================= Using batch ================= &
     
     % Save System
     YY_batch(k,:) = (ssD.C*xsysbatch)';
     
     % mpcBatch(R,Q,S,A,B,N,X_0);
     u_batch = LQRBatch(R,Q,S,ssD.A,ssD.B,N,xsysbatch);
     UU_batch(k) = u_batch;
     xsysbatch = ssD.A*xsysbatch + ssD.B*u_batch;
     
     % =============================================== &
     
     % System Response
     % xsys = ssD.A*xsys + ssD.B*u;
%      disp('..')
end

subplot(2,1,1);
stairs(time,YY_batch,'LineWidth', 3.0)
% hold on
% plot(time,mpc_RefSignal,'k--','LineWidth', 3.0);
title('x_{system}');

subplot(2,1,2)
stairs(time,UU_batch);
title('u');
