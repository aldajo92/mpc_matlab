clc; clear; close all; 

A = [1 1; 2 2];
B = [1 1;1 1];
Hp = 6;
Hu = 3;

P = [1 0; 0 1];
Q = [1 0; 0 1];
R = [0.01];

X_0 = [1;1];
U_init = [0.1; 0.1];

% Reference
REF = [1;1];

[Uresult, PSI, GAMMA, THETA, H, G, Q_exp, R_exp] = MPC_unrestrict(A,B,X_0,U_init,REF,Hp,Hu,P,Q,R);

% PSI = [];
% for i = 1:3
%    PSI = [PSI ; A^i]
% end
% 
% GAMMA = [];
% SUM_AB = zeros(size(A*B));
% for i = 1:Hp
%     SUM_AB = SUM_AB + (A^(i-1)) * B;
%     GAMMA = [GAMMA ; SUM_AB];
% end
% 
% THETA = [];
% for i=1:Hu
%     temp_Gamma = GAMMA(1:(Hp-i+1)*size(B,1),1:size(B,2))
%     temp = [zeros(size(GAMMA,1)-size(temp_Gamma,1), size(GAMMA,2)); temp_Gamma];
%     THETA = [THETA temp];
% end
% 
% I_Q = eye(Hp-1);
% Q_exp = kron(I_Q,Q);
% Q_exp = [
%     Q_exp zeros(size(Q_exp,1),size(Q,2));
%     zeros(size(Q,1),size(Q_exp,2)) P
%     ];
% 
% I_R = eye(Hu);
% R_exp = kron(I_R,R);
% 
% REF_exp = kron(ones(Hp,1),REF)
% 
% % Calculating E_exp
% E_exp = REF_exp - PSI*X_0 - GAMMA*U_init
% 
% % Calculating H
% H = (THETA'*Q_exp*THETA + R)
% 
% % Calculating G
% G = 2*THETA'*Q_exp*E_exp
% 
% Uresult = (G\H)/2

%%
U_min = [0.05; 0.05];
U_max = [5; 5];

x_min = [0.05; 0.05];
x_max = [5; 5];

del_u_min = [0.05; 0.05];
del_u_max = [5; 5];

%%
U_rest = MPC_restrict(A,B,X_0,U_init,REF,Hp,Hu,P,Q,R,U_min,U_max,x_min,x_max,del_u_min,del_u_max)

% A_u_min = -tril(ones(size(U_init,1)*Hu));
% A_u_max = tril(ones(size(U_init,1)*Hu));
% 
% f1_u_kron = ones(Hu,1);
% f1_u_min = kron(f1_u_kron, -(U_min - U_init));
% f1_u_max = kron(f1_u_kron, (U_max - U_init));
% 
% A_x_min = -THETA;
% A_x_max = THETA;
% 
% f1_x_kron = ones(Hp,1);
% f1_x_min = - (kron(f1_x_kron, x_min) - PSI*X_0 + GAMMA*U_init);
% f1_x_max = kron(f1_x_kron, x_max) - PSI*X_0 + GAMMA*U_init;
% 
% A_del_u_min = -eye(size(U_init,1)*Hu);
% A_del_u_max = eye(size(U_init,1)*Hu);
% 
% f1_del_u_min = kron(f1_u_kron, -del_u_min);
% f1_del_u_max = kron(f1_u_kron, del_u_max);
% 
% A_restriction = [A_u_min; A_u_max; A_x_min; A_x_max; A_del_u_min; A_del_u_max];
% f1_restriction = [f1_u_min; f1_u_max; f1_x_min; f1_x_max; f1_del_u_min; f1_del_u_max];
% 
% % U_rest = quadprog(2*H, -G, A_restriction, f1_restriction,[],[], f1_del_u_min, f1_del_u_max);
% U_rest = quadprog(2*H, -G, A_restriction, f1_restriction);
