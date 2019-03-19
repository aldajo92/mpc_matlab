function [Uresult, PSI, GAMMA, THETA, H, G, Q_exp, R_exp] = MPC_unrestrict(A,B,X_0,U_init,ref,Hp,Hu,P,Q,R)

    I_R = eye(Hp);
    R_exp = kron(I_R,R);
    
    I_Q = eye(Hp-1);
    Q_exp = kron(I_Q,Q);
    Q_exp = [
        Q_exp zeros(size(Q_exp,1),size(Q,2));
        zeros(size(Q,1),size(Q_exp,2)) P
        ];

    PSI = [];
    for i = 1:Hp
       PSI = [PSI ; A^i];
    end

    GAMMA = [];
    SUM_AB = zeros(size(A*B));
    for i = 1:Hp
        SUM_AB = SUM_AB + (A^(i-1)) * B;
        GAMMA = [GAMMA ; SUM_AB];
    end
    
    THETA = [];
    for i=1:Hu
        temp_Gamma = GAMMA(1:(Hp-i+1)*size(B,1),1:size(B,2));
        temp = [zeros(size(GAMMA,1)-size(temp_Gamma,1), size(GAMMA,2)); temp_Gamma];
        THETA = [THETA temp];
    end
    
    % Reference expanding
    ref_exp = kron(ones(Hp,1),ref);
    
    % Calculating E_exp
    E_exp = ref_exp - PSI*X_0 - GAMMA*U_init;

    % Calculating H
    H = (THETA'*Q_exp*THETA + R);

    % Calculating G
    G = 2*THETA'*Q_exp*E_exp;

    Uresult = (G\H)'/2

end