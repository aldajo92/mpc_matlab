function [R_exp, Q_exp, T, H, F, Y] = expandSystem(R,Q,S,A,B,N)

    I_R = eye(N);
    R_exp = kron(I_R,R);
    
    I_Q = eye(N-1);
    
    Q_exp = kron(I_Q,Q);
    Q_exp = [
        Q_exp zeros(size(Q_exp,1),size(Q,2));
        zeros(size(Q,1),size(Q_exp,2)) S
        ];
    
    T = [];
    for i = 1:N
       T = [T ; A^i];
    end
    
    S_exp = [];
    Gamma = [];

    for i=1:N
        Gamma = [Gamma; A^(i-1)*B];
    end

    for i=1:N
        temp_Gamma = Gamma(1:(N-i+1)*size(B,1),1:size(B,2));
        temp = [zeros(size(Gamma,1)-size(temp_Gamma,1), size(Gamma,2)); temp_Gamma];
        S_exp = [S_exp temp];
    end
   
    H = R_exp + S_exp'*Q_exp*S_exp;
    
    F = 2*T'*Q_exp*S_exp;
    
    Y = 2*(Q+T'*Q_exp*T);
    
end