function U_rest = MPC_restrict(A,B,X_0,U_init,ref,Hp,Hu,P,Q,R,U_min,U_max,x_min,x_max,del_u_min,del_u_max)

    [~, PSI, GAMMA, THETA, H, G, ~, ~] = MPC_unrestrict(A,B,X_0,U_init,ref,Hp,Hu,P,Q,R);

    A_u_min = -tril(ones(size(U_init,1)*Hu));
    A_u_max = tril(ones(size(U_init,1)*Hu));
    
    f1_u_kron = ones(Hu,1);
    f1_u_min = kron(f1_u_kron, -(U_min - U_init));
    f1_u_max = kron(f1_u_kron, (U_max - U_init));
    
    A_x_min = -THETA;
    A_x_max = THETA;
    
    f1_x_kron = ones(Hp,1);
    f1_x_min = - (kron(f1_x_kron, x_min) - PSI*X_0 + GAMMA*U_init);
    f1_x_max = kron(f1_x_kron, x_max) - PSI*X_0 + GAMMA*U_init;
    
    A_del_u_min = -eye(size(U_init,1)*Hu);
    A_del_u_max = eye(size(U_init,1)*Hu);
    
    f1_del_u_min = kron(f1_u_kron, -del_u_min);
    f1_del_u_max = kron(f1_u_kron, del_u_max);
    
    A_restriction = [A_u_min; A_u_max; A_x_min; A_x_max; A_del_u_min; A_del_u_max];
    f1_restriction = [f1_u_min; f1_u_max; f1_x_min; f1_x_max; f1_del_u_min; f1_del_u_max];
    
    % U_rest = quadprog(2*H, -G, A_restriction, f1_restriction,[],[], f1_del_u_min, f1_del_u_max);
    U_rest = quadprog(2*H, -G, A_restriction, f1_restriction);

end