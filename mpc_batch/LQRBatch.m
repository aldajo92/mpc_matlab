function [u, U] = mpcBatch(R,Q,S,A,B,N,X_0)
    [~, ~, ~, H, F, ~] = expandSystem(R,Q,S,A,B,N);
    U = -inv(H)*F'*X_0;
    u = U(1:size(B,2));
end