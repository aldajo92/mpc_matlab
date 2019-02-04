clc; clear; close all;
%% 
Ts = 1;
Tref = 10;

e_k_0 = 10;

n = 0:1:(Tref/Ts);
e_k = zeros(size(i));

lambda = exp(-Ts/Tref);

for i = 0:1:(Tref/Ts)
    e_k(i+1) = (lambda^i) * e_k_0;
    e_k_0 = e_k(i+1);
end

plot(n, e_k)

%% define s
r_k_plus_i_given_k = zeros(size(i));

for k = 0:1:(Tref/Ts)
    r_k_plus_i_given_k(k+1) = s_ref(k) - e_k(k+1);
end

figure()
plot(n, r_k_plus_i_given_k)

%%
function y = s_ref(x)
    y = 0*x + 5;
end