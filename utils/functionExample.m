clc; clear; close all;

[t,x] = plant(100, [0, 0, 0, 0], 0, 2);

function [t,x] = plant(u, x0, t, step)
    s = 9;
    delta = 0.009;
    betha = 4e-6;
    mu = 0.3;
    w50 = 89.6;
    k = 80;
    c = 0.6;
    Ku = 8.4;

    F = @(t,x) [s - delta * x(1) - betha * x(1) * x(3); betha * x(1) * x(3) - mu * x(2); (1-(x(4)/(x(4)+w50)))*k*x(2) - c*x(3); -Ku * x(4) + u];
    [t,x] = ode45(F,[t,t+step],x0);
end