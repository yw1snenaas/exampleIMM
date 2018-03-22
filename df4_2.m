function J = df4_2( x )
T = 1;
tau = 0.5;
J = [1, 0, 0, T, 0, 0, tau^(-2)*(exp(tau*T) - tau*T - 1), 0, 0;
    0, 1, 0, 0, T, 0, 0, tau^(-2)*(exp(tau*T) - tau*T - 1), 0;
    0, 0, 1, 0, 0, T, 0, 0, tau^(-2)*(exp(tau*T) - tau*T - 1);
    0, 0, 0, 1, 0, 0, tau^(-1)*(exp(tau*T) - 1), 0, 0;
    0, 0, 0, 0, 1, 0, 0, tau^(-1)*(exp(tau*T) - 1), 0;
    0, 0, 0, 0, 0, 1, 0, 0, tau^(-1)*(exp(tau*T) - 1);
    0, 0, 0, 0, 0, 0, exp(tau*T), 0, 0;
    0, 0, 0, 0, 0, 0, 0, exp(tau*T), 0;
    0, 0, 0, 0, 0, 0, 0, 0, exp(tau*T)];
end

