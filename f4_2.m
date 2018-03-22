function x_new = f4_2( x, w )
    if nargin < 2
        tau = 0.5;
        T = 1;
        A4_2 = [1, 0, 0, T, 0, 0, tau^(-2)*(exp(tau*T) - tau*T - 1), 0, 0;
            0, 1, 0, 0, T, 0, 0, tau^(-2)*(exp(tau*T) - tau*T - 1), 0;
            0, 0, 1, 0, 0, T, 0, 0, tau^(-2)*(exp(tau*T) - tau*T - 1);
            0, 0, 0, 1, 0, 0, tau^(-1)*(exp(tau*T) - 1), 0, 0;
            0, 0, 0, 0, 1, 0, 0, tau^(-1)*(exp(tau*T) - 1), 0;
            0, 0, 0, 0, 0, 1, 0, 0, tau^(-1)*(exp(tau*T) - 1);
            0, 0, 0, 0, 0, 0, exp(tau*T), 0, 0;
            0, 0, 0, 0, 0, 0, 0, exp(tau*T), 0;
            0, 0, 0, 0, 0, 0, 0, 0, exp(tau*T)];
        x_new = A4_2 * x;
    else
        tau = 0.5;
        T = 1;
        A4_2 = [1, 0, 0, T, 0, 0, tau^(-2)*(exp(tau*T) - tau*T - 1), 0, 0;
            0, 1, 0, 0, T, 0, 0, tau^(-2)*(exp(tau*T) - tau*T - 1), 0;
            0, 0, 1, 0, 0, T, 0, 0, tau^(-2)*(exp(tau*T) - tau*T - 1);
            0, 0, 0, 1, 0, 0, tau^(-1)*(exp(tau*T) - 1), 0, 0;
            0, 0, 0, 0, 1, 0, 0, tau^(-1)*(exp(tau*T) - 1), 0;
            0, 0, 0, 0, 0, 1, 0, 0, tau^(-1)*(exp(tau*T) - 1);
            0, 0, 0, 0, 0, 0, exp(tau*T), 0, 0;
            0, 0, 0, 0, 0, 0, 0, exp(tau*T), 0;
            0, 0, 0, 0, 0, 0, 0, 0, exp(tau*T)];
        x_new = A4_2 * x;
        x_new = x_new + w*randn();
    end
end

