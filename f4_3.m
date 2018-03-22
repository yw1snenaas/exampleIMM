function x_new = f4_3( x, w )
    if nargin < 2
        T = 1;
        x_new = zeros(7,1);
        x_new(1) = x(1) + T*x(4) - 0.5 * T^2 * x(5) * x(7);
        x_new(2) = x(2) + T*x(5) + 0.5 * T^2 * x(4) * x(7);
        x_new(3) = x(3) + T*x(6);
        x_new(4) = (1 - 0.5 * T^2 * x(7) ^ 2) * x(4) - T*x(5) * x(7);
        x_new(5) = (1 - 0.5 * T^2 * x(7)^2) * x(5) + T*x(4) * x(7);
        x_new(6) = x(6);
        x_new(7) = x(7);
    else
        T = 1;
        x_new = zeros(7,1);
        x_new(1) = x(1) + T*x(4) - 0.5 * T^2 * x(5) * x(7);
        x_new(2) = x(2) + T*x(5) + 0.5 * T^2 * x(4) * x(7);
        x_new(3) = x(3) + T*x(6);
        x_new(4) = (1 - 0.5 * T^2 * x(7) ^ 2) * x(4) - T*x(5) * x(7);
        x_new(5) = (1 - 0.5 * T^2 * x(7)^2) * x(5) + T*x(4) * x(7);
        x_new(6) = x(6);
        x_new(7) = x(7);
        x_new = x_new + w * randn();
    end
end

