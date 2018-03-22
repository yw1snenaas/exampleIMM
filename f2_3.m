function x_new = f2_3( x,w )
    if nargin < 2
        w3 = -3*pi/180;
        T = 1;
        A2_3 = [1, 0, 0, sin(w3*T)/w3, (cos(w3*T) - 1)/w3, 0;
            0, 1, 0, (1 - cos(w3*T))/w3, sin(w3*T)/w3, 0;
            0, 0, 1, 0, 0, 1;
            0, 0, 0, cos(w3*T), -sin(w3*T), 0;
            0, 0, 0, sin(w3*T), cos(w3*T), 0;
            0, 0, 0, 0, 0, 1];
        x_new = A2_3 * x;
    else
        w3 = -3*pi/180;
        T = 1;
        A2_3 = [1, 0, 0, sin(w3*T)/w3, (cos(w3*T) - 1)/w3, 0;
            0, 1, 0, (1 - cos(w3*T))/w3, sin(w3*T)/w3, 0;
            0, 0, 1, 0, 0, 1;
            0, 0, 0, cos(w3*T), -sin(w3*T), 0;
            0, 0, 0, sin(w3*T), cos(w3*T), 0;
            0, 0, 0, 0, 0, 1];
        x_new = A2_3 * x;
        x_new = x_new + w * randn();
    end
end

