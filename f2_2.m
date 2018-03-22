function x_new = f2_2( x, w )
    if nargin < 2
        w2 = 3/180*pi;
        T = 1;
        A2_2 = [1, 0, 0, sin(w2*T)/w2, (cos(w2*T) - 1)/w2, 0;
            0, 1, 0, (1 - cos(w2*T))/w2, sin(w2*T)/w2, 0;
            0, 0, 1, 0, 0, 1;
            0, 0, 0, cos(w2*T), -sin(w2*T), 0;
            0, 0, 0, sin(w2*T), cos(w2*T), 0;
            0, 0, 0, 0, 0, 1];
        x_new = A2_2 * x;
    else
        w2 = 3/180*pi;
        T = 1;
        A2_2 = [1, 0, 0, sin(w2*T)/w2, (cos(w2*T) - 1)/w2, 0;
            0, 1, 0, (1 - cos(w2*T))/w2, sin(w2*T)/w2, 0;
            0, 0, 1, 0, 0, 1;
            0, 0, 0, cos(w2*T), -sin(w2*T), 0;
            0, 0, 0, sin(w2*T), cos(w2*T), 0;
            0, 0, 0, 0, 0, 1];
        x_new = A2_2 * x;
        x_new = x_new + w * randn();
    end
end

