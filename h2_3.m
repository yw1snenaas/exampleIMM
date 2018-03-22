function y = h2_3( x, v )
    if nargin<2
        H = [eye(3,3), zeros(3,3)];
        y = H * x;
    else
        H = [eye(3,3), zeros(3,3)];
        y = H * x;
        y = y + v * randn();
    end
end

