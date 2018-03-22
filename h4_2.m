function y = h4_2( x, v )
    if nargin < 2
        H = [eye(3,3), zeros(3,6)];
        y = H * x;
    else
        H = [eye(3,3), zeros(3,6)];
        y = H * x;
        y = y + v*randn();
    end
end

