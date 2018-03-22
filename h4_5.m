function y = h4_5( x, v )
    if nargin < 2
        H = [eye(3,3), zeros(3,6)];
        y = H * x;
    else
        H = [eye(3,3), zeros(3,6)];
        y = H * x;
        y = y + randn()*v;
    end
end

