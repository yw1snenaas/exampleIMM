function J = df3_1( x )
T = 1;
J = [1, 0, 0, T, 0, 0;
    0, 1, 0, 0, T, 0;
    0, 0, 1, 0, 0, T;
    0, 0, 0, 1, 0, 0;
    0, 0, 0, 0, 1, 0;
    0, 0, 0, 0, 0, 1];
end

