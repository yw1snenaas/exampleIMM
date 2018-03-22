function J = df2_2( x )
T = 1;
w2 = 3/180*pi;
J = [1, 0, 0, sin(w2*T)/w2, (cos(w2*T) - 1)/w2, 0;
    0, 1, 0, (1 - cos(w2*T))/w2, sin(w2*T)/w2, 0;
    0, 0, 1, 0, 0, 1;
    0, 0, 0, cos(w2*T), -sin(w2*T), 0;
    0, 0, 0, sin(w2*T), cos(w2*T), 0;
    0, 0, 0, 0, 0, 1];
end

