function J = df2_3( x )
T = 1;
w3 = -3*pi/180;
J = [1, 0, 0, sin(w3*T)/w3, (cos(w3*T) - 1)/w3, 0;
    0, 1, 0, (1 - cos(w3*T))/w3, sin(w3*T)/w3, 0;
    0, 0, 1, 0, 0, 1;
    0, 0, 0, cos(w3*T), -sin(w3*T), 0;
    0, 0, 0, sin(w3*T), cos(w3*T), 0;
    0, 0, 0, 0, 0, 1];
end

