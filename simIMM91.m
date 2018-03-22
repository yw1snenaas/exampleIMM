% Referrence:
% Tracking a 3D Maneuvering Target With Passive Sensors
clear all;
clc;
close all;
% Initializing
T = 1;          % Sample period
Time = 147;
% Model 1.1: For flight segments with constant speed, a state vector of
% order 6 (n = 6) is used
% x = [X1,X2,X3,dX1,dX2,dX2]
% the discrete-time dynamics matrix:
A1_1 = [1, 0, 0, T, 0, 0;
    0, 1, 0, 0, T, 0;
    0, 0, 1, 0, 0, T;
    0, 0, 0, 1, 0, 0;
    0, 0, 0, 0, 1, 0;
    0, 0, 0, 0, 0, 1];
% the process noise covariance matrix:
sigmaV1_1 = 5;
sigma2V1_1 = sigmaV1_1^2;
R1_1 = [1/4*T^4, 0, 0, 1/2*T^3, 0, 0;
    0, 1/4*T^2, 0, 0, 1/2*T^3, 0;
    0, 0, 1/4*T^2, 0, 0, 1/2*T^3;
    1/2*T^3, 0, 0, T^2, 0, 0;
    0, 1/2*T^3, 0, 0, T^2, 0;
    0, 0, 1/2*T^3, 0, 0, T^2]*sigma2V1_1;
% Model 1.2: For flight segments where the target maneuvers, a state vector
% of order 8 (n = 8) is considered
% x = [X1, X2, X3, dX1, dX2, dX3, ddX1, ddX2]
% the discrete-time dynamics matrix
A1_2 = [1, 0, 0, T, 0, 0, 1/2*T^2, 0;
    0, 1, 0, 0, T, 0, 0, 1/2*T^2;
    0, 0, 1, 0, 0, T, 0, 0;
    0, 0, 0, 1, 0, 0, T, 0;
    0, 0, 0, 0, 1, 0, 0, T;
    0, 0, 0, 0, 0, 1, 0, 0;
    0, 0, 0, 0, 0, 0, 1, 0;
    0, 0, 0, 0, 0, 0, 0, 1];
% the process noise covariance matrix:
sigmaV1_2 = 7.5;
sigma2V1_2 = sigmaV1_2^2;
R1_2 = [1/4*T^4, 0, 0, 1/2*T^3, 0, 0, 1/2*T^2, 0;
    0, 1/4*T^4, 0, 0, 1/2*T^3, 0, 0, 1/2*T^2;
    0, 0, 1/4*T^4, 0, 0, 1/2*T^3, 0, 0;
    1/2*T^3, 0, 0, T^2, 0, 0, T, 0;
    0, 1/2*T^3, 0, 0, T^2, 0, 0, T;
    0, 0, 1/2*T^3, 0, 0, T^2, 0, 0;
    1/2*T^2, 0, 0, T, 0, 0, 1, 0;
    0, 1/2*T^2, 0, 0, T, 0, 0, 1]*sigma2V1_2;
% Model 1.3: A third model is used, with the same variables as the second
% model but a larger sigma_v.
sigmaV1_3 = 40;
sigma2V1_3 = sigmaV1_3^2;
A1_3 = A1_2;
R1_3 = [1/4*T^4, 0, 0, 1/2*T^3, 0, 0, 1/2*T^2, 0;
    0, 1/4*T^4, 0, 0, 1/2*T^3, 0, 0, 1/2*T^2;
    0, 0, 1/4*T^4, 0, 0, 1/2*T^3, 0, 0;
    1/2*T^3, 0, 0, T^2, 0, 0, T, 0;
    0, 1/2*T^3, 0, 0, T^2, 0, 0, T;
    0, 0, 1/2*T^3, 0, 0, T^2, 0, 0;
    1/2*T^2, 0, 0, T, 0, 0, 1, 0;
    0, 1/2*T^2, 0, 0, T, 0, 0, 1]*sigma2V1_3;
% Model 2.1: For flight segments with constant average speed model 1.1 is
% again selected.
A2_1 = A1_1;
R2_1 = R1_1;
% Model 2.2: For turns the exact kinematics of a mobile turning with a
% constant angular rate w are used.
% x = [X1, X2, X3, dX1, dX2, dX3]
sigmaV2_2 = sqrt(500);
sigma2V2_2 = 500;
w2 = 3/180*pi;
A2_2 = [1, 0, 0, sin(w2*T)/w2, (cos(w2*T) - 1)/w2, 0;
    0, 1, 0, (1 - cos(w2*T))/w2, sin(w2*T)/w2, 0;
    0, 0, 1, 0, 0, 1;
    0, 0, 0, cos(w2*T), -sin(w2*T), 0;
    0, 0, 0, sin(w2*T), cos(w2*T), 0;
    0, 0, 0, 0, 0, 1];
R2_2 = [1/4*T^4, 0, 0, 1/2*T^3, 0, 0;
    0, 1/4*T^4, 0, 0, 1/2*T^3, 0;
    0, 0, 1/4*T^4, 0, 0, 1/2*T^3;
    1/2*T^3, 0, 0, T^2, 0, 0;
    0, 1/2*T^3, 0, 0, T^2, 0;
    0, 0, 1/2*T^3, 0, 0, T^2]*sigma2V2_2;
% Model 2.3: For w > 0 model 2.2 describes a counterclockwise turn, and
% model 2.3 is its natural counterpart for a clockwise run.
w3 = -3*pi/180;
sigmaV2_3 = sqrt(500);
sigma2V2_3 = 500;
A2_3 = [1, 0, 0, sin(w3*T)/w3, (cos(w3*T) - 1)/w3, 0;
    0, 1, 0, (1 - cos(w3*T))/w3, sin(w3*T)/w3, 0;
    0, 0, 1, 0, 0, 1;
    0, 0, 0, cos(w3*T), -sin(w3*T), 0;
    0, 0, 0, sin(w3*T), cos(w3*T), 0;
    0, 0, 0, 0, 0, 1];
R2_3 = [1/4*T^4, 0, 0, 1/2*T^3, 0, 0;
    0, 1/4*T^4, 0, 0, 1/2*T^3, 0;
    0, 0, 1/4*T^4, 0, 0, 1/2*T^3;
    1/2*T^3, 0, 0, T^2, 0, 0;
    0, 1/2*T^3, 0, 0, T^2, 0;
    0, 0, 1/2*T^3, 0, 0, T^2]*sigma2V2_3;
% Model 3.2: For flight segments with constant speed, a model similar to
% model 1.1 si again selected, but the noise input is modified. Whrereas in
% model 1.1 we described speed as constant on the average here we consider
% a constant speed.
A3_1 = A1_1;
sigmaV3_1 = sqrt(250);
sigma2V3_1 = 250;
R3_1 = [1, 0, 0, 0, 0, 0;
    0, 1, 0, 0, 0, 0;
    0, 0, 1, 0, 0, 0;
    0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0]*sigma2V3_1;
% Model 3.2: We include thet rotation rate w into the state vector
% x = [X1, X2, X3, dX1, dX2, dX3, w]
% A3_2 = func_model32.m
sigma_w3_2 = 0.05;
sigma2w3_2 = sigma_w3_2^2;
sigmaV3_2 = sqrt(250);
sigma2V3_2 = 250;
R3_2 = [sigma2V3_2, 0, 0, 0, 0, 0, 0;
    0, sigma2V3_2, 0, 0, 0, 0, 0;
    0, 0, sigma2V3_2, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, sigma2w3_2];
% Model 4:
% Referrence:
% G. A. Watson, W. D. Blair. IMM algorithm for tracking targets that
% maneuver through coordinated turns
% Model 4.1:
% CA model:
% x = [X1, X2, X3, dX1, dX2, dX3, ddX1, ddX2, ddX3]
A4_1 = [1, 0, 0, T, 0, 0, T^2/2, 0, 0;
    0, 1, 0, 0, T, 0, 0, T^2/2, 0;
    0, 0, 1, 0, 0, T, 0, 0, T^2/2;
    0, 0, 0, 1, 0, 0, T, 0, 0;
    0, 0, 0, 0, 1, 0, 0, T, 0;
    0, 0, 0, 0, 0, 1, 0, 0, T;
    0, 0, 0, 0, 0, 0, 1, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 1, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 1];
sigma_w4_1 = 10;
sigma2w4_1 = sigma_w4_1^2;
R4_1 = [eye(3,3)*T^4/4, eye(3,3)*T^3/2, eye(3,3)*T^2/2;
    eye(3,3)*T^3/2, eye(3,3)*T^2, eye(3,3)*T;
    eye(3,3)*T^2/2, eye(3,3)*T, eye(3,3)]*sigma2w4_1;
% Model 4.2:
% Exponentially Increasing Acceleration (EIA) Model
% x = [X1, X2, X3, dX1, dX2, dX3, ddX1, ddX2, ddX3]
tau = 0.5;
A4_2 = [1, 0, 0, T, 0, 0, tau^(-2)*(exp(tau*T) - tau*T - 1), 0, 0;
    0, 1, 0, 0, T, 0, 0, tau^(-2)*(exp(tau*T) - tau*T - 1), 0;
    0, 0, 1, 0, 0, T, 0, 0, tau^(-2)*(exp(tau*T) - tau*T - 1);
    0, 0, 0, 1, 0, 0, tau^(-1)*(exp(tau*T) - 1), 0, 0;
    0, 0, 0, 0, 1, 0, 0, tau^(-1)*(exp(tau*T) - 1), 0;
    0, 0, 0, 0, 0, 1, 0, 0, tau^(-1)*(exp(tau*T) - 1);
    0, 0, 0, 0, 0, 0, exp(tau*T), 0, 0;
    0, 0, 0, 0, 0, 0, 0, exp(tau*T), 0;
    0, 0, 0, 0, 0, 0, 0, 0, exp(tau*T)];
sigma_w4_2 = 10;
sigma2w4_2 = sigma_w4_2^2;
G = [tau^(-3) * (exp(tau*T) - 1- tau*T - 0.5*(tau*T)^2) * eye(3,3);
    tau^(-2)*(exp(tau*T) - 1 - tau*T)*eye(3,3);
    tau^(-1)*(exp(tau*T) - 1)*eye(3,3)];
R4_2 = G*(G')*sigma2w4_2;
% Model 4.3:
% Turning Rate Estimator (TRE) Model
% x = [X1, X2, X3, dX1, dX2, dX3, w]
% A4_3 = f4_3
Bk = [T^4/4, T^3/2;
    T^3/2, T^2];
sigma_v4_3 = 10;
sigma_w4_3 = 0.0005;
sigma2v4_3 = sigma_v4_3^2;
sigma2w4_3 = sigma_w4_3^2;
R4_3 = [Bk, zeros(2,2),zeros(2,2),zeros(2,1);
    zeros(2,2), Bk, zeros(2,2), zeros(2,1);
    zeros(2,2), zeros(2,2), Bk, zeros(2,1);
    zeros(1,2), zeros(1,2),zeros(1,2), sigma2w4_3*T^2/sigma2v4_3]*sigma2v4_3;
% Model 4.4
% 3DTR model:
w4_4 = 3/180*pi;
A4_4 = [1, 0, 0, sin(w4_4*T)/w4_4, 0, 0, (1 - cos(w4_4*T))/(w4_4^2), 0, 0;
    0, 1, 0, 0, sin(w4_4*T)/w4_4, 0, 0, (1 - cos(w4_4*T))/(w4_4^2), 0;
    0, 0, 1, 0, 0, sin(w4_4*T)/w4_4, 0, 0, (1 - cos(w4_4*T))/(w4_4^2);
    0, 0, 0, cos(w4_4*T), 0, 0, sin(w4_4*T)/w4_4, 0, 0;
    0, 0, 0, 0, cos(w4_4*T), 0, 0, sin(w4_4*T)/w4_4, 0;
    0, 0, 0, 0, 0, cos(w4_4*T), 0, 0, sin(w4_4*T)/w4_4;
    0, 0, 0, -sin(w4_4*T)*w4_4, 0, 0, cos(w4_4*T), 0, 0;
    0, 0, 0, 0, -sin(w4_4*T)*w4_4, 0, 0, cos(w4_4*T), 0;
    0, 0, 0, 0, 0, -sin(w4_4*T)*w4_4, 0, 0, cos(w4_4*T)];
sigma_w4_4 = 10;
sigma2w4_4 = sigma_w4_4^2;
R4_4 = [eye(3,3)*T^4/4, eye(3,3)*T^3/2, eye(3,3)*T^2/2;
    eye(3,3)*T^3/2, eye(3,3)*T^2, eye(3,3)*T;
    eye(3,3)*T^2/2, eye(3,3)*T, eye(3,3)]*sigma2w4_4;
% Model 4.5
% 3DTR model:
w4_5 = -3/180*pi;
A4_5 = [1, 0, 0, sin(w4_5*T)/w4_5, 0, 0, (1 - cos(w4_5*T))/(w4_5^2), 0, 0;
    0, 1, 0, 0, sin(w4_5*T)/w4_5, 0, 0, (1 - cos(w4_5*T))/(w4_5^2), 0;
    0, 0, 1, 0, 0, sin(w4_5*T)/w4_5, 0, 0, (1 - cos(w4_5*T))/(w4_5^2);
    0, 0, 0, cos(w4_5*T), 0, 0, sin(w4_5*T)/w4_5, 0, 0;
    0, 0, 0, 0, cos(w4_5*T), 0, 0, sin(w4_5*T)/w4_5, 0;
    0, 0, 0, 0, 0, cos(w4_5*T), 0, 0, sin(w4_5*T)/w4_5;
    0, 0, 0, -sin(w4_5*T)*w4_5, 0, 0, cos(w4_5*T), 0, 0;
    0, 0, 0, 0, -sin(w4_5*T)*w4_5, 0, 0, cos(w4_5*T), 0;
    0, 0, 0, 0, 0, -sin(w4_5*T)*w4_5, 0, 0, cos(w4_5*T)];
sigma_w4_5 = 10;
sigma2w4_5 = sigma_w4_5^2;
R4_5 = [eye(3,3)*T^4/4, eye(3,3)*T^3/2, eye(3,3)*T^2/2;
    eye(3,3)*T^3/2, eye(3,3)*T^2, eye(3,3)*T;
    eye(3,3)*T^2/2, eye(3,3)*T, eye(3,3)]*sigma2w4_5;
% Real state:
t = 0:T:Time;
X_real = zeros(6,length(t));
A1 = [1, 0, 0, T, 0, 0;
    0, 1, 0, 0, T, 0;
    0, 0, 1, 0, 0, T;
    0, 0, 0, 1, 0, 0;
    0, 0, 0, 0, 1, 0;
    0, 0, 0, 0, 0, 1];
w4 = -3/180*pi;
A2 = [1, 0, 0, sin(w4)/w4, (cos(w4) - 1)/w4, 0;
    0, 1, 0, (1 - cos(w4))/w4, sin(w4)/w4, 0;
    0, 0, 1, 0, 0, 1;
    0, 0, 0, cos(w4), -sin(w4), 0;
    0, 0, 0, sin(w4), cos(w4), 0;
    0, 0, 0, 0, 0, 1];
w5 = 3/180*pi;
A3 = [1, 0, 0, sin(w5)/w5, (cos(w5) - 1)/w5, 0;
    0, 1, 0, (1 - cos(w5))/w5, sin(w5)/w5, 0;
    0, 0, 1, 0, 0, 1;
    0, 0, 0, cos(w5), -sin(w5), 0;
    0, 0, 0, sin(w5), cos(w5), 0;
    0, 0, 0, 0, 0, 1];
% measurement parameter
measure = zeros(3,length(t));
sigmaR = 150;
sigmaTheta = 0.004;
sigmaPhi = 0.004;
% filtering parameter
% x1_1IMM = zeros(6,1);
% x1_2IMM = zeros(8,1);
% x1_3IMM = zeros(8,1);
% x2_1IMM = zeros(6,1);
% x2_2IMM = zeros(6,1);
% x2_3IMM = zeros(6,1);
% x3_1IMM = zeros(6,1);
% x3_2IMM = zeros(7,1);
% x4_1IMM = zeros(9,1);
% x4_2IMM = zeros(9,1);
% x4_3IMM = zeros(7,1);
% x4_4IMM = zeros(9,1);
% x4_4IMM = zeros(9,1);
x1_pro_IMM = zeros(6,length(t));
x2_pro_IMM = zeros(6,length(t));
x3_pro_IMM = zeros(6,length(t));
x4_pro_IMM = zeros(6,length(t));
% maneuvering target trajectory
for k = 1:length(t)
    if t(k) == 0
        X_real(1,1) = 10000;
        X_real(2,1) = 15000;
        X_real(3,1) = 200;
        X_real(4,1) = -300;
        X_real(5,1) = 0;
        X_real(6,1) = 0;
    elseif (t(k) > 0) && (t(k) <= 23)
        X_real(:,k) = A1*X_real(:,k-1);     % 1st segment.
    elseif (t(k)>23) && (t(k) <= 38)
        X_real(:,k) = A2*X_real(:,k-1);     % 2nd segment.
    elseif (t(k) > 38) &&(t(k) <= 60)
        X_real(:,k) = A1*X_real(:,k-1);     % 3rd segment.
    elseif (t(k) >60) && (t(k) <= 75)
        X_real(:,k) = A3*X_real(:,k-1);     % 4th segment.
    elseif (t(k) > 75) && (t(k) <= 90)
        X_real(:,k) = A1*X_real(:,k-1);     % 5th segment.
    elseif (t(k) > 90) && (t(k) <= 105)
        X_real(:,k) = A2*X_real(:,k-1);     % 6th segment.
    else
        X_real(:,k) = A1*X_real(:,k-1);     % 7th segment.
    end
    % measurement
    [theta, phi, r] = cart2sph(X_real(1,k),X_real(2,k),X_real(3,k));
    r = r + randn() * sigmaR;
    theta = theta + randn() * sigmaTheta;
    phi = phi + randn() * sigmaPhi;
    measure(:,k) = [r;theta;phi];
end
% Filter initialize
x1_pro_IMM(:,1) = X_real(:,1);
x2_pro_IMM(:,1) = X_real(:,1);
x3_pro_IMM(:,1) = X_real(:,1);
x4_pro_IMM(:,1) = X_real(:,1);
x1_1IMM = X_real(:,1);
x1_2IMM = [X_real(:,1);0;0];
x1_3IMM = [X_real(:,1);0;0];
x2_1IMM = X_real(:,1);
x2_2IMM = X_real(:,1);
x2_3IMM = X_real(:,1);
x3_1IMM = X_real(:,1);
x3_2IMM = [X_real(:,1);0];
x4_1IMM = [X_real(:,1);zeros(3,1)];
x4_2IMM = [X_real(:,1);zeros(3,1)];
x4_3IMM = [X_real(:,1);0];
x4_4IMM = [X_real(:,1);zeros(3,1)];
x4_5IMM = [X_real(:,1);zeros(3,1)];
P1_1IMM = zeros(6,6);
P1_2IMM = zeros(8,8);
P1_3IMM = zeros(8,8);
P2_1IMM = zeros(6,6);
P2_2IMM = zeros(6,6);
P2_3IMM = zeros(6,6);
P3_1IMM = zeros(6,6);
P3_2IMM = zeros(7,7);
P4_1IMM = zeros(9,9);
P4_2IMM = zeros(9,9);
P4_3IMM = zeros(7,7);
P4_4IMM = zeros(9,9);
P4_5IMM = zeros(9,9);
P1 = zeros(6,6);
P2 = zeros(6,6);
P3 = zeros(6,6);
P4 = zeros(9,9);
u1_IMM = zeros(3,length(t));
u2_IMM = zeros(3,length(t));
u3_IMM = zeros(2,length(t));
u4_IMM = zeros(5,length(t));
u1_IMM(:,1) = [0.3 0.4 0.3]';
u2_IMM(:,1) = [0.3 0.4 0.3]';
u3_IMM(:,1) = [0.5 0.5]';
u4_IMM(:,1) = [0.2 0.2 0.2 0.2 0.2]';
Hij1 = [0.9, 0.05, 0.05;
        0.1, 0.8, 0.1;
        0.05, 0.15, 0.8];
Hij2 = Hij1;
Hij3 = [0.9, 0.1;
    0.1, 0.9];
Hij4 = [0.8 0.05 0.05 0.05 0.05;
    0.1 0.6 0.1 0.1 0.1;
    0.02 0.02 0.92 0.02 0.02;
    0.1 0.1 0.1 0.6 0.1;
    0.05 0.05 0.05 0.05 0.8];
% Filtering process
for k = 2:length(t)
    c1_i = Hij1'*u1_IMM(:,k-1);      % Predict each model's probability of Model 1
    c2_i = Hij2'*u2_IMM(:,k-1);      % Predict each model's probability of Model 2
    c3_i = Hij3'*u3_IMM(:,k-1);      % Predict each model's probability of Model 3
    c4_i = Hij4'*u4_IMM(:,k-1);
    % Model 1
    x1_1 = (Hij1(1,1)*u1_IMM(1,k-1)*x1_1IMM(1:6,1) + Hij1(2,1)*u1_IMM(2,k-1)*x1_2IMM(1:6,1) +...
        Hij1(3,1) * u1_IMM(3,k-1) *x1_3IMM(1:6,1))/c1_i(1);
    x1_2 = (Hij1(1,2)*u1_IMM(1,k-1) * [x1_1IMM; 0; 0] + Hij1(2,2)*u1_IMM(2,k-1)*x1_2IMM + ...
        Hij1(3,2) * u1_IMM(3,k-1) * x1_3IMM)/c1_i(2);
    x1_3 = (Hij1(1,3)*u1_IMM(1,k-1) * [x1_1IMM; 0; 0] + Hij1(2,3)*u1_IMM(2,k-1) * x1_2IMM +...
        Hij1(3,3) * u1_IMM(3,k-1) * x1_3IMM)/c1_i(3);
    % Model 2
    x2_1 = (Hij2(1,1)*u2_IMM(1,k-1)*x2_1IMM + Hij2(2,1)*u2_IMM(2,k-1) * x2_2IMM + ...
        Hij2(3,1) * u2_IMM(3,k-1) * x2_3IMM)/c2_i(1);
    x2_2 = (Hij2(1,2) * u2_IMM(1,k-1) * x2_1IMM + Hij2(2,2) * u2_IMM(2,k-1) * x2_2IMM + ...
        Hij2(3,2) * u2_IMM(3,k-1) * x2_3IMM)/c2_i(2);
    x2_3 = (Hij2(1,3) * u2_IMM(1,k-1) * x2_1IMM + Hij2(2,3) * u2_IMM(2,k-1) * x2_2IMM + ...
        Hij2(3,3) * u2_IMM(3,k-1) * x2_3IMM)/c2_i(3);
    % Model 3
    x3_1 = (Hij3(1,1) * u3_IMM(1,k-1) * x3_1IMM + Hij3(2,1) * u3_IMM(2,k-1) * x3_2IMM(1:6,1))/c3_i(1);
    x3_2 = (Hij3(1,2) * u3_IMM(1,k-1) * [x3_1IMM; 0] + Hij3(2,2) * u3_IMM(2,k-1) * x3_2IMM)/c3_i(2);
    % Model 4
    x4_1 = (Hij4(1,1) * u4_IMM(1,k-1) * x4_1IMM + Hij4(2,1) * u4_IMM(2,k-1) * x4_2IMM + ...
        Hij4(3,1) * u4_IMM(3,k-1) * [x4_3IMM(1:6,1);0;0;0] + Hij4(4,1) * u4_IMM(4,k-1) * x4_4IMM + ...
        Hij4(5,1) * u4_IMM(5,k-1) * x4_5IMM) / c4_i(1);
    x4_2 = (Hij4(1,2) * u4_IMM(1,k-1) * x4_1IMM + Hij4(2,2) * u4_IMM(2,k-1) * x4_2IMM +...
        Hij4(3,2) * u4_IMM(3,k-1) * [x4_3IMM(1:6,1);0;0;0] + Hij4(4,2) * u4_IMM(4,k-1) * x4_4IMM + ...
        Hij4(5,2) * u4_IMM(5, k-1) * x4_5IMM) / c4_i(2);
    x4_3 = (Hij4(1,3) * u4_IMM(1,k-1) * [x4_1IMM(1:6);0] + Hij4(2,3) * u4_IMM(2, k-1) *[x4_2IMM(1:6);0] + ...
        Hij4(3,3) * u4_IMM(3,k-1) * x4_3IMM + Hij4(4,3) * u4_IMM(4,k-1) * [x4_4IMM(1:6,1);0] + ...
        Hij4(5,3) * u4_IMM(5,k-1) * [x4_5IMM(1:6,1);0]) / c4_i(3);
    x4_4 = (Hij4(1,4) * u4_IMM(1,k-1) * x4_1IMM + Hij4(2,4) * u4_IMM(2,k-1) * x4_2IMM + ...
        Hij4(3,4) * u4_IMM(3,k-1) * [x4_3IMM(1:6,1);0;0;0] + Hij4(4,4) * u4_IMM(4,k-1) * x4_4IMM + ...
        Hij4(5,4) * u4_IMM(5, k-1) * x4_5IMM) / c4_i(4);
    x4_5 = (Hij4(1,5) * u4_IMM(1,k-1) * x4_1IMM + Hij4(2,5) * u4_IMM(2,k-1) * x4_2IMM + ...
        Hij4(3,5) * u4_IMM(3,k-1) * [x4_3IMM(1:6,1);0;0;0] + Hij4(4,5) * u4_IMM(4,k-1) * x4_4IMM + ...
        Hij4(5,5) * u4_IMM(5,k-1) * x4_5IMM) / c4_i(5);
    % Model 1: Calculate the covarience matrix P
    P1_1 = (Hij1(1,1) * u1_IMM(1,k-1)*(P1_1IMM + (x1_1IMM - x1_1)*((x1_1IMM - x1_1)')) + ...
        Hij1(2,1) * u1_IMM(2,k-1) * (P1_2IMM(1:6,1:6) + (x1_2IMM(1:6,1) - x1_1)*((x1_2IMM(1:6,1) - x1_1)')) + ...
        Hij1(3,1) * u1_IMM(3,k-1) * (P1_3IMM(1:6,1:6) + (x1_3IMM(1:6,1) - x1_1)*((x1_3IMM(1:6,1) - x1_1)')) )/c1_i(1);
    P1_2 = (Hij1(1,2) * u1_IMM(1,k-1) * ([P1_1IMM, zeros(6,2);zeros(2,8)] + ([x1_1IMM;0;0] - x1_2) * (([x1_1IMM;0;0] - x1_2)')) + ...
        Hij1(2,2) * u1_IMM(2,k-1) * (P1_2IMM + (x1_2IMM - x1_2)*((x1_2IMM - x1_2)')) + ...
        Hij1(3,2) * u1_IMM(3,k-1) * (P1_3IMM + (x1_3IMM - x1_2) * ((x1_3IMM - x1_2)')) ) / c1_i(2);
    P1_3 = (Hij1(1,3) * u1_IMM(1,k-1) * ([P1_1IMM, zeros(6,2);zeros(2,8)] + ([x1_1IMM;0;0] - x1_3) * (([x1_1IMM;0;0] - x1_3)')) + ...
        Hij1(2,3) * u1_IMM(2,k-1) * (P1_2IMM + (x1_2IMM - x1_3) * ((x1_2IMM - x1_3)')) + ...
        Hij1(3,3) *u1_IMM(3,k-1) *(P1_3IMM + (x1_3IMM - x1_3) * ((x1_3IMM - x1_3)')) )/c1_i(3);
    % Model 2: Calculate the covarience matrix P
    P2_1 = (Hij2(1,1) * u2_IMM(1,k-1) * (P2_1IMM + (x2_1IMM - x2_1) * ((x2_1IMM - x2_1)')) +...
        Hij2(2,1) * u2_IMM(2,k-1) * (P2_2IMM + (x2_2IMM - x2_1) * ((x2_2IMM - x2_1)')) + ...
        Hij2(3,1) * u2_IMM(3,k-1) * (P2_3IMM + (x2_3IMM - x2_1) * ((x2_3IMM - x2_1)')) ) / c2_i(1);
    P2_2 = (Hij2(1,2) * u2_IMM(1,k-1) * (P2_1IMM + (x2_1IMM - x2_2) * ((x2_1IMM - x2_2)')) +...
        Hij2(2,2) * u2_IMM(2,k-1) * (P2_2IMM + (x2_2IMM - x2_2) * ((x2_2IMM - x2_2)')) + ...
        Hij2(3,2) * u2_IMM(3,k-1) * (P2_3IMM + (x2_3IMM - x2_2) * ((x2_3IMM - x2_2)'))) / c2_i(2);
    P2_3 = (Hij2(1,3) * u2_IMM(1,k-1) * (P2_1IMM + (x2_1IMM - x2_3) * ((x2_1IMM - x2_3)')) + ...
        Hij2(2,3) * u2_IMM(2,k-1) * (P2_2IMM + (x2_2IMM - x2_3) * ((x2_2IMM - x2_3)')) + ...
        Hij2(3,3) * u2_IMM(3,k-1) * (P2_3IMM + (x2_3IMM - x2_3) * ((x2_3IMM - x2_3)')) ) / c2_i(3);
    % Model 3: Calculate the covarience matrix P
    P3_1 = (Hij3(1,1) * u3_IMM(1,k-1) * (P3_1IMM + (x3_1IMM - x3_1) * ((x3_1IMM - x3_1)')) + ...
        Hij3(2,1) * u3_IMM(2,k-1) * (P3_2IMM(1:6,1:6) + (x3_2(1:6,1) - x3_1) * ((x3_2(1:6,1) - x3_1)'))) / c3_i(1);
    P3_2 = (Hij3(1,2) * u3_IMM(1,k-1) * ([P3_1IMM, zeros(6,1);zeros(1,7)] + ([x3_1IMM;0] - x3_2) * (([x3_1IMM;0] - x3_2)')) + ...
        Hij3(2,2) * u3_IMM(2,k-1) * (P3_2IMM + (x3_2IMM - x3_2) * ((x3_2IMM - x3_2)'))) / c3_i(2);
    % Model 4: Calculate the covarience matrix P
    P4_1 = (Hij4(1,1) * u4_IMM(1,k-1) *(P4_1IMM + (x4_1IMM - x4_1) * ((x4_1IMM - x4_1)')) + ...
        Hij4(2,1) * u4_IMM(2,k-1) * (P4_2IMM + (x4_2IMM - x4_1) * ((x4_2IMM - x4_1)')) + ...
        Hij4(3,1) * u4_IMM(3,k-1) * ([P4_3IMM(1:6,1:6), zeros(6,3);zeros(3,6), zeros(3,3)] + ([x4_3IMM(1:6,1);0;0;0] - x4_1) * (([x4_3IMM(1:6,1);0;0;0] - x4_1)')) + ...
        Hij4(4,1) * u4_IMM(4,k-1) * (P4_4IMM + (x4_4IMM - x4_1) * ((x4_4IMM - x4_1)')) + ...
        Hij4(5,1) * u4_IMM(5,k-1) * (P4_5IMM + (x4_5IMM - x4_1) * ((x4_5IMM - x4_1)'))) / c4_i(1);
    P4_2 = (Hij4(1,2) * u4_IMM(1,k-1) * (P4_1IMM + (x4_1IMM - x4_2) * ((x4_1IMM - x4_2)')) + ...
        Hij4(2,2) * u4_IMM(2,k-1) * (P4_2IMM + (x4_2IMM - x4_2) * ((x4_2IMM - x4_2)')) + ...
        Hij4(3,2) * u4_IMM(3,k-1) * ([P4_3IMM(1:6,1:6), zeros(6,3); zeros(3,9)] + ([x4_3IMM(1:6,1);zeros(3,1)] - x4_2) * (([x4_3IMM(1:6,1);zeros(3,1)] - x4_2)')) +...
        Hij4(4,2) * u4_IMM(4,k-1) * (P4_4IMM + (x4_4IMM - x4_2) * ((x4_4IMM - x4_2)')) + ...
        Hij4(5,2) * u4_IMM(5,k-1) * (P4_5IMM + (x4_5IMM - x4_2) * ((x4_5IMM - x4_2)'))) / c4_i(2);
    P4_3 = ( Hij4(1,3) * u4_IMM(1,k-1) * ([P4_1IMM(1:6,1:6) ,zeros(6,1);zeros(1,7)] + ([x4_1IMM(1:6,1);0] - x4_3) * (([x4_1IMM(1:6,1);0] - x4_3)')) +...
        Hij4(2,3) * u4_IMM(2,k-1) * ([P4_2IMM(1:6,1:6),zeros(6,1);zeros(1,7)] + ([x4_2IMM(1:6,1);0] - x4_3) * (([x4_2IMM(1:6,1);0] - x4_3)')) + ...
        Hij4(3,3) * u4_IMM(3,k-1) * (P4_3IMM + (x4_3IMM - x4_3) * ((x4_3IMM - x4_3)')) + ...
        Hij4(4,3) * u4_IMM(4,k-1) * ([P4_4IMM(1:6,1:6), zeros(6,1);zeros(1,7)] + ([x4_4IMM(1:6,1);0] - x4_3) * (([x4_4IMM(1:6,1);0] - x4_3)')) + ...
        Hij4(5,3) * u4_IMM(5,k-1) * ([P4_5IMM(1:6,1:6), zeros(6,1);zeros(1,7)] + ([x4_5IMM(1:6,1);0] - x4_3) * (([x4_5IMM(1:6,1);0] - x4_3)')))/c4_i(3);
    P4_4 = (Hij4(1,4) * u4_IMM(1,k-1) * (P4_1IMM + (x4_1IMM - x4_4) * ((x4_1IMM - x4_4)')) + ...
        Hij4(2,4) * u4_IMM(2,k-1) * (P4_2IMM + (x4_2IMM - x4_4) * ((x4_2IMM - x4_4)')) + ...
        Hij4(3,4) * u4_IMM(3,k-1) * ([P4_3IMM(1:6,1:6), zeros(6,3);zeros(3,9)] + ([x4_3IMM(1:6,1);zeros(3,1)] - x4_4) * (([x4_3IMM(1:6,1);zeros(3,1)] - x4_4)')) + ...
        Hij4(4,4) * u4_IMM(4,k-1) * (P4_4IMM + (x4_4IMM - x4_4) * ((x4_4IMM - x4_4)')) + ...
        Hij4(5,4) * u4_IMM(5,k-1) * (P4_5IMM + (x4_5IMM - x4_4) * ((x4_5IMM - x4_4)')) ) / c4_i(4);
    P4_5 = (Hij4(1,5) * u4_IMM(1,k-1) * (P4_1IMM + (x4_1IMM - x4_5) * ((x4_1IMM - x4_5)')) + ...
        Hij4(2,5) * u4_IMM(2,k-1) * (P4_2IMM + (x4_2IMM - x4_5) * ((x4_2IMM - x4_5)')) + ...
        Hij4(3,5) * u4_IMM(3,k-1) * ([P4_3IMM(1:6,1:6) ,zeros(6,3);zeros(3,9)] + ([x4_3IMM(1:6,1);zeros(3,1)] - x4_5) * (([x4_3IMM(1:6,1);zeros(3,1)] - x4_5)')) + ...
        Hij4(4,5) * u4_IMM(4,k-1) * (P4_4IMM + (x4_4IMM - x4_5) * ((x4_4IMM - x4_5)')) + ...
        Hij4(5,5) * u4_IMM(5,k-1) * (P4_5IMM + (x4_5IMM - x4_5) * ((x4_5IMM - x4_5)')) ) / c4_i(5);
    %% Converted measurement Kalman filter:
    % Model 1:
    [x1_1IMM, P1_1IMM, r1_1IMM, S1_1IMM] = CMKF(x1_1, P1_1, @f1_1, @h1_1, @df1_1, @dh1_1, R1_1, measure(1,k), measure(2,k), measure(3,k), sigmaR, sigmaTheta, sigmaPhi);
    [x1_2IMM, P1_2IMM, r1_2IMM, S1_2IMM] = CMKF(x1_2, P1_2, @f1_2, @h1_2, @df1_2, @dh1_2, R1_2, measure(1,k), measure(2,k), measure(3,k), sigmaR, sigmaTheta, sigmaPhi);
    [x1_3IMM, P1_3IMM, r1_3IMM, S1_3IMM] = CMKF(x1_3, P1_3, @f1_3, @h1_3, @df1_3, @dh1_3, R1_3, measure(1,k), measure(2,k), measure(3,k), sigmaR, sigmaTheta, sigmaPhi);
    % Model 2:
    [x2_1IMM, P2_1IMM, r2_1IMM, S2_1IMM] = CMKF(x2_1, P2_1, @f2_1, @h2_1, @df2_1, @dh2_1, R2_1, measure(1,k), measure(2,k), measure(3,k), sigmaR, sigmaTheta, sigmaPhi);
    [x2_2IMM, P2_2IMM, r2_2IMM, S2_2IMM] = CMKF(x2_2, P2_2, @f2_2, @h2_2, @df2_2, @dh2_2, R2_2, measure(1,k), measure(2,k), measure(3,k), sigmaR, sigmaTheta, sigmaPhi);
    [x2_3IMM, P2_3IMM, r2_3IMM, S2_3IMM] = CMKF(x2_3, P2_3, @f2_3, @h2_3, @df2_3, @dh2_3, R2_3, measure(1,k), measure(2,k), measure(3,k), sigmaR, sigmaTheta, sigmaPhi);
    % Model 3:
    [x3_1IMM, P3_1IMM, r3_1IMM, S3_1IMM] = CMKF(x3_1, P3_1, @f3_1, @h3_1, @df3_1, @dh3_1, R3_1, measure(1,k), measure(2,k), measure(3,k), sigmaR, sigmaTheta, sigmaPhi);
    [x3_2IMM, P3_2IMM, r3_2IMM, S3_2IMM] = CMKF(x3_2, P3_2, @f3_2, @h3_2, @df3_2, @dh3_2, R3_2, measure(1,k), measure(2,k), measure(3,k), sigmaR, sigmaTheta, sigmaPhi);
    % Model 4:
    [x4_1IMM, P4_1IMM, r4_1IMM, S4_1IMM] = CMKF(x4_1, P4_1, @f4_1, @h4_1, @df4_1, @dh4_1, R4_1, measure(1,k), measure(2,k), measure(3,k), sigmaR, sigmaTheta, sigmaPhi);
    [x4_2IMM, P4_2IMM, r4_2IMM, S4_2IMM] = CMKF(x4_2, P4_2, @f4_2, @h4_2, @df4_2, @dh4_2, R4_2, measure(1,k), measure(2,k), measure(3,k), sigmaR, sigmaTheta, sigmaPhi);
    [x4_3IMM, P4_3IMM, r4_3IMM, S4_3IMM] = CMKF(x4_3, P4_3, @f4_3, @h4_3, @df4_3, @dh4_3, R4_3, measure(1,k), measure(2,k), measure(3,k), sigmaR, sigmaTheta, sigmaPhi);
    [x4_4IMM, P4_4IMM, r4_4IMM, S4_4IMM] = CMKF(x4_4, P4_4, @f4_4, @h4_4, @df4_4, @dh4_4, R4_4, measure(1,k), measure(2,k), measure(3,k), sigmaR, sigmaTheta, sigmaPhi);
    [x4_5IMM, P4_5IMM, r4_5IMM, S4_5IMM] = CMKF(x4_5, P4_5, @f4_5, @h4_5, @df4_5, @dh4_5, R4_5, measure(1,k), measure(2,k), measure(3,k), sigmaR, sigmaTheta, sigmaPhi);
    % Update the probability of Model 1.1, 1.2 and 1.3:
    u1_IMM(:,k) = Model_P_up(r1_1IMM, r1_2IMM, r1_3IMM, S1_1IMM, S1_2IMM, S1_3IMM, c1_i);
    u2_IMM(:,k) = Model_P_up(r2_1IMM, r2_2IMM, r2_3IMM, S2_1IMM, S2_2IMM, S2_3IMM, c2_i);
    u3_IMM(:,k) = Model_P_up2(r3_1IMM, r3_2IMM, S3_1IMM, S3_2IMM, c3_i);
    u4_IMM(:,k) = Model_P_up4(r4_1IMM, r4_2IMM, r4_3IMM, r4_4IMM, r4_5IMM, S4_1IMM, S4_2IMM, S4_3IMM, S4_4IMM, S4_5IMM, c4_i);
    % The synthesis process of Model 1:
    [x1_pro_IMM(:,k), P1] = Model_mix(x1_1IMM, x1_2IMM(1:6,1), P1_1IMM, P1_2IMM(1:6, 1:6),u1_IMM, x1_3IMM(1:6,1), P1_3IMM(1:6,1:6));
    [x2_pro_IMM(:,k), P2] = Model_mix(x2_1IMM, x2_2IMM, P2_1IMM, P2_2IMM, u2_IMM, x2_3IMM, P2_3IMM);
    [x3_pro_IMM(:,k) ,P3] = Model_mix(x3_1IMM, x3_2IMM(1:6,1), P3_1IMM, P3_2IMM(1:6,1:6), u3_IMM);
    [x4_pro_IMM(:,k), P4] = Model_mix4(x4_1IMM(1:6,1), x4_2IMM(1:6,1), x4_3IMM(1:6,1), x4_4IMM(1:6,1), x4_5IMM(1:6,1), P4_1IMM(1:6,1:6), P4_2IMM(1:6,1:6),...
        P4_3IMM(1:6,1:6), P4_4IMM(1:6,1:6), P4_5IMM(1:6,1:6), u4_IMM);
end
%% Plotting the data
figure;
hold on
grid on
plot3(X_real(1,:),X_real(2,:), X_real(3,:) ,'color','r', 'LineWidth', 3);
plot3(x1_pro_IMM(1,:),x1_pro_IMM(2,:),x1_pro_IMM(3,:),'color','m','LineWidth',2);
plot3(x2_pro_IMM(1,:),x2_pro_IMM(2,:),x2_pro_IMM(3,:),'color','b','LineWidth',2);
plot3(x3_pro_IMM(1,:),x3_pro_IMM(2,:),x3_pro_IMM(3,:),'color','k','LineWidth',2);
plot3(x4_pro_IMM(1,:), x4_pro_IMM(2,:), x4_pro_IMM(3,:),'color','g','LineWidth',2);
plot3(X_real(1,1),X_real(2,1),X_real(3,1),'*k');
legend('Real','Model 1', 'Model 2', 'Model 3', 'Model 4');
hold off
err_m1 = x1_pro_IMM(1:3,:) - X_real(1:3,:);
err_m2 = x2_pro_IMM(1:3,:) - X_real(1:3,:);
err_m3 = x3_pro_IMM(1:3,:) - X_real(1:3,:);
err_m4 = x4_pro_IMM(1:3,:) - X_real(1:3,:);
figure;
hold on
subplot(3,1,1), hold on, grid on, plot(t, err_m1(1,:),'color','m','LineWidth',2), plot(t, err_m2(1,:),'color','b','LineWidth',2),
plot(t, err_m3(1,:),'color','k','LineWidth',2),plot(t, err_m4(1,:),'color','g','LineWidth',2),legend('Model 1','Model 2','Model 3','Model 4'),hold off;
subplot(3,1,2), hold on, grid on, plot(t, err_m1(2,:),'color','m','LineWidth',2), plot(t, err_m2(2,:),'color','b','LineWidth',2),
plot(t, err_m3(2,:),'color','k','LineWidth',2),plot(t, err_m4(2,:),'color','g','LineWidth',2),legend('Model 1','Model 2','Model 3','Model 4'),hold off;
subplot(3,1,3), hold on, grid on, plot(t, err_m1(3,:),'color','m','LineWidth',2), plot(t, err_m2(3,:),'color','b','LineWidth',2),
plot(t, err_m3(3,:),'color','k','LineWidth',2),plot(t, err_m4(3,:),'color','g','LineWidth',2),legend('Model 1','Model 2','Model 3','Model 4'),hold off;
hold off

