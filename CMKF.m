function [ x_new, P, e, S ] = CMKF( x_pre, P_pre, f, h, df, dh, Q, r, theta, phi, sigmaR,sigmaTheta, sigmaPhi, dr, sigmaDr, rho )
% Realize the alogrithm of converted measurement Kalman filter
if nargin == 13
    % Obtain the covariance matrix R of measurement noise and bias of x,
    % that is mu
    sigma2R = sigmaR^2;
    sigma2Theta = sigmaTheta^2;
    sigma2Phi = sigmaPhi^2;
    l1 = exp(-sigma2Theta/2);           % lambda_theta
    l2 = exp(-2*sigma2Theta);           % lambda_theta'
    l3 = exp(-sigma2Phi/2);                 % lambda_phi
    l4 = exp(-2*sigma2Phi);                 % lambda_phi'
    mu_x = r*cos(theta)*cos(phi)*(1 - l1 * l3);
    mu_y = r*sin(theta)*cos(phi)*(1 - l1 * l3);
    mu_z = r*sin(phi)*(1 - l3);
    R = zeros(3,3);
    R(1,1) = -(l1^2)*(l3^2)*(r^2)*((cos(theta))^2)*((cos(phi))^2) + 1/4*((r^2) + sigma2R)...
        *(1+l2*cos(2*theta))*(1+l4*cos(2*phi));
    R(2,2) = -(l1^2)*(l3^2)*(r^2)*((sin(theta))^2)*((cos(phi))^2) + 1/4*((r^2) + sigma2R)...
        *(1-l2*cos(2*theta))*(1+l4*cos(2*phi));
    R(3,3) = -(l3^2)*(r^2)*((sin(phi))^2) + 1/2*((r^2) + sigma2R)*...
        (1 - l4*cos(2*phi));
    R(1,2) = -(l1^2)*(l3^2)*(r^2)*sin(theta)*cos(theta)*((cos(phi))^2) + ...
        1/4*((r^2) + sigma2R)*l2*sin(2*theta)*(1 + l4*cos(2*phi));
    R(1,3) = -l1*(l3^2)*(r^2)*cos(theta)*sin(phi)*cos(phi) + ...
        1/2*((r^2) + sigma2R)*l1*l4*cos(theta)*sin(2*phi);
    R(2,3) = -l1*(l3^2)*(r^2)*sin(theta)*sin(phi)*cos(phi) + ...
        1/2*((r^2) + sigma2R)*l1*l4*sin(theta)*sin(2*phi);
    R(2,1) = R(1,2);
    R(3,1) = R(1,3);
    R(3,2) = R(2,3);
    % filtering part
    x_m = r*cos(phi)*cos(theta);
    y_m = r*cos(phi)*sin(theta);
    z_m = r*sin(phi);
    y = [x_m;y_m;z_m] - [mu_x; mu_y; mu_z];
    [x_new, P, e, S] = ekf_update(x_pre, y, f, df, h, dh, P_pre, Q, R);
elseif nargin == 16
    % Obtain R and mu, which is the covarience matrix of noise and bias of y, respectively.
    sigma2R = sigmaR^2;
    sigma2Theta = sigmaTheta^2;
    sigma2Phi = sigmaPhi^2;
    sigma2Dr = sigmaDr^2;
    l1 = exp(-sigma2Theta/2);
    l2 = exp(-2*sigma2Theta);
    l3 = exp(-sigma2Phi/2);
    l4 = exp(-2*sigma2Phi);
    % Obtain mu
    mu_x = r*cos(theta)*cos(phi)*(1 - l1 * l3);
    mu_y = r*sin(theta)*cos(phi) * (1 - l1 * l3);
    mu_z = r*sin(phi) * (1 - l3);
    mu_dr = -rho * sigmaR * sigmaDr;
    % Obtain R
    R = zeros(4,4);
    R(1,1) = -(l1^2)*(l3^2)*(r^2)*((cos(theta))^2)*((cos(phi))^2) + 1/4*((r^2) + sigma2R)...
        *(1+l2*cos(2*theta))*(1+l4*cos(2*phi));
    R(2,2) = -(l1^2)*(l3^2)*(r^2)*((sin(theta))^2)*((cos(phi))^2) + 1/4*((r^2) + sigma2R)...
        *(1-l2*cos(2*theta))*(1+l4*cos(2*phi));
    R(3,3) = -(l3^2)*(r^2)*((sin(phi))^2) + 1/2*((r^2) + sigma2R)*...
        (1 - l4*cos(2*phi));
    R(1,2) = -(l1^2)*(l3^2)*(r^2)*sin(theta)*cos(theta)*((cos(phi))^2) + ...
        1/4*((r^2) + sigma2R)*l2*sin(2*theta)*(1 + l4*cos(2*phi));
    R(1,3) = -l1*(l3^2)*(r^2)*cos(theta)*sin(phi)*cos(phi) + ...
        1/2*((r^2) + sigma2R)*l1*l4*cos(theta)*sin(2*phi);
    R(2,3) = -l1*(l3^2)*(r^2)*sin(theta)*sin(phi)*cos(phi) + ...
        1/2*((r^2) + sigma2R)*l1*l4*sin(theta)*sin(2*phi);
    R(2,1) = R(1,2);
    R(3,1) = R(1,3);
    R(3,2) = R(2,3);
    R(1,4) = (sigma2R*dr + r*rho*sqrt(sigma2R)*sqrt(sigma2Dr))*cos(phi)*cos(theta)*exp(-sigma2Theta - sigma2Phi);
    R(2,4) = (sigma2R*dr + r*rho*sqrt(sigma2R)*sqrt(sigma2Dr))*cos(phi)*sin(theta)*exp(-sigma2Theta-sigma2Phi);
    R(3,4) = (sigma2R*dr + r*rho*sqrt(sigma2R)*sqrt(sigma2Dr))*sin(phi)*exp(-sigma2Phi);
    R(4,4) = r^2*sigma2Dr + sigma2R*(dr^2) + 3*(1+rho^2)*sigma2R*sigma2Dr + 2*r*r_*rho*sqrt(sigma2R)*sqrt(sigma2Dr);
    R(4,1) = R(1,4);
    R(4,2) = R(2,4);
    R(4,3) = R(3,4);
    % filtering process
    x_m = r*cos(phi)*cos(theta);
    y_m = r*cos(phi)*sin(theta);
    z_m = r*sin(phi);
    Y = [x_m;y_m;z_m];
    y = [x_m;y_m;z_m] - [mu_x;mu_y;mu_z];
    [X_est, P_new, e, S] = ekf_update(x_pre, y, f, df, h, dh, P_pre, Q, R(1:3,1:3));
    % update with pseudo measurement
    Lk = -R(4,1:3)/R(1:3,1:3);
    Hk = [Lk(1)+X_est(2), X_est(1), Lk(2)+X_est(4), X_est(3), Lk(3)+X_est(6), X_est(5) ];
    Ak = P_new(1,1)*P_new(4,4) + P_new(2,2)*P_new(5,5) +...
        P_new(3,3)*P_new(6,6) + 2*P_new(1,2)*P_new(4,5) +...
        2*P_new(1,5)*P_new(2,4) + 2*P_new(1,3)*P_new(4,6) + ...
        2*P_new(1,6)*P_new(3,4) + 2*P_new(2,3)*P_new(5,6) + ...
        2*P_new(2,6)*P_new(3,5) + P_new(1,4)^2 + ...
        P_new(2,5)^2 + P_new(3,6)^2;
    Kk = P_new*(Hk')/(Hk*P_new*(Hk') + R(4,4) + Ak);
    epsilon = Lk * Y + r*dr;
    hh = Lk(1)*X_est(1) + Lk(2)*X_est(3) + Lk(3)*X_est(5) + X_est(1)*X_est(2)...
        + X_est(3)*X_est(4) + X_est(5)*X_est(6);
    delta2 = 2*P_new(1,4) + 2*P_new(2,5) + 2*P_new(3,6);
    x_new = X_est + Kk*(epsilon - mu_dr - hh - 0.5*delta2);
    P = (eye(6,6) - Kk*Hk)*P_new;
else
    disp('Some parameter is not specific.');
    return;
end
end

