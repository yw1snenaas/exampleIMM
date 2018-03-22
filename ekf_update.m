function [ xNew, P_new, e, S ] = ekf_update( x, y, f, df, h , dh, P_old, Q, R )
%% 本方法实现扩展卡尔曼滤波（Extended Kalman Filter）的单步运行情况。
%% 输入变量：
% x：上一时刻的估计值。
% y：本时刻的测量值。
% f：状态转移方程
% df：状态转移方程所对应的Jacobian矩阵计算方法，为一个function。
% h：测量方程。
% dh：测量方程所对应的Jacobian矩阵计算方法，为一个function。
% P_old：上一时刻估计的状态变量的协方差矩阵。
% Q：状态转移过程中的加性噪声所对应的协方差矩阵
% R：测量过程中的加性噪声所对应的协方差矩阵
%% 输出变量：
% xNew：本时刻的估计值
% P_new：本时刻的状态变量的协方差估计值。
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 预测过程
x_temp = f(x);
F = df(x_temp);
P_temp = F * P_old * (F') + Q;
%% 更新过程：
H = dh(x_temp);
S = H * P_temp * (H') + R;
K = P_temp * (H') / S;
e = y - h(x_temp);
xNew = x_temp + K * ( y - h(x_temp) );
P_new = P_temp - K * S * (K');
end

