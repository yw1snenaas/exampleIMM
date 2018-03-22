function x_est = ekf_filter( x_init, y, f, df, h, dh, Q, R, T )
%% 本方法实现扩展卡尔曼滤波（Extended Kalman Filter）。
%% 输入变量：
% x_init：状态变量的初始估计值。
% y：全过程的测量值。
% f：状态转移方程。
% df：状态转移方程所对应的Jacobian矩阵计算方法，为一个function。
% h：测量方程。
% dh：测量方程所对应的Jacobian矩阵计算方法，为一个function。
% Q：状态转移过程中的加性噪声所对应的协方差矩阵
% R：测量过程中的加性噪声所对应的协方差矩阵
%% 输出变量：
% x_est：状态估计值
%% 数据初始化
N1 = size(Q,1);         % 状态变量维数
% N2 = size(R,1);         % 测量值维数
Time = size(y,2);       % 仿真时间
P = eye(N1);
x_est = zeros(N1, Time);
for t = 1:Time
    if t == 1
        x_est(:,1) = x_init;
    else
        [x_est(:,t), P] = ekf_update(x_est(:,t-1), y(:,t), f, df, h, dh, P, Q, R, T);
    end
end
end

