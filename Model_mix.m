function [ x_pro, P ] = Model_mix( x1, x2,P1, P2, u, x3, P3  )
% 模型综合函数
% x_pro 状态综合值
% p 综合协方差矩阵
% x1 模型1状态估计值
% x2 模型2状态估计值
% x3 模型3状态估计值
% P1 模型1状态估计协方差矩阵
% P2 模型2状态估计协方差矩阵
% P3 模型3状态估计协方差矩阵
% u 模型转换概率
if nargin == 7
    x_pro = x1*u(1) + x2*u(2) + x3*u(3);
    P = u(1)*(P1 + (x1 - x_pro)*((x1-x_pro)'))+...
            u(2)*(P2 + (x2-x_pro)*((x2-x_pro)')) + ...
            u(3)*(P3+(x3-x_pro)*((x3-x_pro)'));
elseif nargin == 5
    x_pro = x1*u(1) + x2*u(2);
    P = u(1)*(P1 + (x1 - x_pro)*((x1-x_pro)'))+...
            u(2)*(P2 + (x2-x_pro)*((x2-x_pro)'));
else
    disp('Some parameters are not specific.');
    return;
end
end

