clear all;
close all;
clc;
% 读取Octave控制数据库（注：如使用Matlab，可删除或注释掉本行代码）

%%%%%%%%%%%%%%%%%%%%%%%%%%
% 构建系统矩阵A
A0=  [-9.93 -6.47 0 6.5;-5.96 -9.84 0 4.43e-9;0 0 -14.18 0.88;19.82 19.82 76.3 -20.83];
% 构建输入矩阵B
B0= [0;0;298.4;0];
%定义两组采样时间
K = [-1.067e-4 -1.067e-4 0 1.067e-4];
Ts_2 = 2;
%根据公式计算；
A= expm(A0*Ts_2);
B= matrix_inverse_adjoint(A0)*(A-eye(size(A0,1)))*B0;
% 连续系统转离散系统
n= size (A,1);

%% 定义输入矩阵 B, n x p 矩阵
p = size(B,2);

%% 定义Q矩阵，n x n 矩阵
Q = diag([100, 50, 10, 5]);      % 大幅增加状态权重
F = diag([100, 50, 10, 5]);      % 大幅增加终端权重
R = 1;                            % 大幅减小控制权重，允许更积极的控制

%% 定义step数量k
k_steps=100; 

%% 定义矩阵 X_K， n x k 矩 阵
X_K = zeros(n,k_steps);

%% 初始状态变量值， n x 1 向量
X_K(:,1) =[1;1;1;1];

%% 定义输入矩阵 U_K， p x k 矩阵
U_K=zeros(p,k_steps);

%% 定义预测区间K
N=1;

%% Call MPC_Matrices 函数 求得 E,H矩阵 
[E,H]=MPC_Matrices(A,B,K,Q,R,F,N);

%% 修改：在每个采样周期内保持u不变
for k = 1 : k_steps 
    % 只在每个采样周期的开始时刻计算新的控制输入
    if mod(k-1, Ts_2) == 0
        %% 求得U_K(:,k)
        current_u = Prediction(X_K(:,k),E,H,N,p);
    end
    
    %% 在整个采样周期内使用相同的控制输入
    U_K(:,k) = current_u;
    
    %% 计算第k+1步时状态变量的值
    X_K(:,k+1)=(A*X_K(:,k)+B*U_K(:,k));
    Y_K(:,k) = K * X_K(:,k);
end

%% 绘制状态变量、输出和输入的变化
figure;

% 第一个子图：状态变量
subplot(3, 1, 1);
hold on;
colors = ['b-', 'r-', 'g-', 'm-']; % 不同颜色
state_names = {'状态1', '状态2', '状态3', '状态4'};

for i = 1:size(X_K,1)
    plot(X_K(i,1:k_steps), colors(i), 'LineWidth', 1.5, 'DisplayName', state_names{i});
end

xlabel('时间步');
ylabel('状态值');
title('状态变量变化');
legend('show', 'Location', 'best');
grid on;
hold off;

% 第二个子图：输出变量
subplot(3, 1, 2);
hold on;
for i = 1:size(Y_K,1)
    plot(Y_K(i,:), 'c-', 'LineWidth', 1.5, 'DisplayName', sprintf('输出%d', i));
end

xlabel('时间步');
ylabel('输出值');
title('系统输出变化');
legend('show', 'Location', 'best');
grid on;
hold off;

% 第三个子图：控制输入
subplot(3, 1, 3);
hold on;
for i = 1:size(U_K,1)
    plot(U_K(i,:), 'b-', 'LineWidth', 1.5, 'DisplayName', '控制输入');
end

% 添加采样时刻标记
sampling_instants = 1:Ts_2:k_steps;
sampling_u = U_K(1,sampling_instants);
plot(sampling_instants, sampling_u, 'ro', 'MarkerSize', 6, 'DisplayName', '采样时刻');

xlabel('时间步');
ylabel('控制输入值');
title('控制输入变化（红色圆圈表示采样时刻）');
legend('show');
grid on;
hold off;