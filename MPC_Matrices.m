function [E, H] = MPC_Matrices(A, B, K, Q, R, F, N)
    n = size(A, 1);   % A 是 n x n 矩阵, n = 4
    p = size(B, 2);   % B 是 n x p 矩阵, p = 1
    ny = size(K, 1);  % 输出维度，K是 ny x n 矩阵, ny = 1
    
    % 打印维度信息以便调试
    fprintf('维度检查:\n');
    fprintf('A: %dx%d\n', size(A));
    fprintf('B: %dx%d\n', size(B));
    fprintf('K: %dx%d\n', size(K));
    fprintf('Q: %dx%d\n', size(Q));
    fprintf('R: %dx%d\n', size(R));
    fprintf('F: %dx%d\n', size(F));
    fprintf('n=%d, p=%d, ny=%d, N=%d\n', n, p, ny, N);
    
    %%%%%%%%%%%%
    % 构建 M 和 C 矩阵
    M = [eye(n); zeros(N*n, n)]; % M 矩阵是 (N+1)n x n = 8x4
    
    C = zeros((N+1)*n, N*p); % C 矩阵是 (N+1)n x NP = 8x1
    
    % 定义M 和 C 
    tmp = eye(n);  % n x n 单位矩阵
    
    % 更新M和C
    for i = 1:N
        rows = i*n + (1:n); 
        C(rows, :) = [tmp*B, C(rows-n, 1:end-p)];
        tmp = A * tmp;
        M(rows, :) = tmp;
    end 
    
    fprintf('M: %dx%d\n', size(M));
    fprintf('C: %dx%d\n', size(C));
    
    % 构建K_bar矩阵
    K_bar = kron(eye(N+1), K); % K_bar 应该是 (N+1)*ny x (N+1)*n = 2x8
    fprintf('K_bar: %dx%d\n', size(K_bar));
    
    % 定义Q_bar和R_bar
    % 注意：我们需要确保Q_bar的维度与K_bar匹配
    % 由于K_bar是2x8，Q_bar应该是2x2（基于输出维度）
    Q_y = 1; % 输出权重
    F_y = 2; % 终端输出权重
    
    Q_bar = Q_y * eye((N+1)*ny); % Q_bar: (N+1)*ny x (N+1)*ny = 2x2
    R_bar = kron(eye(N), R);   % R_bar: NP x NP = 1x1
    
    fprintf('Q_bar: %dx%d\n', size(Q_bar));
    fprintf('R_bar: %dx%d\n', size(R_bar));
    
    % 逐步计算，确保每一步维度正确
    fprintf('\n逐步计算:\n');
    
    % 第一步: K_bar' * Q_bar
    temp1 = K_bar' * Q_bar;
    fprintf('K_bar'' * Q_bar: %dx%d * %dx%d = %dx%d\n', ...
            size(K_bar',1), size(K_bar',2), size(Q_bar,1), size(Q_bar,2), ...
            size(temp1,1), size(temp1,2));
    
    % 第二步: (K_bar' * Q_bar) * K_bar
    temp2 = temp1 * K_bar;
    fprintf('(K_bar'' * Q_bar) * K_bar: %dx%d * %dx%d = %dx%d\n', ...
            size(temp1,1), size(temp1,2), size(K_bar,1), size(K_bar,2), ...
            size(temp2,1), size(temp2,2));
    
    % 第三步: M' * (K_bar' * Q_bar * K_bar)
    temp3 = M' * temp2;
    fprintf('M'' * (K_bar'' * Q_bar * K_bar): %dx%d * %dx%d = %dx%d\n', ...
            size(M',1), size(M',2), size(temp2,1), size(temp2,2), ...
            size(temp3,1), size(temp3,2));
    
    % 第四步: [M' * (K_bar' * Q_bar * K_bar)] * C
    E = temp3 * C;
    fprintf('E = [M'' * (K_bar'' * Q_bar * K_bar)] * C: %dx%d * %dx%d = %dx%d\n', ...
            size(temp3,1), size(temp3,2), size(C,1), size(C,2), ...
            size(E,1), size(E,2));
    
    % 计算 H
    temp4 = C' * temp2;
    fprintf('C'' * (K_bar'' * Q_bar * K_bar): %dx%d * %dx%d = %dx%d\n', ...
            size(C',1), size(C',2), size(temp2,1), size(temp2,2), ...
            size(temp4,1), size(temp4,2));
    
    temp5 = temp4 * C;
    fprintf('[C'' * (K_bar'' * Q_bar * K_bar)] * C: %dx%d * %dx%d = %dx%d\n', ...
            size(temp4,1), size(temp4,2), size(C,1), size(C,2), ...
            size(temp5,1), size(temp5,2));
    
    H = temp5 + R_bar;
    fprintf('H = [C'' * (K_bar'' * Q_bar * K_bar) * C] + R_bar: %dx%d + %dx%d = %dx%d\n', ...
            size(temp5,1), size(temp5,2), size(R_bar,1), size(R_bar,2), ...
            size(H,1), size(H,2));
end