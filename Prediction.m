function u_k= Prediction(x_k,E,H,N,p)

U_k = zeros(N*p,1); % NP x 1
options = optimset('MaxIter', 200);
% 利用二次规划求解系统控制（输入）
U_k = quadprog(H,(x_k)'*E,[],[],[],[],[],[],[],options);
% 根据模型预测控制的策略，仅选取所得输入的第一项， 参考（5.3.18）


u_k = U_k(1:p,1); % 取第一个结果

end