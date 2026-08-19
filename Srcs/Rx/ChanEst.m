function H = ChanEst(rxGrid, rs0, rs1, prm)
% ChanEst - 信道估计, 基于导频的LS估计 + 插值
% 输入:
%   rxGrid   - 接收资源网格 (NSubc x NSym)
%   rs0, rs1 - 本地生成的参考信号 (NSubc x 1)
%   prm      - 参数结构体
% 输出:
%   H - NSubc x NSym 信道频率响应估计
%
% 算法:
%   1. LS估计: 在导频位置 H_rs = rxGrid(:, pilotPos) ./ rs_tx
%   2. 插值: 频域线性插值 + 时域线性插值, 扩展到所有符号

NSubc = prm.NSubc;                  % 1200
NSym  = prm.NSym;                   % 14

% 导频符号位置 (MATLAB 1-indexed, 对应协议的符号3和10)
pilotIdx1 = prm.PilotSymbols(1) + 1;   % 符号3 -> 索引4
pilotIdx2 = prm.PilotSymbols(2) + 1;   % 符号10 -> 索引11

%% 1) LS估计 (在导频位置)
H = zeros(NSubc, NSym);
H(:, pilotIdx1) = rxGrid(:, pilotIdx1) ./ rs0;
H(:, pilotIdx2) = rxGrid(:, pilotIdx2) ./ rs1;

%% 2) 时域线性插值 (从2个导频符号扩展到14个符号)
% 导频位置: 4和11 (1-indexed)
% 对每个子载波, 在两个导频之间做线性插值
for k = 1:NSubc
    h1 = H(k, pilotIdx1);   % 导频1处的信道响应
    h2 = H(k, pilotIdx2);   % 导频2处的信道响应
    
    % 符号1~3: 外推 (使用导频1的值)
    H(k, 1:pilotIdx1-1) = h1;
    
    % 符号4~10: 线性插值
    for l = pilotIdx1+1 : pilotIdx2-1
        alpha = (l - pilotIdx1) / (pilotIdx2 - pilotIdx1);
        H(k, l) = (1 - alpha) * h1 + alpha * h2;
    end
    
    % 符号11~14: 外推 (使用导频2的值)
    H(k, pilotIdx2+1:NSym) = h2;
end

end
