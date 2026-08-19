function out = RateMatch(coded, K, C, G, Qm, NL, rvidx)
% RateMatch - 速率匹配 (单个码块, 3GPP 36.212 5.1.4.1)
% 输入:
%   coded - Turbo编码输出 (3列矩阵 [d0 d1 d2], 各D=K+4)
%   K     - 码块长度
%   C     - 码块总数
%   G     - 总可用bit数 (36.213: G = N_PRB*12*Nsymb_data*NL*Qm)
%   Qm    - 调制阶数 (2/4/6)
%   NL    - 层数 (1或2)
%   rvidx - 冗余版本 (0~3)
% 输出:
%   out   - 速率匹配后bit列向量 (长度 E = NL*Qm*floor(G/(NL*Qm*C)))
%
% 处理 (36.212 5.1.4.1, 与 lteRateMatchTurbo 一致):
%   1. 子块交织: 32列, NULL哑元放在最前, 列置换表P, 逐列读出
%   2. 比特收集: 循环缓冲 w=[v0 | v1,v2交错], 长度 Kw=3*Kpi
%   3. 比特选择: 从 k0=R*(2*ceil(Ncb/(8R))*rv+2) 循环读E个非NULL位
% 注: 本实现按均匀分配 E=NL*Qm*floor(G/(NL*Qm*C)) (项目TBsize配置下
%     G 被 C 整除, 与36.212的ceil/floor分配一致)

D = K + 4;
Ccol = 32;
R = ceil(D / Ccol);
Kpi = R * Ccol;
ND = Kpi - D;
NULL = -1;
P = [0 16 8 24 4 20 12 28 2 18 10 26 6 22 14 30 ...
    1 17 9 25 5 21 13 29 3 19 11 27 7 23 15 31];

% --- 子块交织 (y=[NULL在前, d]; 流2的y整体循环移位1位) ---
d = coded.';                                  % 3 x D (输入为3列矩阵)
y = [NULL * ones(3, ND), d];                  % 3 x Kpi
y(3, :) = circshift(y(3, :), [0 -1]);      % 校验流2 (36.212 5.1.4.1.1)
v = zeros(3, Kpi);
for k = 0:Kpi-1
    j = floor(k / R); r = mod(k, R);
    v(:, k + 1) = y(:, r * Ccol + P(j + 1) + 1);
end

% --- 比特收集: 循环缓冲 ---
w = zeros(1, 3 * Kpi);
w(1:Kpi) = v(1, :);
w(Kpi+1:2:end-1) = v(2, :);
w(Kpi+2:2:end)   = v(3, :);

% --- 比特选择: E, k0, 循环读跳过NULL ---
Ncb = 3 * Kpi;
E = NL * Qm * floor(G / (NL * Qm * C));
k0 = R * (2 * ceil(Ncb / (8 * R)) * rvidx + 2);
out = zeros(E, 1);
count = 0;
j = 0;
while count < E
    x = w(mod(k0 + j, Ncb) + 1);
    if x ~= NULL
        count = count + 1;
        out(count) = x;
    end
    j = j + 1;
end
end
