function out = DeRateMatch(bits, K, C, G, Qm, NL, rvidx)
% DeRateMatch - 解速率匹配 (单个码块, 3GPP 36.212 5.1.4.1 逆过程)
% 输入:
%   bits  - 接收bit列向量 (软信息或硬判决)
%   K     - 码块长度
%   C     - 码块总数
%   G     - 总可用bit数
%   Qm    - 调制阶数 (2/4/6)
%   NL    - 层数 (1或2)
%   rvidx - 冗余版本 (0~3)
% 输出:
%   out   - 解速率匹配后bit (3列矩阵 [d0 d1 d2], 每列 D=K+4)
%
% 算法: RateMatch的逆过程
%   1. 根据rv_idx确定起始位置k0
%   2. 将接收bit填回循环缓冲区的对应位置
%   3. 反子块交织

D = K + 4;
Ccol = 32;
R = ceil(D / Ccol);
Kpi = R * Ccol;
ND = Kpi - D;
P = [0 16 8 24 4 20 12 28 2 18 10 26 6 22 14 30 ...
    1 17 9 25 5 21 13 29 3 19 11 27 7 23 15 31];

% --- 参数计算 (与RateMatch一致) ---
Ncb = 3 * Kpi;
E = NL * Qm * floor(G / (NL * Qm * C));
k0 = R * (2 * ceil(Ncb / (8 * R)) * rvidx + 2);

% --- 步骤1: 将接收bit填回循环缓冲区 ---
w = zeros(1, Ncb);
count = 0;
j = 0;
while count < E
    pos = mod(k0 + j, Ncb) + 1;
    if w(pos) == 0 && count < numel(bits)  % 未填充位置
        w(pos) = bits(count + 1);
        count = count + 1;
    end
    j = j + 1;
end

% --- 步骤2: 从循环缓冲区分离3路 ---
v1 = w(1:Kpi);
v2 = w(Kpi+1:2:2*Kpi);
v3 = w(Kpi+2:2:2*Kpi);

% --- 步骤3: 反子块交织 ---
% 与RateMatch的子块交织相反: 按列写入, 按行读出
y1 = zeros(1, Kpi);
y2 = zeros(1, Kpi);
y3 = zeros(1, Kpi);

for k = 0:Kpi-1
    j = floor(k / R);
    r = mod(k, R);
    y1(r * Ccol + P(j+1) + 1) = v1(k+1);
    y2(r * Ccol + P(j+1) + 1) = v2(k+1);
    y3(r * Ccol + P(j+1) + 1) = v3(k+1);
end

% 流2反循环移位
y3 = circshift(y3, [0 1]);

% 去除NULL填充, 提取有效数据
d0 = y1(ND+1:end);
d1 = y2(ND+1:end);
d2 = y3(ND+1:end);

out = [d0(:), d1(:), d2(:)];

end
