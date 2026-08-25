function out = DeRateMatch(bits, K, C, G, Qm, NL, rvidx)
% DeRateMatch - 解速率匹配 (单个码块, 3GPP 36.212 5.1.4.1 逆过程)
% 输入:
%   bits  - 接收LLR列向量 (软信息, 长度E, 由DemodulateSoft输出)
%   K     - 码块长度
%   C     - 码块总数
%   G     - 总可用bit数
%   Qm    - 调制阶数 (2/4/6)
%   NL    - 层数 (1或2)
%   rvidx - 冗余版本 (0~3)
% 输出:
%   out   - 解速率匹配后LLR矩阵 (3列 [d0 d1 d2], 每列 D=K+4)
%           发送位=接收LLR原值, 打孔位/NULL位=0 (译码时按擦除处理)
%
% 算法: RateMatch的逆过程
%   1. 根据rv_idx确定起始位置k0
%   2. 重建循环缓冲区的NULL(哑元)位置, 将接收LLR从k0起填入非NULL位置
%      (重复位LLR求和 = 软合并)
%   3. 分离3路, 反子块交织, 去除NULL, 得到3路LLR

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

% --- 步骤1: 重建循环缓冲, 标记NULL位置 ---
% 按发送端RateMatch的同样方式构造y(含NULL)并子块交织, 得到v,
% 其中NULL值=-1, 其余为0; 再组装成循环缓冲w (NULL位置=-1, 其余为NaN)
y = [-1 * ones(3, ND), zeros(3, D)];
v = zeros(3, Kpi);
for k = 0:Kpi-1
    j = floor(k / R); r = mod(k, R);
    v(:, k + 1) = y(:, r * Ccol + P(j + 1) + 1);
end
% 校验流2按协议重排 (36.212 5.1.4.1.1):
%   v2(k) = v(pi(k)), pi(k) = (P(floor(k/R)) + C_column*(mod(k,R)) + 1) mod Kpi
piIdx = mod(P(floor((0:Kpi-1) / R) + 1) + Ccol * mod((0:Kpi-1), R), Kpi) + 1;
v(3, :) = v(3, piIdx);
w = NaN(1, Ncb);                       % NaN=未填充, -1=NULL
w(1:Kpi) = v(1, :);
w(Kpi+1:2:end-1) = v(2, :);
w(Kpi+2:2:end)   = v(3, :);
w(w == 0) = NaN;                        % 数据位(0)转为未填充标记

% --- 步骤2: 将接收LLR从k0起填入非NULL位置 (重复位软合并=求和) ---
filled = false(1, Ncb);
count = 0;
j = 0;
while count < E
    pos = mod(k0 + j, Ncb) + 1;
    if w(pos) ~= -1                         % 非NULL位置 (NULL=-1)
        if filled(pos)
            w(pos) = w(pos) + bits(count + 1);   % 重复: LLR求和(软合并)
        else
            w(pos) = bits(count + 1);
            filled(pos) = true;
        end
        count = count + 1;
    end
    j = j + 1;
end
w(w == -1) = 0;                             % NULL位置(反交织后丢弃, 置0无影响)

% --- 步骤3: 从循环缓冲区分离3路 ---
v1 = w(1:Kpi);
v2 = w(Kpi+1:2:3*Kpi-1);
v3 = w(Kpi+2:2:3*Kpi);

% --- 步骤4: 反子块交织 ---
% 先恢复流2的原始交织输出: v2_raw(pi(k)) = v2_final(k) (协议 5.1.4.1.1 逆映射)
v3_raw = zeros(1, Kpi);
v3_raw(piIdx) = v3;

% 与RateMatch的子块交织相反: 按列写入, 按行读出
y1 = zeros(1, Kpi);
y2 = zeros(1, Kpi);
y3 = zeros(1, Kpi);
for k = 0:Kpi-1
    j = floor(k / R);
    r = mod(k, R);
    y1(r * Ccol + P(j+1) + 1) = v1(k+1);
    y2(r * Ccol + P(j+1) + 1) = v2(k+1);
    y3(r * Ccol + P(j+1) + 1) = v3_raw(k+1);
end

% --- 步骤5: 去除NULL, 输出LLR (打孔位保持0=擦除) ---
d0 = y1(ND+1:end);
d1 = y2(ND+1:end);
d2 = y3(ND+1:end);
p0 = isnan(d0);                             % 打孔位标记 (NaN)
p1 = isnan(d1);
p2 = isnan(d2);
d0(isnan(d0)) = 0;
d1(isnan(d1)) = 0;
d2(isnan(d2)) = 0;
out = [d0(:), d1(:), d2(:)];
out(p0, 1) = 0;                             % 打孔位按擦除(0)处理
out(p1, 2) = 0;
out(p2, 3) = 0;

end
