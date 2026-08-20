function [decoded, crcOK] = TurboDecode(coded, K, maxIter)
% TurboDecode - Turbo译码 (Log-MAP算法, 3GPP 36.212 5.1.3.2)
% 输入:
%   coded   - 接收端3路LLR (3列矩阵 [d0 d1 d2], 每列 D=K+4,
%            由DeRateMatch输出: 发送位=±1, 打孔位=0)
%   K       - 码块长度
%   maxIter - 最大迭代次数 (建议6~8)
% 输出:
%   decoded - 译码后bit列向量
%   crcOK   - 码块级CRC校验标志 (logical)
%
% 算法 (PPT 任务21):
%   - Log-MAP算法 (BCJR算法的对数域实现)
%   - 两个SISO译码器迭代交换外信息
%   - 交织器/去交织器与编码端一致 (TurboIntlvrIdx)
%   - 迭代maxIter次后硬判决输出

% 提取3路接收LLR (D = K+4 行, 含尾比特)
D = K + 4;
r0 = coded(:, 1);   % 系统比特LLR
r1 = coded(:, 2);   % 校验流1 LLR
r2 = coded(:, 3);   % 校验流2 LLR

% 交织器索引
piIdx = TurboIntlvrIdx(K);

% 初始化先验信息
Le21 = zeros(K, 1);  % SISO2输出的外信息 -> 解交织 -> SISO1的先验

%% 迭代译码
for iter = 1:maxIter
    % SISO1: 输入系统比特r0, 校验比特r1, 先验信息Le21
    % 尾比特(后4行)参与trellis递推, 提供尾部约束(终止状态0)
    Le1 = sisoLogMap(r0, r1, Le21, K, D);

    % 交织: SISO1的外信息交织后作为SISO2的先验
    Le12 = Le1(piIdx + 1);

    % SISO2: 输入交织后的系统比特, 校验比特r2, 先验信息Le12
    % (尾比特不参与交织, 原样透传)
    r0_pi = [r0(piIdx + 1); r0(K+1:D)];
    Le2 = sisoLogMap(r0_pi, r2, Le12, K, D);

    % 解交织: SISO2的外信息解交织后作为SISO1的先验
    Le21(piIdx + 1) = Le2;
end

%% 最终判决
% LLR = 系统比特 + 先验信息 + 外信息, LLR>0 判为1
L = r0(1:K) + Le21;
decoded = double(L > 0);

%% CRC校验 (如果有CRC24B)
if K > 40
    % 尝试校验CRC24B
    crcOK = checkCRC24B(decoded);
else
    crcOK = true;
end

end

%% ===== 局部函数: SISO Log-MAP译码器 =====
function Le = sisoLogMap(sys, par, La, K, D)
% Log-MAP算法实现8状态RSC译码器
% 输入:
%   sys - 系统比特LLR (D x 1, 后4行为尾比特)
%   par - 校验比特LLR (D x 1, 后4行为尾比特校验)
%   La  - 先验信息 (K x 1, 仅前K步有先验)
%   K   - 码块长度
%   D   - trellis步数 (D = K+4, 含尾比特)
% 输出:
%   Le  - 外信息 (K x 1)

Nst = 8;  % 8个状态
Le = zeros(K, 1);

% 状态转移表 (8状态RSC: g0=1+D^2+D^3, g1=1+D+D^3)
% next_state(s, input) -> 下一状态
% output(s, input) -> 输出校验bit
[nextState, outBit] = buildTrellis();

% 前向递推度量
alpha = -inf(D+1, Nst);
alpha(1, 1) = 0;  % 初始状态为0

% 后向递推度量
beta = -inf(D+1, Nst);
beta(D+1, 1) = 0;  % 终止状态为0 (尾比特使编码器归零)

% 分支度量
gamma = -inf(D, Nst, 2);

% 计算分支度量
% 注意: inp=0分支不含sys/La项 (La=±inf时 La*0 会产生NaN污染)
% 尾比特步(k>K): 先验La=0
for k = 1:D
    if k <= K
        lak = La(k);
    else
        lak = 0;                    % 尾比特无先验信息
    end
    for s = 0:Nst-1
        gamma(k, s+1, 1) = 0.5 * par(k) * outBit(s+1, 1);   % inp=0
        gamma(k, s+1, 2) = 0.5 * (sys(k) + par(k)*outBit(s+1, 2) + lak);  % inp=1
    end
end

% 前向递推
for k = 1:D
    for s = 0:Nst-1
        for inp = 0:1
            ps = prevState(s, inp);  % 前一状态
            if ps >= 0
                val = alpha(k, ps+1) + gamma(k, ps+1, inp+1);
                alpha(k+1, s+1) = logExp(alpha(k+1, s+1), val);
            end
        end
    end
end

% 后向递推
for k = D:-1:1
    for s = 0:Nst-1
        for inp = 0:1
            ns = nextState(s+1, inp+1);  % 下一状态
            val = beta(k+1, ns+1) + gamma(k, s+1, inp+1);
            beta(k, s+1) = logExp(beta(k, s+1), val);
        end
    end
end

% 计算外信息 (仅前K步)
for k = 1:K
    % L(0) = max* over s of [alpha(k,s) + gamma(k,s,0) + beta(k+1,next(s,0))]
    % L(1) = max* over s of [alpha(k,s) + gamma(k,s,1) + beta(k+1,next(s,1))]
    L0 = -inf;
    L1 = -inf;
    for s = 0:Nst-1
        ns0 = nextState(s+1, 1);  % input=0
        ns1 = nextState(s+1, 2);  % input=1
        val0 = alpha(k, s+1) + gamma(k, s+1, 1) + beta(k+1, ns0+1);
        val1 = alpha(k, s+1) + gamma(k, s+1, 2) + beta(k+1, ns1+1);
        L0 = logExp(L0, val0);
        L1 = logExp(L1, val1);
    end
    le = L1 - L0;  % 外信息 = LLR - 系统比特 - 先验信息
    if isnan(le)
        le = 0;    % 防御: 无路径信息时置0
    end
    Le(k) = max(min(le, 30), -30);  % 外信息限幅, 防止±inf扩散污染
end

end

%% ===== 辅助函数 =====
function [nextState, outBit] = buildTrellis()
% 构建8状态RSC编码器的网格图 (表驱动)
% 与 MATLAB poly2trellis(4,[15 13],15) 一致, 即与 TurboEncode 的 rscEncode 同构
% g0=1+D^2+D^3 (反馈), g1=1+D+D^3 (前向)
nextState = [0 4; 4 0; 1 5; 5 1; 6 2; 2 6; 7 3; 3 7];
outTbl    = [0 3; 0 3; 1 2; 1 2; 1 2; 1 2; 0 3; 0 3];
outBit = mod(outTbl, 2);        % 校验输出 = 输出符号的LSB
end

function ps = prevState(s, inp)
% 计算前一状态 (用于后向递推, 表驱动, 与buildTrellis一致)
NS = [0 4; 4 0; 1 5; 5 1; 6 2; 2 6; 7 3; 3 7];
ps = -1;
for p = 0:7
    if NS(p+1, inp+1) == s
        ps = p;
        return;
    end
end
end

function result = logExp(a, b)
% log(exp(a) + exp(b)) 的数值稳定实现
% max* 运算: max(a,b) + log(1 + exp(-|a-b|))
% NaN(由 -inf+inf 等产生)按 -inf(路径不存在)处理, 防止污染传播
if isnan(a)
    a = -inf;
end
if isnan(b)
    b = -inf;
end
if a == -inf
    result = b;
elseif b == -inf
    result = a;
else
    result = max(a, b) + log(1 + exp(-abs(a - b)));
end
end

function crcOK = checkCRC24B(bits)
% 校验CRC24B
% gCRC24B(D) = D^24 + D^23 + D^6 + D^5 + D + 1 (36.212 5.1.1)
g = [1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 1 1];
L = 24;
data = bits(1:end-L);

% 计算数据的CRC, 与接收到的CRC位比较
remainder = crcEncode([data; zeros(L, 1)], g);
crcOK = isequal(remainder(:), bits(end-L+1:end));
end

function crc = crcEncode(bits, g)
% CRC编码
L = length(g) - 1;
R = 0;
for b = bits.'
    R = R * 2 + b;
    if R >= 2^L
        R = bitxor(R, poly2num(g));
    end
end
crc = zeros(L, 1);
for dd = (L-1):-1:0
    crc(L-dd) = mod(floor(R / 2^dd), 2);
end
end

function n = poly2num(p)
% 多项式系数转数值
n = 0;
for i = 1:length(p)
    n = n + p(i) * 2^(length(p) - i);
end
end
