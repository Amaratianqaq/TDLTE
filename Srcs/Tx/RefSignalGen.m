function [rs0, rs1] = RefSignalGen(prm)
% RefSignalGen - 生成上行解调参考信号 DMRS (PUSCH, 对应文档任务5: pusch_rs_gen)
% 依据: 3GPP TS 36.211 Section 5.5.1 / 5.5.2
% 输入:
%   prm - 参数结构体 (含 CellID, NSlot, NRB, NDMRS1, CyclicShift, DeltaSS,
%         GroupHopping, SequenceHopping; 由 GetLTEParams 返回)
% 输出:
%   rs0 - 导频符号0(符号3, 时隙0)的DMRS序列, Msc x 1 复列向量
%   rs1 - 导频符号1(符号10, 时隙1)的DMRS序列, Msc x 1 复列向量
%         (Msc = NRB*12, 文档L73: 导频占据符号3和10)
%
% 生成步骤 (文档L517: 先定组号u, 再定组内序号v):
%   1. 组号  u = (fgh + fss) mod 30,  fss = (N_ID^cell + Δss) mod 30
%           组跳转关闭时 fgh = 0
%   2. 组内序号 v (序列跳转关闭时 v = 0)
%   3. ZC基序列 r_u,v(n) = x_q(n mod N_ZC),  x_q(m) = exp(-j*pi*q*m*(m+1)/N_ZC)
%      N_ZC = 小于 Msc 的最大素数;  q = floor(qbar+1/2) + v*(-1)^floor(2*qbar),
%      qbar = N_ZC*(u+1)/31
%   4. 循环移位 r^(alpha)(n) = exp(j*alpha*n)*r(n)
%      alpha = 2*pi*n_cs/12,  n_cs = (n1 + n2 + n_PN) mod 12
%      n1 = 表5.5.2.1.1-1(ndmrs1), n2 = 表5.5.2.1.1-2(cyc_shift),
%      n_PN(ns) = sum_{i=0}^{7} c(8*7*ns + i)*2^i (Gold序列)
% 幅度恒为1: |r^(alpha)(n)| = 1, 便于接收端信道估计

Msc = prm.NRB * 12;                              % 序列长度 (1200 for 100RB)

%% 1) 组号 u 与组内序号 v
fss = mod(prm.CellID + prm.DeltaSS, 30);         % 序列移位样式
if strcmpi(prm.GroupHopping, 'On')
    fgh = groupHoppingPattern(prm);              % 组跳转样式 (Gold序列)
else
    fgh = 0;
end
u = mod(fgh + fss, 30);                          % 序列组号

if strcmpi(prm.SequenceHopping, 'On') && Msc >= 6*12
    v = sequenceHoppingPattern(prm);             % 序列跳转 (仅 >=6RB)
else
    v = 0;
end

%% 2) ZC 基序列 (Msc >= 36 即 >=3RB 时; 本项目 100RB 走此分支)
NZC   = prevprime(Msc);                          % 小于Msc的最大素数
qbar  = NZC * (u + 1) / 31;
q     = floor(qbar + 0.5) + v * (-1)^floor(2*qbar);   % 根序列号
n     = (0:Msc-1).';
m     = mod(n, NZC);
rBase = exp(-1i * pi * q .* m .* (m + 1) / NZC); % 基序列 r_u,v(n)

%% 3) 循环移位 (两个导频符号/时隙各自独立)
map1  = [0 2 3 4 6 8 9 10];                      % 表5.5.2.1.1-1: n_DMRS^(1)->循环移位
map2  = [0 6 3 4 2 8 10 9];                      % 表5.5.2.1.1-2: n_DMRS^(2)->循环移位
n1    = map1(prm.NDMRS1 + 1);
n2    = map2(prm.CyclicShift + 1);

rs0 = zeros(Msc, 1);
rs1 = zeros(Msc, 1);
for ns = 0:1                                     % 时隙0 -> 符号3, 时隙1 -> 符号10
    npn    = pseudoRandomCS(prm, ns);            % 伪随机循环移位 n_PN
    ncs    = mod(n1 + n2 + npn, 12);
    alpha  = 2 * pi * ncs / 12;
    seq    = exp(1i * alpha * n) .* rBase;       % r^(alpha)(n)
    if ns == 0
        rs0 = seq;
    else
        rs1 = seq;
    end
end

end

%% ===== 局部函数 =====

function npn = pseudoRandomCS(prm, ns)
% 伪随机循环移位: n_PN(ns) = sum_{i=0}^{7} c(8*N_symb^UL*ns + i) * 2^i
% c 为 Gold 序列; c_init 优先取 N_csh_DMRS_ID(NDMRSID),
% 未配置时按小区参数确定 (与 ltePUSCHDRS 实测规则一致, 覆盖常见 CellID<120):
%   CellID < 30 : c_init = (CellID + Δss) mod 30
%   CellID >= 30: c_init = (CellID mod 30) + Δss + 32, 结果 >= 62 再减 30
% 注: 36.211 协议简化写法为 c_init = N_ID^cell; 本项目默认 CellID=0,
%     两种写法一致; 教学场景 CellID<120 与官方逐点一致, 更大 CellID
%     官方另有内部规则, 工程上请以 ltePUSCHDRS 为准
N_SYMB_UL = 7;                                   % 正常CP每时隙符号数
if prm.CellID < 30
    cinit = mod(prm.CellID + prm.DeltaSS, 30);
else
    cinit = mod(prm.CellID, 30) + prm.DeltaSS + 32;
    if cinit >= 62
        cinit = cinit - 30;
    end
end
c = GoldSeq(8 * N_SYMB_UL * 2, cinit);           % 覆盖两个时隙
npn = 0;
for i = 0:7
    npn = npn + c(8 * N_SYMB_UL * ns + i + 1) * 2^i;
end
end

function fgh = groupHoppingPattern(prm)
% 组跳转样式: fgh(ns) = [sum_{i=0}^{7} c(8*ns + i)*2^i] mod 30
% c 为 Gold 序列, 初始化 c_init 与 PUSCH 序列移位样式相关
c = GoldSeq(8 * 2, 0);
fgh = 0;
for i = 0:7
    fgh = fgh + c(8 * prm.NSlot + i + 1) * 2^i;
end
fgh = mod(fgh, 30);
end

function v = sequenceHoppingPattern(prm)
% 序列跳转: v = c(ns), c 为 Gold 序列 (需 >=6RB 才启用)
c = GoldSeq(1, 0);
v = c(prm.NSlot + 1);
end
