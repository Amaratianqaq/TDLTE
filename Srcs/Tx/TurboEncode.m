function coded = TurboEncode(cb, K)
% TurboEncode - 单个码块的Turbo编码 (速率1/3, 3GPP 36.212 5.1.3.2)
% 输入:
%   cb   - 单个码块bit列向量 {0,1} (长度<=K, 不足部分按filler补0)
%   K    - 码块长度 (含filler, 须为36.212表5.1.3-3合法值)
% 输出:
%   coded - 编码后bit, 3列矩阵 [d0 d1 d2], 每列 D=K+4
%           d0=系统流(含4尾列), d1=校验流1, d2=校验流2
%           (排列与 lteTurboEncode 一致)
%
% 编码结构 (36.212 5.1.3.2.1):
%   - 8状态RSC子编码器: g0(D)=1+D^2+D^3(反馈), g1(D)=1+D+D^3(前向)
%     递推: w_k = c_k xor s2 xor s3;  z_k = w_k xor s1 xor s3
%   - QPP内交织器: c'(i) = c(Pi(i)), Pi 由 TurboIntlvrIdx 查表
%   - 尾比特(Trellis termination): 反馈值x喂回编码器直至归零,
%     两编码器各3对(x,z), 12位按列主序排入3流尾4列:
%       d0尾=[x1,z2,x1',z2'], d1尾=[z1,x3,z1',x3'], d2尾=[x2,z3,x2',z3']

if nargin < 2 || isempty(K)
    K = numel(cb);
end
D = K + 4;

% 补齐到 K (filler 位置按0处理, 文档2.2.4)
c = zeros(K, 1);
c(1:numel(cb)) = cb(:);

% QPP 交织
piIdx = TurboIntlvrIdx(K);
cp = c(piIdx + 1);

% 两个 RSC 编码 (含尾比特)
[z1, x1, z1t] = rscEncode(c);
[z2, x2, z2t] = rscEncode(cp);

% 组装 3 流 (36.212 5.1.3.2.1 尾列排列; 流2数据为编码器2的校验z')
% 每列长度 D = K+4: 前K位数据 + 末4位尾比特
d0 = zeros(D, 1); d1 = zeros(D, 1); d2 = zeros(D, 1);
d0(1:K) = c;
d1(1:K) = z1;
d2(1:K) = z2;
d0(K+1:K+4) = [x1(1); z1t(2); x2(1); z2t(2)];
d1(K+1:K+4) = [z1t(1); x1(3); z2t(1); x2(3)];
d2(K+1:K+4) = [x1(2); z1t(3); x2(2); z2t(3)];

coded = [d0, d1, d2];
end

%% ===== 局部函数: 8状态RSC编码器(含尾比特, 表驱动) =====
function [z, x, zt] = rscEncode(c)
% 表驱动实现, 与 MATLAB poly2trellis(4,[15 13],15) 完全一致
% (该 trellis 即 36.212 图 5.1.3-2 的 8 状态 RSC: g0=1+D^2+D^3, g1=1+D+D^3)
% NS(s+1, inp+1) = 下一状态; OUT(s+1, inp+1) = 输出(十进制2bit), 校验位=LSB
NS  = [0 4; 4 0; 1 5; 5 1; 6 2; 2 6; 7 3; 3 7];
OUT = [0 3; 0 3; 1 2; 1 2; 1 2; 1 2; 0 3; 0 3];
K = numel(c);
s = 0;
z = zeros(K, 1);
for k = 1:K
    o = OUT(s+1, c(k)+1);
    z(k) = mod(o, 2);           % 校验输出 = 输出符号的LSB
    s = NS(s+1, c(k)+1);
end
% 尾比特: 反馈值 x = s1 xor s3 (状态最高位 xor 最低位), 喂回直至归零
% (反馈函数由 nextStates 表推出: f(s) = MSB(next(s,0)) + 0)
x = zeros(3, 1);
zt = zeros(3, 1);
for t = 1:3
    xb = bitxor(bitand(bitshift(s, -2), 1), bitand(s, 1));  % MSB xor LSB
    x(t) = xb;
    o = OUT(s+1, xb+1);
    zt(t) = mod(o, 2);
    s = NS(s+1, xb+1);
end
end
