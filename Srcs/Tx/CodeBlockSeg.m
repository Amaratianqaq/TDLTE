function [cbs, info] = CodeBlockSeg(bits)
% CodeBlockSeg - 码块分割, 每段附加gCRC24B (3GPP 36.212 5.1.2)
% 输入:
%   bits - 加过TB-CRC的bit序列 (B位)
% 输出:
%   cbs  - cell数组, 每个元素是一个码块 (含CRC24B, 前部可能含filler=0)
%   info - 结构体 (C码块数, K各码块长度, Filler数F, Kp/Km/Cp/Cm等)
%
% 流程 (36.212 5.1.2, 与 lteCodeBlockSegment 一致):
%   B<=6144: C=1, 无码块CRC, K圆整到表5.1.3-3最小合法值(填filler)
%   B>6144 : C=ceil(B/6120), B'=B+24C, 每码块附CRC24B
%   K+ = 使 C*K>=B' 的最小表值; K- = 小于K+的最大表值
%   C+ = ceil((B'-C*K-)/(K+-K-)), C- = C-C+ (K-块在前)
%   F = C+*K+ + C-*K- - B' 个 filler, 全部放在码块0开头(编码时按0)

Z = 6144; L = 24;
B = numel(bits);
Ks = turboBlockSizes();

if B <= Z
    C = 1; Bp = B;
else
    C = ceil(B / (Z - L)); Bp = B + C * L;
end

Kp = Ks(find(Ks * C >= Bp, 1));
Km = Ks(find(Ks < Kp, 1, 'last'));
if C * Kp == Bp
    Cp = C; Cm = 0;
else
    Cp = ceil((Bp - C * Km) / (Kp - Km));
    Cm = C - Cp;
end
F = Cp * Kp + Cm * Km - Bp;             % filler 总数(块0)

% 块长度序列: K- 块在前, K+ 块在后
Krs = [repmat(Km, Cm, 1); repmat(Kp, Cp, 1)];

cbs = cell(C, 1);
ptr = 1;
for r = 0:C-1
    Kr = Krs(r+1);
    if C == 1
        d = B;                           % 无码块CRC
    elseif r == 0
        d = Kr - L - F;                  % 块0: 扣filler与CRC
    else
        d = Kr - L;                      % 其余块: 扣CRC
    end
    seg = bits(ptr:ptr+d-1); ptr = ptr + d;
    if C > 1
        seg = CRCAttach(seg, '24B');     % 码块CRC24B (filler按0参与计算)
    end
    if r == 0 && F > 0
        seg = [zeros(F, 1); seg];        % filler(0)放在码块0开头
    end
    cbs{r+1} = seg;
end

info.C = C; info.K = Krs; info.F = F;
info.Kp = Kp; info.Km = Km; info.Cp = Cp; info.Cm = Cm;
end

%% ===== 局部函数: 表5.1.3-3 合法块长 (188个) =====
function Ks = turboBlockSizes()
Ks = [40:8:512, 528:16:1024, 1056:32:2048, 2112:64:4096, 4160:64:6144];
end
