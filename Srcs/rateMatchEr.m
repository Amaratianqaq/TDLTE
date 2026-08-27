function E = rateMatchEr(G, Qm, NL, C, r)
% rateMatchEr - 第r个码块(1-based)的速率匹配输出bit数 (3GPP 36.212 5.1.4.1.2)
% 输入:
%   G  - 总可用bit数
%   Qm - 调制阶数 (2/4/6)
%   NL - 层数
%   C  - 码块总数
%   r  - 码块序号 (1-based, 1..C)
% 输出:
%   E  - 该码块速率匹配输出bit数
%
% 标准 (36.212 5.1.4.1.2): G' = G/(NL*Qm); gamma = G' mod C
%   前 (C-gamma) 个码块: E = NL*Qm*floor(G'/C)
%   后 gamma 个码块:    E = NL*Qm*ceil(G'/C)
% 说明: 旧实现 E=NL*Qm*floor(G/(NL*Qm*C)) 仅当G整除时正确; DC置空后
%       G=1199*12*Qm 不再整除, 余数必须分配给最后gamma个码块, 否则各块
%       总长 < G 导致码块级联与解速率匹配错位。

Gp = G / (NL * Qm);              % 每层调制符号总数 (应为整数)
gamma = mod(Gp, C);
if r <= C - gamma                 % 前 C-gamma 个码块取 floor
    E = NL * Qm * floor(Gp / C);
else                              % 后 gamma 个码块取 ceil
    E = NL * Qm * ceil(Gp / C);
end
end
