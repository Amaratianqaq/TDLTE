function out = Descramble(bits, prm)
% Descramble - 解扰, 使用与发送端相同的Gold序列 (3GPP 36.211 5.3.1 PUSCH)
% 输入:
%   bits - bit列向量 {0,1}
%   prm  - 参数结构体 (含 CellID, NSlot, RNTI)
% 输出:
%   out  - 解扰后bit列向量 {0,1}
%
% 算法: 与Scramble相同, Gold序列异或
%   c_init = n_RNTI*2^14 + q*2^13 + floor(n_s/2)*2^9 + N_ID^cell
%   out = (bits + c) mod 2

ns = prm.NSlot;                              % 时隙号 (0/1)
q  = 0;                                      % 单码字
c_init = prm.RNTI * 2^14 + q * 2^13 + floor(ns / 2) * 2^9 + prm.CellID;

c = GoldSeq(numel(bits), c_init);
out = mod(bits(:) + c, 2);

end
