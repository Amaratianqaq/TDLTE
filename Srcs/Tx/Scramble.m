function out = Scramble(bits, prm)
% Scramble - 加扰, 使用31位Gold伪随机序列 (3GPP 36.211 5.3.1 PUSCH)
% 输入:
%   bits - bit列向量 {0,1}
%   prm  - 参数结构体 (含 CellID, NSlot, RNTI)
% 输出:
%   out  - 加扰后bit列向量 {0,1}  (out = bits xor c)
%
% PUSCH 加扰 (36.211 5.3.1):
%   c_init = n_RNTI*2^14 + q*2^13 + floor(n_s/2)*2^9 + N_ID^cell
%   (q=码字序号(0), n_s=时隙号, 每子帧起点重新初始化)
%   b~(i) = (b(i) + c(i)) mod 2

ns = prm.NSlot;                              % 时隙号 (0/1)
q  = 0;                                      % 单码字
c_init = prm.RNTI * 2^14 + q * 2^13 + floor(ns / 2) * 2^9 + prm.CellID;

c = GoldSeq(numel(bits), c_init);
out = mod(bits(:) + c, 2);
end
