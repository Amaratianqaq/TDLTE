function out = DescrambleSoft(llr, prm)
% DescrambleSoft - 解扰 (LLR域: 异或等价于乘±1)
% 输入:
%   llr - 比特对数似然比列向量 (软信息)
%   prm - 参数结构体 (含 CellID, NSlot, RNTI)
% 输出:
%   out - 解扰后LLR列向量
% 原理: 发送端 out = (bits + c) mod 2 (c为Gold序列, {0,1})
%       LLR域: LLR' = LLR * (1 - 2c)   (c=0保持, c=1取反)

ns = prm.NSlot;
q  = 0;
c_init = prm.RNTI * 2^14 + q * 2^13 + floor(ns / 2) * 2^9 + prm.CellID;
c = GoldSeq(numel(llr), c_init);
out = llr(:) .* (1 - 2 * c(:));
end
