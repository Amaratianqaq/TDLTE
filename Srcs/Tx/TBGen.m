function [bits, info] = TBGen(prm)
% TBGen - 生成传输块随机bit数据 (对应文档2.2.1 产生随机信源)
% 输入:
%   prm - 参数结构体 (含 TBsize, Modulation 等, 由 GetLTEParams 返回)
% 输出:
%   bits - 列向量 {0,1}, 长度 = prm.TBsize
%   info - 结构体, 携带长度/调制/G等元信息供后续模块使用
%
% G 按 36.213 (PUSCH): G = N_PRB * N_sc^RB * N_symb_data * NL * Qm
% 文档L91: 子载波1200, 14符号中导频占2个 -> 数据符号12个

bits = randi([0 1], prm.TBsize, 1);

% 总可用编码bit数 (文档L91: 1200子载波 x 12数据符号 x Qm x NL)
info.A = prm.TBsize;                        % 传输块长度
info.B = prm.TBsize;                        % 加TB-CRC前长度(此处未加)
info.G = prm.NDataSubc * (prm.NSym - prm.NPilot) * prm.NL * prm.Qm;   % 1200子载波中DC置空 -> 1199
info.Modulation = prm.Modulation;
info.Qm = prm.Qm;
end
