function out = CRCAttach(bits, type)
% CRCAttach - CRC校验位附加 (3GPP 36.212 5.1.1)
% 输入:
%   bits - 列向量 {0,1}
%   type - '24A'(传输块级) / '24B'(码块级) / '16' / '8'
% 输出:
%   out  - 附加CRC后的列向量 [bits; 校验位]
%
% 生成多项式 (文档2.2.25 / 36.212 5.1.1):
%   gCRC24A(D) = D^24+D^23+D^18+D^17+D^14+D^11+D^10+D^7+D^6+D^5+D^4+D^3+D+1
%   gCRC24B(D) = D^24+D^23+D^6+D^5+D+1
%   gCRC16(D)  = D^16+D^12+D^5+1
%   gCRC8(D)   = D^8+D^7+D^4+D^3+D+1
% 校验位 p0..p(L-1) 满足码字多项式可被 g(D) 整除, p0 为最高位先发送

switch type
    case '24A', taps = [24 23 18 17 14 11 10 7 6 5 4 3 1 0]; L = 24;
    case '24B', taps = [24 23 6 5 1 0];                        L = 24;
    case '16',  taps = [16 12 5 0];                            L = 16;
    case '8',   taps = [8 7 4 3 1 0];                          L = 8;
    otherwise,  error('CRCAttach: 未知CRC类型 %s', type);
end

bits = bits(:);
% LFSR: 消息后补 L 个0(左移L位)做模2除法, 余数即校验位
% (注: 用行向量迭代, for对列向量只迭代1次会导致b为整个向量)
g = 2^L;
for t = taps(2:end)
    g = g + 2^t;                       % 生成多项式数值
end
R = 0;
for b = [bits; zeros(L, 1)].'          % 行向量: 逐个bit迭代
    R = R * 2 + b;
    if R >= 2^L
        R = bitxor(R, g);              % 首位为1则异或
    end
end
% 余数按 D^(L-1)..D^0 取位 -> p0 在前
p = zeros(L, 1);
for dd = (L-1):-1:0
    p(L-dd) = mod(floor(R / 2^dd), 2);
end
out = [bits; p];
end
