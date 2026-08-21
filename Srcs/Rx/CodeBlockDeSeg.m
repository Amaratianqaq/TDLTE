function [bits, crcOK] = CodeBlockDeSeg(cbDecoded, info)
% CodeBlockDeSeg - 码块去分段 + TB-CRC校验 (3GPP 36.212 5.1.2 逆过程)
% 输入:
%   cbDecoded - cell数组, 每个元素是译码后的码块bit (不含CRC24B)
%   info      - CodeBlockSeg返回的info结构体
% 输出:
%   bits  - 恢复的传输块bit列向量 (不含TB-CRC)
%   crcOK - TB级CRC24A校验是否通过 (logical)
%
% 算法:
%   1. 去除每个码块的CRC24B (C>1时)
%   2. 去除填充比特
%   3. 拼接
%   4. 校验CRC24A

C  = info.C;
F  = info.F;
Kp = info.Kp;
Km = info.Km;
Cp = info.Cp;
Cm = info.Cm;

% 块长度序列: K- 块在前, K+ 块在后 (与CodeBlockSeg一致)
Krs = [repmat(Km, Cm, 1); repmat(Kp, Cp, 1)];

% 1. 拼接各码块 (去除CRC24B和填充比特)
totalLen = 0;
for r = 1:C
    Kr = Krs(r);
    if C == 1
        totalLen = totalLen + Kr;  % 无码块CRC
    else
        totalLen = totalLen + Kr - 24;  % 去除CRC24B
    end
end
totalLen = totalLen - F;  % 去除填充比特

bits = zeros(totalLen, 1);
ptr = 1;
for r = 1:C
    seg = cbDecoded{r};

    % 去除码块CRC24B (C>1时)
    if C > 1
        seg = seg(1:end-24);
    end

    % 去除填充比特 (仅第一个码块的前F个)
    if r == 1 && F > 0
        seg = seg(F+1:end);
    end

    % 拼接
    len = numel(seg);
    bits(ptr:ptr+len-1) = seg(:);
    ptr = ptr + len;
end

% 2. 校验TB级CRC24A (36.212 5.1.1: 整码字除G(x), 余数0则无错)
L = 24;
crc = crcEncode([bits; zeros(L, 1)], '24A');
crcOK = all(crc == 0);

% 3. 剥离TB-CRC, 输出原始传输块 (信息位在前A位, 校验位在后L位)
bits = bits(1:end-L);

end

%% ===== 局部函数: CRC编码 =====
function crc = crcEncode(bits, type)
% CRC编码计算
switch type
    case '24A', taps = [24 23 18 17 14 11 10 7 6 5 4 3 1 0]; L = 24;
    case '24B', taps = [24 23 6 5 1 0];                        L = 24;
    otherwise,  error('CRC类型不支持');
end

g = 2^L;
for t = taps(2:end)
    g = g + 2^t;
end

R = 0;
for b = bits.'
    R = R * 2 + b;
    if R >= 2^L
        R = bitxor(R, g);
    end
end

crc = zeros(L, 1);
for dd = (L-1):-1:0
    crc(L-dd) = mod(floor(R / 2^dd), 2);
end
end
