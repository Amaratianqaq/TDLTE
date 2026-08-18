function seq = GoldSeq(len, cinit)
% GoldSeq - 31位Gold伪随机序列生成 (3GPP 36.211 Section 7.2)
% 输入:
%   len   - 序列长度
%   cinit - 初始化值 (32位)
% 输出:
%   seq   - {0,1} 列向量
%
% 说明:
%   c(n) = [x1(n+NC) + x2(n+NC)] mod 2, NC = 1600
%   x1: x1(0)=1, x1(1..30)=0, x1(n+31)=[x1(n+3)+x1(n)] mod 2
%   x2: x2(i)=cinit的第i位(i=0..30), x2(n+31)=[x2(n+3)+x2(n+2)+x2(n+1)+x2(n)] mod 2
%   用于加扰(Scramble)和DMRS伪随机循环移位(n_PN)

NC = 1600;
x1 = zeros(1, NC + len);
x2 = zeros(1, NC + len);

% 初始化 x1
x1(1) = 1;                                  % x1(0) = 1, 其余 x1(1..30) = 0

% 初始化 x2: c_init = sum(x2(i)*2^i), i = 0..30
for i = 1:31
    x2(i) = bitand(bitshift(cinit, -(i-1)), 1);
end

% 递推生成 (n = 0 .. NC+len-31)
for n = 0:(NC + len - 32)
    x1(n + 32) = bitxor(x1(n + 4), x1(n + 1));           % x1(n+31) = x1(n+3) + x1(n)
    x2(n + 32) = bitxor(bitxor(bitxor(x2(n + 4), x2(n + 3)), x2(n + 2)), x2(n + 1)); % x2(n+31) = x2(n+3)+x2(n+2)+x2(n+1)+x2(n)
end

% 输出 c(n) = [x1(n+1600) + x2(n+1600)] mod 2
seq = mod(x1(NC+1:NC+len) + x2(NC+1:NC+len), 2).';
end
