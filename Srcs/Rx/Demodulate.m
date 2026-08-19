function bits = Demodulate(symbols, mode)
% Demodulate - 解调 (硬判决, 3GPP 36.211 7.1)
% 输入:
%   symbols - 复数符号列向量
%   mode    - 'QPSK' / '16QAM' / '64QAM'
% 输出:
%   bits    - {0,1} 列向量
%
% 算法: 对每个符号找最近的星座点, 输出对应的bit
% 与Modulate.m互逆

switch mode
    case 'QPSK'
        Qm = 2; s = sqrt(2);
        % 星座点: (I+jQ)/sqrt(2), 对应bit [b0 b1]
        % b0: I>0->0, I<0->1;  b1: Q>0->0, Q<0->1
        mI = [1 -1];                        % 0->+1, 1->-1
        mQ = [1 -1];

    case '16QAM'
        Qm = 4; s = sqrt(10);
        mI = [1 3 -1 -3];                   % 00->+1, 01->+3, 10->-1, 11->-3
        mQ = [1 3 -1 -3];

    case '64QAM'
        Qm = 6; s = sqrt(42);
        mI = [3 1 5 7 -3 -1 -5 -7];         % 000->+3, 001->+1, ...
        mQ = [3 1 5 7 -3 -1 -5 -7];

    otherwise
        error('Demodulate: 不支持调制方式 %s', mode);
end

symbols = symbols(:) * s;                   % 反归一化
nsym = numel(symbols);

% 构建完整星座表
[I_grid, Q_grid] = ndgrid(mI, mQ);
constellation = I_grid + 1i * Q_grid;       % Qm个星座点
constellation = constellation(:);

% 构建星座点对应的bit表
bitTable = zeros(numel(constellation), Qm);
for idx = 1:numel(constellation)
    iIdx = ceil(idx / numel(mQ));            % I索引
    qIdx = mod(idx-1, numel(mQ)) + 1;       % Q索引
    iBits = de2bi(iIdx-1, Qm/2, 'left-msb');
    qBits = de2bi(qIdx-1, Qm/2, 'left-msb');
    bitTable(idx, 1:2:Qm) = iBits;
    bitTable(idx, 2:2:Qm) = qBits;
end

% 硬判决: 对每个符号找最近的星座点
bits = zeros(nsym * Qm, 1);
for k = 1:nsym
    [~, idx] = min(abs(symbols(k) - constellation).^2);
    bits((k-1)*Qm+1 : k*Qm) = bitTable(idx, :).';
end

end
