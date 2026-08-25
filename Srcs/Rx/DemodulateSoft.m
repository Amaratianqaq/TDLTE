function llr = DemodulateSoft(symbols, mode, noiseVar)
% DemodulateSoft - 软判决解调 (max-log MAP, 输出比特LLR)
% 输入:
%   symbols  - 均衡后复数符号列向量 (归一化, 与Modulate输出同尺度)
%   mode     - 'QPSK' / '16QAM' / '64QAM'
%   noiseVar - 每符号噪声方差 (归一化符号域的标量)
% 输出:
%   llr - 比特对数似然比列向量 (>0 倾向bit=1, <0 倾向bit=0)
%
% 算法: max-log近似
%   LLR(b_k) = ( min_{c:b_k=0} |s-c|^2 - min_{c:b_k=1} |s-c|^2 ) / noiseVar
% 与Modulate.m的星座映射一致 (I位在奇位置, Q位在偶位置)

switch mode
    case 'QPSK',  Qm = 2; s = sqrt(2);  mI = [1 -1];            mQ = [1 -1];
    case '16QAM', Qm = 4; s = sqrt(10); mI = [1 3 -1 -3];       mQ = [1 3 -1 -3];
    case '64QAM', Qm = 6; s = sqrt(42); mI = [3 1 5 7 -3 -1 -5 -7]; mQ = mI;
    otherwise,    error('DemodulateSoft: 未知调制 %s', mode);
end

sym = symbols(:) * s;                 % 反归一化 (与Modulate一致, 未归一化尺度)
nsym = numel(sym);

%% 星座表与bit表 (与Demodulate.m完全一致)
[Ig, Qg] = ndgrid(mI, mQ);
constellation = Ig(:) + 1i*Qg(:);     % 未归一化星座点
M = numel(constellation);
bitTable = zeros(M, Qm);
for idx = 1:M
    iIdx = mod(idx-1, numel(mI)) + 1;
    qIdx = ceil(idx / numel(mI));
    iBits = de2bi(iIdx-1, Qm/2, 'left-msb');
    qBits = de2bi(qIdx-1, Qm/2, 'left-msb');
    bitTable(idx, 1:2:Qm) = iBits;
    bitTable(idx, 2:2:Qm) = qBits;
end

%% max-log LLR (距离在未归一化尺度, 噪声方差同步放大 s^2)
dist = abs(sym - constellation(:).').^2;   % nsym x M
noiseVarU = max(noiseVar, 1e-9) * s^2;    % 防除零 + 尺度换算
llr = zeros(nsym * Qm, 1);
for b = 1:Qm
    b0 = bitTable(:, b) == 0;
    d0 = min(dist(:, b0), [], 2);
    d1 = min(dist(:, ~b0), [], 2);
    llr((0:nsym-1)*Qm + b) = (d0 - d1) / noiseVarU;
end
llr = max(min(llr, 30), -30);             % 限幅, 防Turbo译码数值溢出
end
