function symbols = Modulate(bits, mode)
% Modulate - 调制映射 (3GPP 36.211 7.1, QPSK/16QAM/64QAM)
% 输入:
%   bits - bit列向量 {0,1}
%   mode - 'QPSK' / '16QAM' / '64QAM'
% 输出:
%   symbols - 复数符号列向量 (归一化功率=1)
%
% LTE QAM 为"交错位"映射 (36.211 表7.1.2-1/7.1.3-1/7.1.4-1):
%   16QAM: I 由第1,3位, Q 由第2,4位;  2bit格雷: 00->+1,01->+3,11->-3,10->-1
%   64QAM: I 由第1,3,5位, Q 由第2,4,6位; 3bit格雷: 000->+3,001->+1,011->+7,
%          010->+5,110->-5,111->-7,101->-1,100->-3
% 归一化: QPSK 1/sqrt(2), 16QAM 1/sqrt(10), 64QAM 1/sqrt(42)

switch mode
    case 'QPSK'
        Qm = 2; s = sqrt(2);
        mI = [1 -1];                        % 1bit: 0->+1, 1->-1
        mQ = [1 -1];
    case '16QAM'
        Qm = 4; s = sqrt(10);
        mI = [1 3 -1 -3];                   % 2bit二进制序(00,01,10,11)->(+1,+3,-1,-3)
        mQ = [1 3 -1 -3];
    case '64QAM'
        Qm = 6; s = sqrt(42);
        mI = [3 1 5 7 -3 -1 -5 -7];         % 3bit二进制序(000..111)
        mQ = [3 1 5 7 -3 -1 -5 -7];
    otherwise
        error('Modulate: 不支持调制方式 %s', mode);
end

bits = bits(:);
nsym = floor(numel(bits) / Qm);
g = reshape(bits(1:Qm*nsym), Qm, nsym).';   % 每行一组

% 提取 I/Q 位 (交错) 并查表
iBits = g(:, 1:2:Qm);                       % 第1,3,5...位
qBits = g(:, 2:2:Qm);                       % 第2,4,6...位
iIdx = bi2de(iBits, 'left-msb') + 1;        % 组内二进制序号
qIdx = bi2de(qBits, 'left-msb') + 1;
% 注意: MATLAB 中 A(I) 结果方向与 A 一致(行向量A*列索引I仍返回行),
% 故用 reshape 强制为列向量
symbols = reshape(mI(iIdx) + 1i * mQ(qIdx), [], 1) / s;   % 归一化, 列向量
end
