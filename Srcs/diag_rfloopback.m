%% diag_rfloopback.m - XSRP硬件环回诊断 (定位时延/相关度/频偏/格式问题)
%  功能: 生成发送IQ -> XSRP环回 -> 量化分析接收与发送的差异
%  输出: 控制台打印诊断指标, 保存 diag_result.mat (txIQ/rxIQ/指标)
%  用法: 硬件连接正常后直接运行; 需要 Signal Processing Toolbox (xcorr)

clear; clc; close all;

srcDir = fileparts(mfilename('fullpath'));
addpath(srcDir);
addpath(fullfile(srcDir, 'Tx'));
addpath(fullfile(srcDir, 'Rx'));

prm = GetLTEParams();
txIQ = OFDM_TxMain(prm);                                   % 1x30720 发送IQ

rmsAmp = @(x) sqrt(mean(abs(x).^2));
fprintf('发送信号: 样点数=%d, 峰值=%.3f, RMS=%.3f\n', ...
    numel(txIQ), max(abs(txIQ)), rmsAmp(txIQ));

%% 1) XSRP环回 (需硬件在线)
pcip   = '192.168.1.180';
xsrpip = '192.168.1.166';
rxIQ   = OFDM_RFLoopback(txIQ, pcip, xsrpip);
fprintf('接收信号: 样点数=%d, 峰值=%.3f, RMS=%.3f\n', ...
    numel(rxIQ), max(abs(rxIQ)), rmsAmp(rxIQ));

%% 2) 互相关: 时延 + 归一化相关度 (判断"数据内容是否一致")
[corrVal, lags] = xcorr(rxIQ, txIQ, 4096);
[peak, idx] = max(abs(corrVal));
delay = lags(idx);
normCorr = peak / sqrt(sum(abs(rxIQ).^2) * sum(abs(txIQ).^2));
fprintf('\n[互相关] 峰值位置(时延)=%d 样点, 归一化相关度=%.4f\n', delay, normCorr);
fprintf('  相关度>0.9: 内容基本一致(大概率只是时延/幅度问题);\n');
fprintf('  相关度<0.5: 内容不一致(字节序/位序/IQ交叉/采样率/频偏严重)\n');

%% 3) 对齐后频偏估计 (相位斜率 -> 频率偏差)
fd = NaN;
if abs(delay) < numel(rxIQ) - 4096 && peak > 0
    if delay >= 0
        segTx = txIQ;
        segRx = rxIQ(delay+1 : delay + numel(segTx));
    else
        segRx = rxIQ(1 : numel(rxIQ) + delay);
        segTx = txIQ(1-delay : numel(txIQ));
    end
    nseg = min(numel(segRx), numel(segTx));
    r = segRx(1:nseg) .* conj(segTx(1:nseg));
    p = polyfit((0:nseg-1).', unwrap(angle(r)), 1);
    fd = p(1) / (2*pi);
    fprintf('[频偏] 估计频偏 = %.1f Hz (%%f=%.3f ppm @ 2.38GHz)\n', ...
        fd, fd / 2.38e9 * 1e6);
end

%% 4) I/Q 交叉与倒序检查 (格式错误的常见形态)
nchk = min(2048, numel(rxIQ));
if delay >= 0 && delay < numel(rxIQ) - nchk
    a = txIQ(1:nchk).';
    b = rxIQ(delay+1 : delay+nchk).';
else
    a = txIQ(1:nchk).';
    b = rxIQ(1:nchk).';
end
c_ii = abs(sum(a .* conj(b)) / nchk);
c_qi = abs(sum(a .* conj(1i*b)) / nchk);       % Q作I (90度旋转)
c_iq = abs(sum(1i*a .* conj(b)) / nchk);
fprintf('\n[I/Q检查] 同相相关=%.4f, I->Q相关=%.4f, Q->I相关=%.4f\n', ...
    c_ii, c_iq, c_qi);
fprintf('  若同相相关明显小于I->Q/Q->I: 接收I/Q两路对调或交叉\n');

%% 5) 星座图对比 (当前无同步假设下的解调结果)
rxGrid = OFDMDemod(rxIQ, prm);
[rs0, rs1] = RefSignalGen(prm);
H = ChanEst(rxGrid, rs0, rs1, prm);
eqGrid = Equalize(rxGrid, H, 'ZF');
dataSyms = ExtractData(eqGrid, prm);
figure('Name', '诊断星座图');
subplot(1,2,1);
plot(real(dataSyms(1:4000)), imag(dataSyms(1:4000)), '.');
axis equal; grid on; title('接收均衡后星座图 (无同步)');
subplot(1,2,2);
% 发送星座参考: 按调制方式画标准星座点
switch prm.Modulation
    case 'QPSK'
        s = sqrt(2); ref = [1+1i, 1-1i, -1+1i, -1-1i]/s;
    case '16QAM'
        s = sqrt(10); mI = [1 3 -1 -3];
        [I, Q] = ndgrid(mI, mI); ref = (I(:) + 1i*Q(:)).'/s;
    case '64QAM'
        s = sqrt(42); mI = [3 1 5 7 -3 -1 -5 -7];
        [I, Q] = ndgrid(mI, mI); ref = (I(:) + 1i*Q(:)).'/s;
end
plot(real(ref), imag(ref), 'r+', 'MarkerSize', 8);
axis equal; grid on; title('发送星座参考');

save('diag_result.mat', 'txIQ', 'rxIQ', 'delay', 'normCorr', 'fd');
fprintf('\n诊断数据已保存: diag_result.mat\n');
