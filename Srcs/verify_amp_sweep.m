%% verify_amp_sweep.m - OFDM发送幅度扫描 (找防削波工作点)
% 背景: 5MHz单音(幅度60%)能回来, 但满幅OFDM相关度≈0 -> 疑似接收端饱和/削波。
% 方法: 同一段OFDM信号分别以 2047/1500/1000/600/300 峰值发送,
%       每次抓回后做频偏校正+定时对齐, 计算与发送信号的互相关 rho。
% 预期: 若削波是主因, 幅度降低后 rho 应明显上升; 找到 rho 最高的幅度档。
% 前置: 天线保持当前(能出5MHz峰)的姿态固定不动, 上位机衰减2dB/增益22dB不变。
% 使用: 直接运行, 约30~60秒。

clear; clc;

srcDir = fileparts(mfilename('fullpath'));
addpath(srcDir);
addpath(fullfile(srcDir, 'Tx'));
addpath(fullfile(srcDir, 'Rx'));

prm = GetLTEParams();
rng(42);
txIQ = OFDM_TxMain(prm);
txIQ = txIQ(:).';

amps = [2047 1500 1000 600 300];
rhoAll = zeros(1, numel(amps));
fprintf('==== OFDM 幅度扫描 (请勿碰设备/天线) ====\n');
fprintf('%-8s %-10s %-10s %-10s %-10s %-10s\n', '峰值', 'rx功率', 'rho', 'rhoC', '帧偏移', '频偏Hz');
for ai = 1:numel(amps)
    a = amps(ai);
    rxIQ = OFDM_RFLoopback(txIQ, '192.168.1.180', '192.168.1.166', a);
    rxIQ = rxIQ(:).';
    [rxAl, offStar, cfoHz, rho] = syncAlign(rxIQ, txIQ, prm);
    u = rxAl - mean(rxAl);  v = txIQ - mean(txIQ);
    c = xcorr(conj(u), v);
    rhoC = max(abs(c)) / (sqrt(sum(abs(u).^2)) * sqrt(sum(abs(v).^2)));
    rhoAll(ai) = rho;
    fprintf('%-8d %-10.4g %-10.4f %-10.4f %-10d %-10.1f\n', ...
        a, mean(abs(rxIQ).^2), rho, rhoC, offStar-1, cfoHz);
    save(sprintf('amp_%d.mat', a), 'rxIQ', 'txIQ', 'rho', 'offStar', 'cfoHz');
end

%% 记录结果到日志
logResult('verify_amp_sweep', struct('amps',amps,'rhos',rhoAll));

fprintf('\n>> 看哪一档 rho 最高。若随幅度降低 rho 明显上升 -> 削波是主因,\n');
fprintf('   把该档峰值写入 OFDM_RFLoopback 的调用(第4参数), 或降低上位机接收增益。\n');

%% ================= 局部函数 =================
function [rxAl, offStar, cfoHz, rho] = syncAlign(rxIQ, txIQ, prm)
r = rxIQ(:).' - mean(rxIQ);
NFFT = prm.NFFT; L = numel(r);
ac = conj(r(1:end-NFFT)) .* r(NFFT+1:end);
w  = abs(r(1:end-NFFT)).^2 + abs(r(NFFT+1:end)).^2;
cfoSub = angle(sum(ac .* w)) / (2*pi);
cfoHz  = cfoSub * 15000;
n = 0:L-1;
rC = r .* exp(-1j*2*pi*cfoSub*n/NFFT);

pad = NFFT + 160;
rCp = [rC, rC(1:pad)];
acAll = conj(rCp(1:end-NFFT)) .* rCp(NFFT+1:end);
eAll1 = abs(rCp(1:end-NFFT)).^2;
eAll2 = abs(rCp(NFFT+1:end)).^2;
cp = prm.CPLengths;
symStart = cumsum([0, cp(1:13) + NFFT*ones(1,13)]);
num = zeros(1,L); den = zeros(1,L);
for l = 1:14
    cl = cp(l); s = symStart(l);
    numS = movsum(acAll, cl, 'Endpoints', 'discard');
    e1S  = movsum(eAll1, cl, 'Endpoints', 'discard');
    e2S  = movsum(eAll2, cl, 'Endpoints', 'discard');
    idx = mod((0:L-1) + s, L) + 1;
    num = num + numS(idx);
    den = den + sqrt(e1S(idx) .* e2S(idx));
end
metric = num ./ max(den, 1e-12);
[~, offStar] = max(metric);
rxAl = [rC(offStar:end), rC(1:offStar-1)];
u = rxAl - mean(rxAl);
v = txIQ(:).' - mean(txIQ);
c = xcorr(u, v);
rho = max(abs(c)) / (sqrt(sum(abs(u).^2)) * sqrt(sum(abs(v).^2)));
end
