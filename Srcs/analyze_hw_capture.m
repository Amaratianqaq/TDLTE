%% analyze_hw_capture.m - 离线分析 hw_capture.mat (verify_hw_link保存的真实抓包)
% 目标: 确认接收信号特征 (方形IQ=无信号量化噪声? 削波? 有信号但弱?)
% 无需硬件, 直接运行。

clear; clc; close all;
load('hw_capture.mat');        % txIQ, rxIQ, prm

tx = txIQ(:).';  rx = rxIQ(:).';
N = numel(rx);
fprintf('tx样点数=%d, rx样点数=%d\n', numel(tx), numel(rx));

%% 1) 基本统计
fprintf('\n==== 统计 ====\n');
fprintf('tx: 峰值=%.4f RMS=%.4f DC=%.4g\n', max(abs(tx)), sqrt(mean(abs(tx).^2)), abs(mean(tx)));
fprintf('rx: 峰值=%.4f RMS=%.4f DC=%.4g  max|I|=%.4f max|Q|=%.4f\n', ...
    max(abs(rx)), sqrt(mean(abs(rx).^2)), abs(mean(rx)), max(abs(real(rx))), max(abs(imag(rx))));

% I/Q 是否独立均匀? 方形特征: I和Q的边缘分布近似均匀, 且I,Q几乎不相关
fprintf('\n==== 方形特征检查 ====\n');
fprintf('corr(I,Q)=%.4f (接近0=独立)\n', corrcoef(real(rx).', imag(rx).'));
fprintf('I直方图(16桶)计数: %s\n', mat2str(histcounts(real(rx), 16)));
fprintf('Q直方图(16桶)计数: %s\n', mat2str(histcounts(imag(rx), 16)));
fprintf('I唯一值数=%d, Q唯一值数=%d (量化位宽线索: ~2^b)\n', ...
    numel(unique(real(rx))), numel(unique(imag(rx))));

% 幅度分布: 均匀方形 -> 幅度三角形分布; 圆形信号 -> 瑞利分布
amp = abs(rx);
fprintf('幅度直方图(16桶): %s\n', mat2str(histcounts(amp, 16)));

%% 2) 频谱 (对比 tx/rx)
fprintf('\n==== 频谱特征 ====\n');
Fs = 30.72e6;
X = fft(tx, N); Y = fft(rx, N);
f = (0:N-1)/N*Fs; f(f > Fs/2) = f(f > Fs/2) - Fs;
band = abs(f) < 9.5e6 & abs(f) > 0.2e6;
% 有用频带(9.5MHz内, 除DC)能量占比
eTxBand = sum(abs(X(band)).^2) / sum(abs(X).^2);
eRxBand = sum(abs(Y(band)).^2) / sum(abs(Y).^2);
fprintf('tx有用频带能量占比=%.3f, rx有用频带能量占比=%.3f\n', eTxBand, eRxBand);
% DC与低频(0~1MHz)能量占比
lowBand = abs(f) < 1e6;
eRxDC = sum(abs(Y(lowBand)).^2) / sum(abs(Y).^2);
eTxDC = sum(abs(X(lowBand)).^2) / sum(abs(X).^2);
fprintf('tx低频(0~1MHz)能量占比=%.4f, rx低频(0~1MHz)能量占比=%.4f\n', eTxDC, eRxDC);

%% 3) 同步相关 (syncLTE)
[rxAl, rho, cfoHz, delay] = syncLTE(rx, tx, prm);
fprintf('\n==== syncLTE ====\n');
fprintf('rho=%.4f, 频偏=%.1f Hz, 帧偏移=%d 样点\n', rho, cfoHz, delay);

%% 4) 图形
figure('Name', '硬件抓包分析', 'Position', [80 80 1200 700]);
subplot(2,2,1); plot(real(tx(1:4000)), imag(tx(1:4000)), '.', 'MarkerSize', 1);
axis equal; grid on; title('发送时域IQ (前4000样点)');
subplot(2,2,2); plot(real(rx(1:4000)), imag(rx(1:4000)), '.', 'MarkerSize', 1);
axis equal; grid on; title('接收时域IQ (前4000样点)');
subplot(2,2,3); plot(f/1e6, 20*log10(abs(Y)+1e-9)); grid on;
xlabel('MHz'); ylabel('dB'); title('接收频谱');
xlim([-15.36 15.36]);
subplot(2,2,4); plot(f/1e6, 20*log10(abs(X)+1e-9)); grid on;
xlabel('MHz'); ylabel('dB'); title('发送频谱');
xlim([-15.36 15.36]);
