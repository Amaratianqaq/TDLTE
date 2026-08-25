%% verify_rho.m - 单次OFDM环回快速检测 (调增益/衰减后快速看rho)
% 用法: 修改上位机接收增益/发射衰减后, 运行本脚本看 rho 是否上升。
% 输出: rx功率 / rho(频偏+定时对齐后互相关) / 频谱包络相关(判断OFDM能量是否在回传里)
%   - rho高(>0.3)      -> 信号回来了, 链路OK, 去跑 verify_hw_link 看CRC
%   - rho低但包络相关高 -> OFDM能量在, 但相位/定时处理失败(继续调同步)
%   - 两者都低          -> 信号仍没回来, 继续调增益/衰减/天线
% 参数: 第4参数为发送峰值(默认1000, 防饱和)。

clear; clc;

srcDir = fileparts(mfilename('fullpath'));
addpath(srcDir);
addpath(fullfile(srcDir, 'Tx'));
addpath(fullfile(srcDir, 'Rx'));

ampStr = input('发送峰值(12位, 默认1000, 可输入2047/1500/600): ', 's');
if isempty(ampStr), amp = 1000; else, amp = str2double(ampStr); end
paprStr = input('PAPR削减倍数(默认0=不削减, 可输入1.5/1.2): ', 's');
if isempty(paprStr), papr = 0; else, papr = str2double(paprStr); end

prm = GetLTEParams();
rng(42);
txIQ = OFDM_TxMain(prm);
txIQ = txIQ(:).';

rxIQ = OFDM_RFLoopback(txIQ, '192.168.1.180', '192.168.1.166', amp, papr);
rxIQ = rxIQ(:).';

% 稳健同步: 频偏搜索 + 互相关定时 (见 syncLTE.m)
[rxAl, rho, cfoHz, delay] = syncLTE(rxIQ, txIQ, prm);

% 频谱包络相关 (平滑后, 抗梳齿错位: 只比频谱形状, 不看kHz级对齐)
N = numel(rxIQ);
X = fft(txIQ, N);  Y = fft(rxIQ, N);
f = (0:N-1)/N*30.72e6; f(f>15.36e6) = f(f>15.36e6) - 30.72e6;
band = abs(f) < 9.5e6 & abs(f) > 0.2e6;
K = 150;                                    % 平滑窗 150kHz
smX = movmean(abs(X), K); smY = movmean(abs(Y), K);
rhoEnv = corrcoef(smX(band), smY(band)); rhoEnv = rhoEnv(1,2);
rhoEnvRaw = corrcoef(abs(X(band)), abs(Y(band))); rhoEnvRaw = rhoEnvRaw(1,2);

fprintf('==== 单次OFDM环回 (峰值=%d) ====\n', amp);
% 削波率: |样点| > 0.9*max 的比例 (硬限幅特征) ; 接收PAPR
clipFrac = mean(abs(rxIQ(:)) > 0.9 * max(abs(rxIQ(:))));
paprx = max(abs(rxIQ)) / sqrt(mean(abs(rxIQ).^2));
fprintf('rx功率=%.4f | 削波率=%.1f%% | 接收PAPR=%.2f | rho=%.4f | 平滑包络相关=%.4f (原始=%.4f) | 帧偏移=%d | 频偏=%.1f Hz\n', ...
    mean(abs(rxIQ).^2), clipFrac*100, paprx, rho, rhoEnv, rhoEnvRaw, delay, cfoHz);

% 回传频带能量表 (发OFDM时看回传是否跟随宽带形状)
bands = [1 3 5 7 9 11 13];
bp = zeros(1, numel(bands));
for k = 1:numel(bands)
    bm = abs(abs(f) - bands(k)*1e6) < 0.5e6;
    bp(k) = max(abs(Y(bm)).^2);
end
medY = median(abs(Y(abs(f) > 0.2e6)).^2);
fprintf('回传频带峰值/全局中位数: ');
fprintf('%.0fMHz:%.0fx  ', [bands; bp/medY]);
fprintf('\n');

%% 记录结果到日志
logResult('verify_rho', struct('amp',amp,'papr',papr,'rho',rho,'rhoEnv',rhoEnv, ...
    'rxpow',mean(abs(rxIQ).^2),'clipPct',clipFrac*100,'delay',delay,'cfoHz',cfoHz));

if rho > 0.3
    fprintf('>> 信号回来了! 去跑 verify_hw_link 看CRC。\n');
elseif rhoEnv > 0.5
    fprintf('>> OFDM能量在回传里, 但相关仍低 -> 继续优化同步/看是否干扰淹没。\n');
elseif mean(abs(rxIQ).^2) > 0.5
    fprintf('>> 回传功率接近满幅(饱和?) 且无信号 -> 降低接收增益或加大发射衰减再试。\n');
else
    fprintf('>> 信号没回来 -> 调增益/衰减/天线姿态再试。\n');
end
