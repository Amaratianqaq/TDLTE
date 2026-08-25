function [rxSync, rho, cfoHz, delay, phase] = syncLTE(rxIQ, txIQ, prm)
% syncLTE - 稳健同步: 粗频偏搜索 + 互相关定时 (已知发送信号时的首选方法)
% 输入:
%   rxIQ  - 接收信号 (1x30720, 复数)
%   txIQ  - 发送参考信号 (1x30720)
%   prm   - 参数结构体 (用 prm.NFFT)
% 输出:
%   rxSync - 对齐+去频偏后的信号 (与txIQ对齐, 供OFDM_RxMain使用)
%   rho    - 对齐后的归一化互相关 (信号强度指标)
%   cfoHz  - 估计频偏 (Hz)
%   delay  - 帧起点偏移 (样点, 0=已对齐)
%   phase  - 对齐信号与参考的公共相位 (rad), 供多帧相干平均时逐帧校正
%
% 原理: 两级频偏搜索(±8kHz, 250Hz粗搜→±200Hz细搜), 每档去频偏后与
%       txIQ 做全滞后互相关, 取峰值最高档; 不依赖CP相关(抗DC/低频干扰)。

r = rxIQ(:).' - mean(rxIQ);
v = txIQ(:).' - mean(txIQ);
N  = numel(r);
NFFT = prm.NFFT;
n = 0:N-1;

%% 1) 粗频偏搜索 ±8kHz / 250Hz
best = struct('cfo',0,'rho',0,'lag',0);
grid1 = -8000:250:8000;
for k = 1:numel(grid1)
    rc = r .* exp(-1j*2*pi*grid1(k)*n/NFFT);
    [rho, lag] = xcorrPeak(rc, v);
    if rho > best.rho
        best = struct('cfo',grid1(k),'rho',rho,'lag',lag);
    end
end

%% 2) 细频偏搜索 ±200Hz / 25Hz
grid2 = best.cfo + (-200:25:200);
for k = 1:numel(grid2)
    rc = r .* exp(-1j*2*pi*grid2(k)*n/NFFT);
    [rho, lag] = xcorrPeak(rc, v);
    if rho > best.rho
        best = struct('cfo',grid2(k),'rho',rho,'lag',lag);
    end
end

%% 3) 应用: 去频偏 + 循环移位对齐
rc = r .* exp(-1j*2*pi*best.cfo*n/NFFT);
% xcorr(r,v) 峰值滞后 = rx 中信号相对 tx 的延迟 d (r(i)≈v(i-d))
% 对齐: rxSync(i) = rc(i+d), 即循环左移 d
delay = best.lag;
rxSync = rc(mod((0:N-1) + delay, N) + 1);
cfoHz = best.cfo;
rho   = best.rho;
% 公共相位: 对齐信号与参考在lag=0的相关相位 (供多帧相干平均用)
c0 = sum(rxSync .* conj(v));
phase = angle(c0);
end

%% ================= 辅助 =================
function [rho, lag] = xcorrPeak(a, b)
c = xcorr(a, b);
[cmax, imax] = max(abs(c));
lag = imax - (numel(c)+1)/2;
rho = cmax / (sqrt(sum(abs(a).^2)) * sqrt(sum(abs(b).^2)));
end
