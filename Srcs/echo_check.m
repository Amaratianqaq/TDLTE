%% echo_check.m - 检查回波/多径 (假说: 延迟>CP的回波导致OFDM ISI)
% 用已保存的 rx_accum_result.mat:
%   1) 从导频估计信道H, IFFT得冲激响应h, 看CP(144样点)之外有无显著能量
%   2) 对xcorr(rx,tx)找主峰之外的次峰(回波延迟)
% 若CP外有明显能量 -> 回波确认 -> 解释单音通/OFDM不通的所有现象!

clear; clc;
addpath(fileparts(mfilename('fullpath')));
addpath(fullfile(fileparts(mfilename('fullpath')), 'Tx'));
addpath(fullfile(fileparts(mfilename('fullpath')), 'Rx'));

if ~exist('rx_accum_result.mat','file')
    error('没有 rx_accum_result.mat, 请先运行 rx_accum');
end
S = load('rx_accum_result.mat');
rxAvg = S.rxAvg(:).';
prm = GetLTEParams();
NFFT = prm.NFFT;

%% 同步
rng(42);
txIQ = OFDM_TxMain(prm); txIQ = txIQ(:).';
[rxAl, rho, ~, delay] = syncLTE(rxAvg, txIQ, prm);
fprintf('同步rho=%.4f, 帧偏移=%d\n', rho, delay);

%% 1) xcorr次峰检测 (回波会在另一滞后处产生次峰)
c = xcorr(rxAl, txIQ);
c = abs(c);
[cmax, imax] = max(c);
% 找主峰外的局部最大值 (间隔>50样点)
pk = [];
for lag = 51:numel(c)-50
    if c(lag) >= c(lag-1) && c(lag) >= c(lag+1) && c(lag) > 0.1*cmax && lag ~= imax
        pk = [pk; lag, c(lag)/cmax]; %#ok<AGROW>
    end
end
if isempty(pk)
    fprintf('xcorr无显著次峰 (回波不明显)\n');
else
    [~, ii] = sort(pk(:,2), 'descend');
    fprintf('xcorr次峰 (相对主峰):\n');
    for k = 1:min(5, numel(ii))
        lagAbs = pk(ii(k),1) - (numel(c)+1)/2;
        fprintf('  滞后 %+d 样点, 相对幅度 %.3f (%+.1f dB)\n', ...
            lagAbs, pk(ii(k),2), 20*log10(pk(ii(k),2)+eps));
    end
end

%% 2) 信道冲激响应: 导频LS -> 排除污染区 -> IFFT
[rs0, rs1] = RefSignalGen(prm);
rxGrid = OFDMDemod(rxAl, prm);
pilotIdx1 = prm.PilotSymbols(1)+1;
Hp = rxGrid(:, pilotIdx1) ./ rs0;      % 全频域LS (1200点)
% 排除污染区行451~750 (92%功率的低频干扰) 和边缘, 只留干净子载波
clean = true(1200,1);
clean(451:750) = false;
Hc = Hp;  Hc(~clean) = 0;
% IFFT到冲激响应 (1200点 = 66.7us, 每点=15kHz间隔对应1/(1200*15k)=55.6ns)
h = ifft(Hc);
p = abs(h).^2;
ptot = sum(p);
CP_samples = 144;      % CP=144样点@30.72MHz = 4.69us
% 冲激响应每点 = 55.6ns; CP对应 144*32.55ns... 换算: 1/(1200*15000)=55.6ns/点
CP_pts = round(144 / 32.55);   % 144样点@30.72MHz=4.69us -> 在55.6ns网格上=84点
fprintf('\n信道冲激响应 (IFFT of 干净子载波LS):\n');
fprintf('主径位置=样点%d, 相对能量=%.1f%%\n', find(p==max(p),1), 100*max(p)/ptot);
% CP内 vs CP外能量
CP_pts = 84;  % 4.69us / 55.6ns
energyCP = sum(p(1:CP_pts));
energyOut = sum(p(CP_pts+1:end));
fprintf('CP内(0~%.1fus)能量=%.1f%% | CP外(>%.1fus)能量=%.1f%%\n', ...
    CP_pts*0.0556, 100*energyCP/ptot, CP_pts*0.0556, 100*energyOut/ptot);
% CP外最强径
[pm, im] = max(p(CP_pts+1:end));
if energyOut/ptot > 0.03
    fprintf('>> CP外有明显能量(%.1f%%)! 最强径在%.1fus处 -> 回波/多径确认!!\n', ...
        100*energyOut/ptot, (CP_pts+im)*0.0556);
    fprintf('   延迟超过CP -> OFDM产生ISI, 平坦均衡无法消除 -> 解释全部现象!\n');
else
    fprintf('>> CP外能量仅%.1f%% -> 无明显多径, 回波假说不成立。\n', 100*energyOut/ptot);
end
figure('Name','信道冲激响应');
bar((0:numel(p)-1)*0.0556, 10*log10(p/max(p)+eps)); grid on;
xlabel('延迟 (us)'); ylabel('相对能量 (dB)');
xline(CP_pts*0.0556, 'r--', 'CP边界(4.69us)');
title('信道冲激响应 (导频LS-IFFT, 干净子载波)');
logResult('echo_check', struct('rho',rho,'cpIn',100*energyCP/ptot,'cpOut',100*energyOut/ptot));
