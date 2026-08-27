%% grid_probe.m - 观察接收OFDM网格的频域能量分布 (无需硬件)
% 用已保存的 rx_accum_result.mat (多帧平均) 看干扰在OFDM网格中的位置:
%   - 若功率集中在低频行(601附近) -> 干扰可擦除, 配合NotchMHz有救
%   - 若功率平坦分布 -> 干扰铺满全频段, 擦除无用, 确认为设备侧
% 使用: 先跑过 rx_accum 生成数据, 再运行本脚本。

clear; clc;
addpath(fileparts(mfilename('fullpath')));
addpath(fullfile(fileparts(mfilename('fullpath')), 'Tx'));
addpath(fullfile(fileparts(mfilename('fullpath')), 'Rx'));

if ~exist('rx_accum_result.mat', 'file')
    error('没有 rx_accum_result.mat, 请先运行 rx_accum');
end
S = load('rx_accum_result.mat');
rxIQ = S.rxAvg(:).';
prm = GetLTEParams();

%% OFDM解调得到网格
rxGrid = OFDMDemod(rxIQ, prm);
Nsc = prm.NSubc;

%% 每子载波平均功率 (对12个数据符号平均; 导频列也包含, 不影响分布观察)
pow = mean(abs(rxGrid).^2, 2);        % 1200x1
med = median(pow);
fprintf('网格子载波功率分布 (行601=DC):\n');
fprintf('子载波范围        | 相对中位数倍数\n');
bin = 50;  % 每50行一段
for k = 1:bin:Nsc
    rngRows = k:min(k+bin-1, Nsc);
    seg = mean(pow(rngRows)) / med;
    % 标注DC附近
    mark = '';
    if k <= 601 && k+bin-1 >= 601, mark = ' <-- DC'; end
    fprintf('行 %4d~%4d       | %8.1f x%s\n', k, rngRows(end), seg, mark);
end
[pmax, imax] = max(pow);
fprintf('\n功率最大子载波: 行%d (频率约 %+.1f MHz), 相对中位数 %.1f x\n', ...
    imax, (imax-601)*0.015, pmax/med);
fprintf('总功率: 低频行(601±133, ±2MHz)占比 = %.1f%%\n', ...
    100*sum(pow(601-133:601+133))/sum(pow));
fprintf('其余(±2~9MHz) 占比 = %.1f%%\n', ...
    100*(1 - sum(pow(601-133:601+133))/sum(pow)));

%% 画图
figure('Name','接收网格子载波功率');
x = ((1:Nsc) - 601) * 0.015;          % MHz
plot(x, 10*log10(pow/med+eps), 'b'); grid on; hold on;
xline(0,'r--','DC'); xline(2,'m--','+2MHz'); xline(-2,'m--','-2MHz');
xlabel('频率 (MHz)'); ylabel('相对中位数 (dB)');
title('接收OFDM网格子载波功率分布');
xlim([-9 9]);
logResult('grid_probe', struct('maxRow',imax,'maxFreqMHz',(imax-601)*0.015, ...
    'lowFreqPct',100*sum(pow(601-133:601+133))/sum(pow)));
