%% test_notch.m - 频域去干扰参数快速测试 (无需硬件)
% 用已保存的数据 (rx_accum_result.mat 的rxAvg 或 hw_capture.mat 的rxIQ)
% 测试不同 NotchMHz (0/0.5/1.0/1.5/2.0) 下的CRC, 找出最优去干扰宽度。
% 原理: 设备0~1MHz确定性干扰被擦除后, 剩余子载波只受随机噪声限制。
clear; clc;

srcDir = fileparts(mfilename('fullpath'));
addpath(srcDir);
addpath(fullfile(srcDir, 'Tx'));
addpath(fullfile(srcDir, 'Rx'));

if exist('rx_accum_result.mat', 'file')
    S = load('rx_accum_result.mat');
    rxIQ = S.rxAvg(:).';
    fprintf('使用 rx_accum_result.mat (多帧平均后信号)\n');
elseif exist('hw_capture.mat', 'file')
    S = load('hw_capture.mat');
    rxIQ = S.rxIQ(:).';
    fprintf('使用 hw_capture.mat (单帧信号)\n');
else
    error('没有可用数据: 先运行 rx_accum 或 verify_hw_link 生成 .mat 再测');
end

fprintf('\n==== NotchMHz 扫描 (软判决+MMSE+Turbo12+污染区隔离) ====\n');
nmList = [0 0.5 1.0 1.5 2.0 2.5 3.0];
crcList = NaN(size(nmList));
for ni = 1:numel(nmList)
    nm = nmList(ni);
    prm = GetLTEParams();
    prm.NotchMHz = nm;
    try
        [~, crcOK] = OFDM_RxMain(rxIQ, prm);
        crcList(ni) = crcOK;
        fprintf('NotchMHz=%.1f : CRC=%d\n', nm, crcOK);
    catch ME
        fprintf('NotchMHz=%.1f : 出错 %s\n', nm, ME.message);
    end
end
fprintf('\n>> CRC=1 的最小NotchMHz即最优(擦除最少又去掉干扰);\n');
fprintf('   若全为0: 干扰不只低频, 或信号仍太弱, 需加大平均帧数K或调天线。\n');

%% 记录结果到日志
logResult('test_notch', struct('notchMHz',nmList,'crcs',crcList));
