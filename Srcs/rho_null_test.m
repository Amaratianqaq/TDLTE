%% rho_null_test.m - 相关度真伪检验
% 问题: 回传与发送信号的 rho≈0.1, 到底是"信号弱但在"还是"虚假相关"?
% 方法: 一次真实环回后, 分别与
%   参考A: 实际发送的OFDM (正确参考)
%   参考B: 另一随机种子的OFDM (结构相同数据不同, 错误参考)
%   参考C: 纯随机复数信号 (同功率)
% 计算同步后的rho。判定:
%   rhoA 显著高于 rhoB/rhoC -> 相关是真实的(信号确实在回传里, 只是弱)
%   三者相近               -> 0.1左右的rho是虚假相关(设备内容所致)
% 前置: 保持当前上位机设置与天线姿态不变。

clear; clc;

srcDir = fileparts(mfilename('fullpath'));
addpath(srcDir);
addpath(fullfile(srcDir, 'Tx'));
addpath(fullfile(srcDir, 'Rx'));

prm = GetLTEParams();
rng(42); txA = OFDM_TxMain(prm); txA = txA(:).';   % 正确参考(实际发送)
rng(7);  txB = OFDM_TxMain(prm); txB = txB(:).';   % 错误OFDM参考(同结构不同数据)
rng(99); txC = (randn(1,30720) + 1j*randn(1,30720)); % 随机参考
txC = txC / norm(txC) * norm(txA);

%% 一次真实环回 (峰值2047, 不PAPR削减)
rxIQ = OFDM_RFLoopback(txA, '192.168.1.180', '192.168.1.166', 2047, 0);
rxIQ = rxIQ(:).';

%% 分别同步并计算rho
[~, rA, ~, ~] = syncLTE(rxIQ, txA, prm);
[~, rB, ~, ~] = syncLTE(rxIQ, txB, prm);
[~, rC, ~, ~] = syncLTE(rxIQ, txC, prm);

fprintf('\n==== rho 真伪检验 ====\n');
fprintf('rho(正确参考A)   = %.4f\n', rA);
fprintf('rho(错误OFDM参考B)= %.4f\n', rB);
fprintf('rho(随机参考C)   = %.4f\n', rC);
fprintf('纯噪声期望上限   ≈ 0.03 (max over ~3万滞后+频偏搜索)\n');

if rA > 2*max(rB, rC) + 0.02
    fprintf('\n>> 正确参考相关显著更高 -> rho=%.4f 是真实的: 信号确实在回传里(但弱, SNR约-7~-9dB)。\n', rA);
    fprintf('   方向: 继续强化天线耦合/电平, 把弱信号推上去。\n');
else
    fprintf('\n>> 各参考相关相近 -> rho≈%.4f 是虚假相关(设备内容所致), 与是否发送无关。\n', rA);
    fprintf('   方向: 天线/参数/设备路径问题, 走厂商路线。\n');
end

%% 记录结果到日志
logResult('rho_null_test', struct('rhoCorrect',rA,'rhoWrongOFDM',rB,'rhoRandom',rC));
