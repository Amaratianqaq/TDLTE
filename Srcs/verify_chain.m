%% verify_chain.m - 无信道直通端到端验证 (调用现有Tx/Rx模块)
%  功能: 发送链路(OFDM_TxMain) -> 理想信道直通 -> 接收链路(OFDM_RxMain)
%        -> 对比误码率与CRC校验, 验证修复后的模块是否正确工作
%  覆盖场景: QPSK / 16QAM / 64QAM (项目TBsize配置) + 小TB(含filler, 比特重复)场景
%  说明: 无需XSRP硬件, 直接运行本脚本即可

clear; clc;

srcDir = fileparts(mfilename('fullpath'));
addpath(srcDir);
addpath(fullfile(srcDir, 'Tx'));
addpath(fullfile(srcDir, 'Rx'));

allOK = true;

%% 场景1~3: 三种调制方式 (项目默认配置)
mods = {'QPSK', '16QAM', '64QAM'};
tbOf = [15840, 28336, 55056];
qmOf = [2, 4, 6];
for m = 1:numel(mods)
    prm = GetLTEParams();
    prm.Modulation = mods{m};
    prm.Qm = qmOf(m);
    prm.TBsize = tbOf(m);

    % 固定随机种子, 使发送信源可复现
    rng(42);
    txIQ = OFDM_TxMain(prm);
    rng(42);                                   % 重置种子, 重现发送端信源
    [txBits, ~] = TBGen(prm);

    rxIQ = txIQ;                               % 理想信道直通 (无噪声)
    [rxBits, crcOK] = OFDM_RxMain(rxIQ, prm);

    ber = mean(txBits ~= rxBits);
    pass = crcOK && ber == 0;
    fprintf('%s: 误码率=%g, CRC=%d (rxBits长度=%d) -> %s\n', mods{m}, ...
        ber, crcOK, numel(rxBits), ternary(pass, '通过', '失败'));
    allOK = allOK && pass;
end

%% 场景4: 小TB (含filler, 解速率匹配为比特重复场景)
prm = GetLTEParams();
prm.Modulation = 'QPSK';
prm.Qm = 2;
prm.TBsize = 100;

rng(42);
txIQ = OFDM_TxMain(prm);
rng(42);
[txBits, ~] = TBGen(prm);

rxIQ = txIQ;
[rxBits, crcOK] = OFDM_RxMain(rxIQ, prm);

ber = mean(txBits ~= rxBits(1:numel(txBits)));
pass = crcOK && ber == 0;
fprintf('QPSK小TB(100, 含filler): 误码率=%g, CRC=%d (rxBits长度=%d) -> %s\n', ...
    ber, crcOK, numel(rxBits), ternary(pass, '通过', '失败'));
allOK = allOK && pass;

fprintf('\n全部通过: %s\n', mat2str(allOK));

%% 记录结果到日志
logResult('verify_chain', struct('allOK',allOK));

%% 局部函数: 三元条件 (兼容老版本MATLAB)
function r = ternary(cond, a, b)
if cond
    r = a;
else
    r = b;
end
end
