%% TD-LTE 物理层 (PUSCH结构) 环回测试主脚本
%  功能: 发送 -> XSRP RF环回 -> 接收 -> CRC校验

clear; clc;

%% 0. 添加路径
srcDir = fileparts(mfilename('fullpath'));
addpath(srcDir);
addpath(fullfile(srcDir, 'Tx'));
addpath(fullfile(srcDir, 'Rx'));

%% 1. 参数配置
prm = GetLTEParams();

%% 2. 发送端: 生成时域IQ信号
txIQ = OFDM_TxMain(prm);

%% 3. RF环回: 通过XSRP平台收发 (TX1/RX1)
% 使用TX1/RX1: 上位机 发射通道=TxCh1、接收通道=RxCh1, 天线接 TX1/RX1。
% 如需切换TX2/RX2: 上位机改 TxCh2/RxCh2、天线接TX2/RX2,
% 并把第6参数rxCh改为2 (修改Get_IQ命令第2字节, 该字节含义为推测)。
pcip   = '192.168.1.180';
xsrpip = '192.168.1.166';
rxIQ   = OFDM_RFLoopback(txIQ, pcip, xsrpip);

%% 4. 接收端: 解调并校验
[bits, crcOK] = OFDM_RxMain(rxIQ, prm);

%% 5. 结果输出
fprintf('CRC校验结果: %s\n', mat2str(crcOK));
