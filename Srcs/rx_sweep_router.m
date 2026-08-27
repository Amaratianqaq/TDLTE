%% rx_sweep_router.m - 扫描 0x68 路由/环回命令的值 (找"真接收"模式)
% 背景: rx_diag_data 证实设备在回环"最后发送的帧"(rho~0.29, 内容随数据变)。
%       怀疑 0x68 命令字节5='06' 选择了"发送数据环回"而非"真实RX接收"。
% 本脚本: 对一组候选值, 每个值做: 发X->采A | 不发->采C, 输出:
%   rho(A,X)      : 回传与发送帧的相干度
%   rho(C,X)      : 控制帧是否仍含"上一帧回环" (高=还在回环)
%   corr(A,C)     : 信号帧与控制帧相似度 (高=控制里也是回环副本)
%   rho(D,X)      : A - alpha*C 之后残差相关 (真直达信号)
%   >9MHz占比     : 带外失真能量 (回环失真的指标)
% 判读: 理想"真接收"模式应满足: rho(A,X) 高(0.5+), 且 rho(C,X) 低(控制帧干净),
%       corr(A,C) 低。若某值出现该组合, 即找到正确模式!
% 使用: 直连+1GHz+衰减2/增益5 状态运行, 约8~12分钟, 勿碰设备。

clear; clc; close all;
addpath(fileparts(mfilename('fullpath')));
addpath(fullfile(fileparts(mfilename('fullpath')), 'Tx'));
addpath(fullfile(fileparts(mfilename('fullpath')), 'Rx'));

SAMPLE_LENGTH = 30720;
B = {'00','00','99','bb'};
cmdSync   = uint8(hex2dec({'00','00','99','bb','69','00','00','00','00','00','00','00','00','00','00','00','00','00','00','00'}));
cmdDelay  = uint8(hex2dec({'00','00','99','bb','67','00','00','00','00','00','00','00','00','00','00','00','00','00','00','00'}));
cmdTx     = uint8(hex2dec([B, '65','02','00','03','00','01','78','00', repmat({'00'},1,8)]));
cmdSend   = uint8(hex2dec({'00','00','99','bb','64','00','00','00','00','00','00','00','00','00','00','00','00','00','00','00'}));
cmdGet    = uint8(hex2dec([B, '66','01','00','01','00','80','00','F0', repmat({'00'},1,8)]));
pcip = '192.168.1.180'; xsrpip = '192.168.1.166';

prm = GetLTEParams();
rng(42); txX = OFDM_TxMain(prm); txX = txX(:).';
dataX = packIQ(txX, SAMPLE_LENGTH);

% 候选路由值: '06'为当前(基线), 其它为猜测
cands = {'00','01','02','03','04','05','06','07','08','09','0A','0B'};
% 再加一个"完全不发0x68"的情况
cands{end+1} = 'NONE';

fprintf('==== 0x68 路由值扫描 (直连/1GHz, 勿碰设备) ====\n');
fprintf('%-6s | %-9s | %-9s | %-9s | %-9s | %-9s | %-9s\n', ...
    '路由', 'rho(A,X)', 'rho(C,X)', 'corr(A,C)', 'rho(D,X)', '消能%', '>9M%');

results = [];
for ci = 1:numel(cands)
    val = cands{ci};
    if strcmp(val, 'NONE')
        cmdRouter = [];
    else
        cmdRouter = uint8(hex2dec([B, '68','00','00','00', '00',val,'00','00', repmat({'00'},1,8)]));
    end

    rxA = udpLoopback(dataX, cmdSync, cmdRouter, cmdDelay, cmdTx, cmdSend, cmdGet, SAMPLE_LENGTH, pcip, xsrpip);
    rxC = udpLoopback([],   cmdSync, cmdRouter, cmdDelay, cmdTx, [],      cmdGet, SAMPLE_LENGTH, pcip, xsrpip);
    if isempty(rxA) || isempty(rxC)
        fprintf('%-6s | 未收到数据, 跳过\n', val);
        continue;
    end
    rxA = rxA(:).'; rxC = rxC(:).';

    [~, rhoAX, ~, ~] = syncLTE(rxA, txX, prm);
    [~, rhoCX, ~, ~] = syncLTE(rxC, txX, prm);
    corrAC = max(abs(xcorr(rxA-mean(rxA), rxC-mean(rxC)))) / ...
        (sqrt(sum(abs(rxA-mean(rxA)).^2)) * sqrt(sum(abs(rxC-mean(rxC)).^2)));

    % 对消残差
    cX = xcorr(rxA - mean(rxA), rxC - mean(rxC));
    [~, imax] = max(abs(cX));
    lagC = imax - (numel(cX)+1)/2;
    C_al = circshift(rxC, -lagC);
    alpha = sum(conj(C_al).*rxA) / sum(abs(C_al).^2);
    D = rxA - alpha*C_al;
    [~, rhoD, ~, ~] = syncLTE(D, txX, prm);
    removed = (1 - sum(abs(D).^2)/sum(abs(rxA).^2)) * 100;

    % >9MHz 能量占比
    Xf = fft(rxA); f = (0:SAMPLE_LENGTH-1)/SAMPLE_LENGTH*30.72e6;
    f(f>15.36e6) = f(f>15.36e6) - 30.72e6;
    P = abs(Xf).^2;
    outPct = 100*sum(P(abs(f)>9e6)) / sum(P);

    fprintf('%-6s | %-9.4f | %-9.4f | %-9.4f | %-9.4f | %-9.1f | %-9.1f\n', ...
        val, rhoAX, rhoCX, corrAC, rhoD, removed, outPct);
    results = [results; struct('val',val,'rhoAX',rhoAX,'rhoCX',rhoCX,'corrAC',corrAC, ...
        'rhoD',rhoD,'removed',removed,'outPct',outPct)]; %#ok<AGROW>
end

%% 判定: 找"真接收"特征 = rho(A,X)高 且 rho(C,X)低 且 corr(A,C)低
fprintf('\n==== 判定 ====\n');
for r = results.'
    if r.rhoAX > 0.4 && r.rhoCX < 0.15 && r.corrAC < 0.4
        fprintf('>> 路由值 %s: rho(A,X)=%.3f 高, rho(C,X)=%.3f 低, corr(A,C)=%.3f 低 -> 疑似真接收模式!!\n', ...
            r.val, r.rhoAX, r.rhoCX, r.corrAC);
    end
end
if isempty(results) || ~any([results.rhoAX] > 0.4 & [results.rhoCX] < 0.15 & [results.corrAC] < 0.4)
    fprintf('>> 所有候选值都没有出现"高rho(A,X)+低rho(C,X)"组合 -> 0x68不是开关, 回环在设备固件内,\n');
    fprintf('   只能等厂商 TDLTE_PHY_ALL_main 环境。\n');
end
fprintf('>> 注: 若某值回传变成静音/全0, 说明该字节含义不是环回选择, 记下即可。\n');

save('rx_sweep_router_result.mat', 'results');
logResult('rx_sweep_router', struct('vals',strjoin(cands,','), ...
    'rhoAX',[results.rhoAX],'rhoCX',[results.rhoCX],'corrAC',[results.corrAC],'rhoD',[results.rhoD]));
fprintf('结果已存 rx_sweep_router_result.mat\n');

%% ================= 局部函数 =================
function dataIQ = packIQ(txIQ, SAMPLE_LENGTH)
txIQ = txIQ(:).';
len = SAMPLE_LENGTH * 2;
dataIQ = zeros(1, len);
dataIQ(1,1:2:len-1) = real(txIQ);
dataIQ(1,2:2:len)   = imag(txIQ);
dataIQ = dataIQ .* (2047 / max(abs(dataIQ)));
dataIQ = fix(dataIQ);
for n = 1:len
    if dataIQ(n) > 2047
        dataIQ(n) = 2047;
    elseif dataIQ(n) < 0
        dataIQ(n) = 4096 + dataIQ(n);
    end
end
dataIQ(1,1:2:len-1) = dataIQ(1,1:2:len-1) .* 16;
dataIQ(1,2:2:len)   = fix(dataIQ(1,2:2:len)./256) + rem(dataIQ(1,2:2:len),256).*256;
dataIQ = uint16(dataIQ);
end

function rxIQ = udpLoopback(dataIQ, cmdSync, cmdRouter, cmdDelay, cmdTx, cmdSend, cmdGet, SAMPLE_LENGTH, pcip, xsrpip)
rxIQ = [];
udp_obj = udp(xsrpip,13344,'LocalHost',pcip,'LocalPort',12345,'TimeOut',15, ...
    'OutputBufferSize',61440,'InputBufferSize',61440*10);
udp_obj.Timeout = 15;
try
    fopen(udp_obj);
    if ~isempty(cmdSync),   fwrite(udp_obj, cmdSync,   'uint8'); end
    if ~isempty(cmdRouter), fwrite(udp_obj, cmdRouter, 'uint8'); end
    if ~isempty(cmdDelay),  fwrite(udp_obj, cmdDelay,  'uint8'); end
    if ~isempty(cmdTx),     fwrite(udp_obj, cmdTx,     'uint8'); end
    if ~isempty(dataIQ)
        fwrite(udp_obj, cmdSend, 'uint8');
        SEND_PACKET_LENGTH = 512;
        for pn = 1:fix(SAMPLE_LENGTH*2/SEND_PACKET_LENGTH)
            fwrite(udp_obj, dataIQ(1,((pn-1)*SEND_PACKET_LENGTH+1):(pn*SEND_PACKET_LENGTH)), 'uint16');
        end
    end
    if ~isempty(cmdGet), fwrite(udp_obj, cmdGet, 'uint8'); end
    RECV_PACKET_LENGTH = 240*4;
    data_byte_numb = SAMPLE_LENGTH*4;
    buf = zeros(1, SAMPLE_LENGTH*4);
    data_byte_c = 0;
    recvInd = 1;
    while recvInd == 1
        [udp_data, count] = fread(udp_obj, RECV_PACKET_LENGTH);
        for pj = 1:count
            buf(data_byte_c+pj) = udp_data(pj);
        end
        data_byte_c = data_byte_c + count;
        if (data_byte_c >= data_byte_numb) || (data_byte_c == 0)
            recvInd = 0;
        end
    end
    try
        fclose(udp_obj);
    catch
    end
    if data_byte_c > 0
        N = floor(data_byte_c/4);
        b = double(buf(1:N*4));
        o = (0:N-1)*4;
        iw = b(o+1) + b(o+2)*256;
        qw = b(o+3) + b(o+4)*256;
        iv = bitshift(iw,-8) + bitand(iw,15)*256;
        qv = bitshift(qw,-8) + bitand(qw,15)*256;
        iv(iv>2047) = iv(iv>2047) - 4096;
        qv(qv>2047) = qv(qv>2047) - 4096;
        rxIQ = iv + 1j*qv;
    end
catch
end
try
    fclose(udp_obj);
catch
end
delete(udp_obj); clear udp_obj;
end
