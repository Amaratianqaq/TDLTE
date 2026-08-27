%% verify_sweep.m - 命令参数组合黑盒扫描
% 背景: 射频路径"收不到自己的信号"(rho≈0)。在缺少设备协议文档的情况下,
%       系统化尝试 tx_command(变体A/B) x Get_IQ采集时隙(00/01/03) 共6组,
%       每组发送同一段伪随机IQ序列并抓回, 计算 rho/rhoC/rhoB 与字节统计,
%       找出能让信号回来的组合 (rho显著升高者即为有效配置)。
% 前置: 上位机已配置 TX_Freq=RX_Freq(如2.38G), 衰减建议按手册用2dB,
%       天线连接, 运行方式=真实系统。
% 运行: 直接运行, 约1~2分钟 (6组, 每组含UDP收发与相关计算)。

clear; clc;
addpath(fileparts(mfilename('fullpath')));   % 确保 logResult 可用

SAMPLE_LENGTH = 30720;

%% 1) 测试图案与打包 (同 verify_udp_format)
rng(7);
txI = randi([0 4095], 1, SAMPLE_LENGTH);
txQ = randi([0 4095], 1, SAMPLE_LENGTH);
len = SAMPLE_LENGTH * 2;
dataIQ = zeros(1, len);
dataIQ(1,1:2:len-1) = txI;
dataIQ(1,2:2:len)   = txQ;
dataIQ(1,1:2:len-1) = dataIQ(1,1:2:len-1) .* 16;
dataIQ(1,2:2:len)   = fix(dataIQ(1,2:2:len)./256) + rem(dataIQ(1,2:2:len),256).*256;
dataIQ = uint16(dataIQ);
zt = (double(txI) - 2048) + 1j*(double(txQ) - 2048);

%% 2) 命令模板 (帧头 00 00 99 bb)
B = {'00','00','99','bb'};
txCmdA = [B, '65','02','00','03','00','01','78','00', repmat({'00'},1,8)];  % 变体A(2时隙)
txCmdB = [B, '65','0A','03','FF','00','03','78','00', repmat({'00'},1,8)];  % 变体B(10时隙)
slot00 = [B, '66','01','00','00','00','80','00','F0', repmat({'00'},1,8)];  % 采集时隙00
slot01 = [B, '66','01','00','01','00','80','00','F0', repmat({'00'},1,8)];  % 采集时隙01
slot03 = [B, '66','01','00','03','00','80','00','F0', repmat({'00'},1,8)];  % 采集时隙03

combos = {
    'txA+slot00', txCmdA, slot00;
    'txA+slot01', txCmdA, slot01;
    'txA+slot03', txCmdA, slot03;
    'txB+slot00', txCmdB, slot00;
    'txB+slot01', txCmdB, slot01;
    'txB+slot03', txCmdB, slot03;
    };

fprintf('==== 命令组合扫描 (伪随机PN图案, 用新同步syncLTE判据) ====\n');
prm = GetLTEParams();
best = struct('name','','rho',0);
for k = 1:size(combos,1)
    name = combos{k,1};
    [rxBytes, ok] = udpLoopback(combos{k,2}, combos{k,3}, dataIQ, SAMPLE_LENGTH);
    if ~ok
        fprintf('%-12s 未收到数据\n', name);
        continue;
    end
    N = min(floor(numel(rxBytes)/4), SAMPLE_LENGTH);
    rb = double(rxBytes(1:N*4));
    % 布局1解包 (当前假设: I=b0b1, Q=b2b3)
    o = (0:N-1)*4;
    iw = rb(o+1) + rb(o+2)*256;
    qw = rb(o+3) + rb(o+4)*256;
    iv = floor(iw/16); qv = floor(qw/16);
    iv(iv>2047) = iv(iv>2047) - 4096;
    qv(qv>2047) = qv(qv>2047) - 4096;
    z = iv + 1j*qv;
    % 新同步rho: 频偏搜索+互相关定时 (抗频偏/定时, 旧算法会被频偏掩盖)
    [~, rhoSync] = syncLTE(z, zt, prm);
    % 旧判据(无频偏校正)保留作对照
    c = xcorr(z, zt);
    [cmax, ~] = max(abs(c));
    rho = cmax / (sqrt(sum(abs(z).^2)) * sqrt(sum(abs(zt).^2)));
    % 共轭备选
    c2 = xcorr(conj(z), zt);
    rhoC = max(abs(c2)) / (sqrt(sum(abs(z).^2)) * sqrt(sum(abs(zt).^2)));
    % 字节级相关
    tb = double(typecast(dataIQ, 'uint8'));
    tb = tb(1:N*4);
    [cb, ~] = max(abs(xcorr(rb-mean(rb), tb-mean(tb))));
    rhoB = cb / sqrt(sum((rb-mean(rb)).^2) * sum((tb-mean(tb)).^2));
    fprintf('%-12s rhoSync=%.4f (旧rho=%.4f rhoC=%.4f rhoB=%.4f std=%.2f)\n', ...
        name, rhoSync, rho, rhoC, rhoB, std(rb));
    if rhoSync > best.rho
        best = struct('name', name, 'rho', rhoSync);
    end
    save(sprintf('sweep_%s.mat', name), 'rxBytes');
end

fprintf('\n>> 最佳组合: %s (rho=%.4f)\n', best.name, best.rho);
if best.rho > 0.3
    fprintf('   该组合能使信号回来, 据此修改 OFDM_RFLoopback 中的命令即可。\n');
else
    fprintf(['   所有组合均未通 -> 射频路径本身有问题: 先做上位机 RF_Test 设备测试\n', ...
        '   (真实系统+衰减2dB, 测试灯是否亮); 检查天线/连线/设备是否初始化。\n']);
end

%% 记录结果到日志
logResult('verify_sweep', struct('bestCombo',best.name,'bestRho',best.rho));

%% 局部函数: 一次UDP收发
function [rxBytes, ok] = udpLoopback(txCmdHex, getCmdHex, dataIQ, SAMPLE_LENGTH)
pcip   = '192.168.1.180';
xsrpip = '192.168.1.166';
test_set_sync_clock = uint8(hex2dec({'00','00','99','bb','69','00','00','00','00','00','00','00','00','00','00','00','00','00','00','00'}));
test_Set_router     = uint8(hex2dec({'00','00','99','bb','68','00','00','00','00','06','00','00','00','00','00','00','00','00','00','00'}));
test_set_delay      = uint8(hex2dec({'00','00','99','bb','67','00','00','00','00','00','00','00','00','00','00','00','00','00','00','00'}));
test_Send_IQ        = uint8(hex2dec({'00','00','99','bb','64','00','00','00','00','00','00','00','00','00','00','00','00','00','00','00'}));
test_tx_command     = uint8(hex2dec(txCmdHex));
test_Get_IQ         = uint8(hex2dec(getCmdHex));

udp_obj = udp(xsrpip,13344,'LocalHost',pcip,'LocalPort',12345,'TimeOut',15, ...
    'OutputBufferSize',61440,'InputBufferSize',61440*10);
udp_obj.Timeout = 15;
ok = false; rxBytes = uint8([]);
try
    fopen(udp_obj);
    fwrite(udp_obj, test_set_sync_clock, 'uint8');
    fwrite(udp_obj, test_Set_router,  'uint8');
    fwrite(udp_obj, test_set_delay,   'uint8');
    fwrite(udp_obj, test_tx_command,  'uint8');
    fwrite(udp_obj, test_Send_IQ,     'uint8');
    SEND_PACKET_LENGTH = 512;
    for pn = 1:fix(SAMPLE_LENGTH*2/SEND_PACKET_LENGTH)
        fwrite(udp_obj, dataIQ(1,((pn-1)*SEND_PACKET_LENGTH+1):(pn*SEND_PACKET_LENGTH)), 'uint16');
    end
    fwrite(udp_obj, test_Get_IQ, 'uint8');

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
    fclose(udp_obj);
    if data_byte_c > 0
        rxBytes = uint8(buf(1:data_byte_c));
        ok = true;
    end
catch ME
    fprintf('    UDP错误: %s\n', ME.message);
end
try, fclose(udp_obj); catch, end
delete(udp_obj); clear udp_obj;
end
