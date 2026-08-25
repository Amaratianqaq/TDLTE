%% scan_config.m - 黑盒扫描: router值 x 接收通道 x 数据包格式
% 背景: 控制组实验证明"回传与发射内容无关"(rho≈0.2假相关来自OFDM频谱形状),
%       发射/采集路径未通。在缺少设备协议文档的情况下, 系统化扫描:
%   变量1: router命令第6字节 0x00~0x0F (模式/路由选择)
%   变量2: Get_IQ第2字节 01/02 (接收通道或时隙)
%   变量3: 数据包格式 (当前uint16格式 / 每包前加帧头)
% 判据: 伪随机图案与设备固定内容的假相关仅~0.02,
%       任何组合使 rho>0.3 即视为"发射数据到达了回传路径"。
% 运行: 硬件在线, 直接运行, 约5~8分钟 (72组)。

clear; clc;
addpath(fileparts(mfilename('fullpath')));

SAMPLE_LENGTH = 30720;

%% 1) 伪随机测试图案与打包 (同 verify_udp_format)
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

B = {'00','00','99','bb'};
cmdSync   = uint8(hex2dec({'00','00','99','bb','69','00','00','00','00','00','00','00','00','00','00','00','00','00','00','00'}));
cmdDelay  = uint8(hex2dec({'00','00','99','bb','67','00','00','00','00','00','00','00','00','00','00','00','00','00','00','00'}));
cmdTx     = uint8(hex2dec([B, '65','02','00','03','00','01','78','00', repmat({'00'},1,8)]));
cmdSend   = uint8(hex2dec({'00','00','99','bb','64','00','00','00','00','00','00','00','00','00','00','00','00','00','00','00'}));

fprintf('==== 黑盒配置扫描 (伪随机图案, 阈值rho>0.3) ====\n');
best = struct('name','','rho',0);
results = {};
for router = 0:15
    cmdRouter = uint8(hex2dec([B, '68','00','00','00', sprintf('%02X',router), repmat({'00'},1,11)]));
    for ch = 1:2
        for pktMode = 1:2
            name = sprintf('router=%02X ch=%d pkt=%s', router, ch, ...
                ternary(pktMode==1,'普通','加帧头'));
            % 组装数据包
            if pktMode == 1
                dataOut = dataIQ;
                pktLen = 512;                       % 每包512个uint16
            else
                % 每包前加16字节帧头(8个uint16), 包长=8+512
                hdr = uint16([0 0 0x99bb 0x6400 0 0 0 0]);  % 8个uint16帧头
                dataOut = [];
                for p = 1:fix(len/512)
                    dataOut = [dataOut, hdr, dataIQ((p-1)*512+1 : p*512)]; %#ok<AGROW>
                end
                pktLen = 520;
            end
            cmdGet = uint8(hex2dec([B, '66', sprintf('%02X',ch), '00','01','00','80','00','F0', repmat({'00'},1,8)]));
            [rxBytes, ok] = udpLoopback(cmdRouter, cmdGet, dataOut, SAMPLE_LENGTH, pktLen);
            if ~ok
                fprintf('%-28s 未收到数据\n', name);
                continue;
            end
            N = min(floor(numel(rxBytes)/4), SAMPLE_LENGTH);
            rb = double(rxBytes(1:N*4));
            o = (0:N-1)*4;
            iw = rb(o+1) + rb(o+2)*256;
            qw = rb(o+3) + rb(o+4)*256;
            iv = floor(iw/16); qv = floor(qw/16);
            iv(iv>2047) = iv(iv>2047) - 4096;
            qv(qv>2047) = qv(qv>2047) - 4096;
            z = iv + 1j*qv;
            c = xcorr(z - mean(z), zt - mean(zt));
            rho = max(abs(c)) / (sqrt(sum(abs(z-mean(z)).^2)) * sqrt(sum(abs(zt-mean(zt)).^2)));
            fprintf('%-28s rho=%.4f\n', name, rho);
            results{end+1} = struct('name',name,'rho',rho);
            if rho > best.rho
                best = struct('name',name,'rho',rho);
            end
            if rho > 0.3
                fprintf('   >> 命中! 该组合可使信号回来\n');
            end
        end
    end
end

fprintf('\n>> 最佳: %s (rho=%.4f)\n', best.name, best.rho);
if best.rho > 0.3
    fprintf('   找到有效配置! 按此修改 OFDM_RFLoopback 命令即可。\n');
else
    fprintf(['   全部组合均未通 -> 发射路径(DA/射频)或采集路径(ADC/时隙)在设备侧未生效。\n', ...
        '   建议: 1)确认上位机 发射通道(TxCh1/TxCh2) 与天线接口一致;\n', ...
        '   2)确认上位机 运行方式=真实系统、TX/RX频率一致;\n', ...
        '   3)联系设备厂商/指导老师获取 UDP 命令协议文档。\n']);
end
save('scan_config_result.mat', 'results', 'best');

%% 局部函数: 一次UDP收发
function [rxBytes, ok] = udpLoopback(cmdRouter, cmdGet, dataOut, SAMPLE_LENGTH, pktLen)
pcip   = '192.168.1.180';
xsrpip = '192.168.1.166';
cmdSync   = uint8(hex2dec({'00','00','99','bb','69','00','00','00','00','00','00','00','00','00','00','00','00','00','00','00'}));
cmdDelay  = uint8(hex2dec({'00','00','99','bb','67','00','00','00','00','00','00','00','00','00','00','00','00','00','00','00'}));
cmdTx     = uint8(hex2dec({'00','00','99','bb','65','02','00','03','00','01','78','00','00','00','00','00','00','00','00','00'}));
cmdSend   = uint8(hex2dec({'00','00','99','bb','64','00','00','00','00','00','00','00','00','00','00','00','00','00','00','00'}));

udp_obj = udp(xsrpip,13344,'LocalHost',pcip,'LocalPort',12345,'TimeOut',10, ...
    'OutputBufferSize',61440*2,'InputBufferSize',61440*10);
udp_obj.Timeout = 10;
ok = false; rxBytes = uint8([]);
try
    fopen(udp_obj);
    fwrite(udp_obj, cmdSync,   'uint8');
    fwrite(udp_obj, cmdRouter, 'uint8');
    fwrite(udp_obj, cmdDelay,  'uint8');
    fwrite(udp_obj, cmdTx,     'uint8');
    fwrite(udp_obj, cmdSend,   'uint8');
    for pn = 1:fix(numel(dataOut)/pktLen)
        fwrite(udp_obj, dataOut(1,((pn-1)*pktLen+1):(pn*pktLen)), 'uint16');
    end
    fwrite(udp_obj, cmdGet, 'uint8');
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
try fclose(udp_obj); catch, end
delete(udp_obj); clear udp_obj;
end

function r = ternary(c, a, b)
if c, r = a; else, r = b; end
end
