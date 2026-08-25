function [rxdataIQ] = OFDM_RFLoopback(txdataIQ,pcip,xsrpip,txPeak,paprClip,rxCh)
% txPeak   - 可选: 发送峰值幅度(12位满幅2047), 默认2047。
%           若接收端饱和/削波, 可降低峰值(如800)重试。
% paprClip - 可选: PAPR削减倍数(如1.5), 默认0=不削减。
%           OFDM高峰均比导致"峰值饱和/平均被埋", 削减峰值到paprClip*RMS
%           可提升平均功率2~3倍而不增峰值。接收端均衡可容忍其轻微失真。
% rxCh     - 可选: 接收通道 1=RX1, 2=RX2 (默认1)。
%           修改Get_IQ命令第2字节('01'/'02')。注意: 该字节含义为推测
%           (原注释称"采集时隙号"), 若改2后回传变静音请改回1。
% 注意: TX通道由上位机"发射通道(TxCh1/TxCh2)"设置, 本代码无TX通道字节;
%       使用TX2/RX2需同时在上位机设置 发射通道=TxCh2、接收通道=RxCh2,
%       并把天线接到 TX2/RX2 接口。

%%对数据长度进行补零或溢出删除
SAMPLE_LENGTH = 30720;                  % 样点数30.72Msps * 1ms
if nargin < 4 || isempty(txPeak), txPeak = 2047; end
if nargin < 5 || isempty(paprClip), paprClip = 0; end
if nargin < 6 || isempty(rxCh), rxCh = 1; end
TxdataI=zeros(1,SAMPLE_LENGTH);
TxdataQ=zeros(1,SAMPLE_LENGTH);
if length(txdataIQ)>=SAMPLE_LENGTH
    TxdataI=real(txdataIQ(1,1:SAMPLE_LENGTH));
    TxdataQ=imag(txdataIQ(1,1:SAMPLE_LENGTH));
else
    TxdataI(1,1:length(txdataIQ))=real(txdataIQ);
    TxdataQ(1,1:length(txdataIQ))=imag(txdataIQ);
end

%% PAPR削减: 把超过 paprClip*RMS 的峰值压回限制值
if paprClip > 0
    s = TxdataI + 1j*TxdataQ;
    rms = sqrt(mean(abs(s).^2));
    lim = paprClip * rms;
    over = abs(s) > lim;
    if any(over)
        s(over) = lim * s(over) ./ abs(s(over));
    end
    TxdataI = real(s);
    TxdataQ = imag(s);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%发送数据处理
%放大峰值，浮点取整
len = SAMPLE_LENGTH*2;
dataIQ = zeros(1,len);
dataIQ(1,1:2:len-1) = TxdataI(1,:);
dataIQ(1,2:2:len) = TxdataQ(1,:);
dataIQ = dataIQ.*(txPeak/max(dataIQ));    %放大峰值至txPeak(默认2047, 防饱和可调小)
dataIQ = fix(dataIQ);                   %浮点数强制取整

%防止溢出，并对负数进行补码操作
for n = 1 : len
    if dataIQ(n) > 2047
        dataIQ(n) = 2047;
    elseif  dataIQ(n) < 0
        dataIQ(n) = 4096 + dataIQ(n);
    end
end

%按接口定义排列比特序
dataIQ(1,1:2:len-1) = dataIQ(1,1:2:len).*16;    %I路：b11~b0 空4bits
dataIQ(1,2:2:len) = fix(dataIQ(1,2:2:len)./256) + rem(dataIQ(1,2:2:len),256).*256;  %Q路：b7~b0 空4bits b11~b8
dataIQ = uint16(dataIQ);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%定义配置参数常量
test_set_sync_clock = uint8(hex2dec({'00','00','99','bb',  '69','00','00','00',  '00','00','00','00',  '00','00','00','00',  '00','00','00','00'}));%OK
test_Set_router = uint8(hex2dec({'00','00','99','bb', '68','00','00','00',  '00','06','00','00',  '00','00','00','00',  '00','00','00','00'}));%网口环回%OK
test_set_delay_system = uint8(hex2dec({'00','00','99','bb', '67','00','00','00',  '00','00','00','00',  '00','00','00','00',  '00','00','00','00'}));
test_tx_command = uint8(hex2dec({'00','00','99','bb', '65','02','00','03',  '00','01','78','00',  '00','00','00','00',  '00','00','00','00'}));%0A  10个时隙，03FF 时隙开关000001111111111，0000 分频，7800 数据个数/时隙30720%OK
%test_tx_command = uint8(hex2dec({'00','00','99','bb', '65','0A','03','FF',  '00','03','78','00',  '00','00','00','00',  '00','00','00','00'}));%0A  10个时隙，03FF 时隙开关000001111111111，0000 分频，7800 数据个数/时隙30720
test_Send_IQ = uint8(hex2dec({'00','00','99','bb', '64','00','00','00',  '00','00','00','00',  '00','00','00','00',  '00','00','00','00'}));
% Get_IQ: '66' [RX通道: 01=RX1, 02=RX2(推测, 原注释称采集时隙号)] '00' [时隙] '01' '00' '80' '00' 'F0' ...
% 注意: 若把该字节改为02后回传变成静音, 说明该字节含义不是通道, 请改回'01'。
rxChStr = '01';                                   % 接收通道: '01'=RX1, '02'=RX2
if rxCh == 2, rxChStr = '02'; end
test_Get_IQ  = uint8(hex2dec({'00','00','99','bb', '66', rxChStr, '00','01',  '00','80','00','F0',  '00','00','00','00',  '00','00','00','00'}));%01 采集时隙号，0000 分频，0060 包的数量96，00F0 包的大小240（实际接收字节数为配置值乘以4）%OK
%test_Get_IQ  = uint8(hex2dec({'00','00','99','bb', '66','01','00','03',  '00','60','00','F0',  '00','00','00','00',  '00','00','00','00'}));%01 采集时隙号，0000 分频，0060 包的数量96，00F0 包的大小240（实际接收字节数为配置值乘以4）

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%创建UDP对象，并打开
% 先释放可能残留的MATLAB UDP对象(防止"Address already in use"端口占用)
try
    objs = instrfind('Type', 'udp');
    for k = 1:numel(objs)
        if isvalid(objs(k)) && strcmp(get(objs(k), 'Status'), 'open')
            fclose(objs(k));
        end
        delete(objs(k));
    end
catch
end
udp_obj = udp(xsrpip,13344,'LocalHost',pcip,'LocalPort',12345,'TimeOut',15,'OutputBufferSize',61440,'InputBufferSize',61440*10); %#ok<UDPP>
udp_obj.Timeout=15;
fopen(udp_obj);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%发送配置命令
fwrite(udp_obj, test_set_sync_clock, 'uint8');
fwrite(udp_obj, test_Set_router,  'uint8');
fwrite(udp_obj, test_set_delay_system, 'uint8');
fwrite(udp_obj, test_tx_command, 'uint8');
fwrite(udp_obj, test_Send_IQ, 'uint8');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%发送已调数据
SEND_PACKET_LENGTH = 512;   %发送UDP包长度，数据为U16，所以每个UDP包发送字节数数为512*2，
for pn = 1:fix(SAMPLE_LENGTH*2/SEND_PACKET_LENGTH)
    fwrite(udp_obj, dataIQ(1,((pn-1)*SEND_PACKET_LENGTH+1) : (pn*SEND_PACKET_LENGTH)), 'uint16');
end
%fwrite(udp_obj, dataIQ(1,(pn*SEND_PACKET_LENGTH+1 ): (SAMPLE_LENGTH*2)),'uint16');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%发送采集（接收）数据启动命令
fwrite(udp_obj, test_Get_IQ, 'uint8');

%% 采集变量初始化
RECV_PACKET_LENGTH = 240*4;                         %接收包字节数。大小为配置值乘以4。
data_byte_numb = SAMPLE_LENGTH*4;                   %接收数据字节数，0分频1个IQ样点对应3个字节
udp_data_ri = zeros(1, SAMPLE_LENGTH); %#ok<NASGU>
udp_data_rq = zeros(1, SAMPLE_LENGTH); %#ok<NASGU>
udp_data_rr0 = zeros(1, SAMPLE_LENGTH*4);
rxdataI = zeros(1,SAMPLE_LENGTH); %#ok<PREALL>
rxdataQ = zeros(1,SAMPLE_LENGTH); %#ok<PREALL>

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%开始接收数据
recvInd = 1;
data_byte_c = 0;
while recvInd == 1
    % 从网口读数据
    [udp_data,count] = fread(udp_obj,RECV_PACKET_LENGTH);
    %接收数据拼接
    for pj = 1 : count
        udp_data_rr0(data_byte_c+pj) = udp_data(pj);
    end
    data_byte_c = data_byte_c + count;
    %判断接收总字节数
    if ((data_byte_c>=data_byte_numb) || (data_byte_c==0))        %如果数据接收完成或超时没有接收到数据则退出
        recvInd = 0;    % 停止运行
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%关闭UDP对象
fclose(udp_obj);
delete(udp_obj);
clear udp_obj;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%接收数据处理
freqDiv=1;
[rxdataI,rxdataQ]=OFDM_RxDataDeal(udp_data_rr0,freqDiv,data_byte_numb);

rxdataIQ=rxdataI+rxdataQ*1i;
rxdataIQ=rxdataIQ/2047;
%plot(rxdataI)
end
