%% evm_amp_sweep.m - 验证"互调假说": EVM随发送幅度变化吗?
% 背景: 单音52dB但OFDM每子载波仅-1.77dB -> 疑似接收路径非线性产生互调,
%       互调只毁多载波不毁单音。互调功率∝信号^3, 信号∝信号^1,
%       故降低发送幅度应显著改善EVM(每降10dB信号, EVM提升~20dB)。
% 方法: 固定上位机设置, 发送峰值扫 2047/1000/400, 每次K=16帧平均后测EVM。
% 判定: EVM随幅度下降明显改善 -> 互调确认, 找到最优工作点, 有救!
%       EVM基本不变             -> 非互调, 缺口是设备固定底噪, 收尾。
% 注意: 每次约2~3分钟, 共约8分钟。勿碰天线。

clear; clc;
addpath(fileparts(mfilename('fullpath')));
addpath(fullfile(fileparts(mfilename('fullpath')), 'Tx'));
addpath(fullfile(fileparts(mfilename('fullpath')), 'Rx'));

SAMPLE_LENGTH = 30720;
B = {'00','00','99','bb'};
cmdSync   = uint8(hex2dec({'00','00','99','bb','69','00','00','00','00','00','00','00','00','00','00','00','00','00','00','00'}));
cmdRouter = uint8(hex2dec({'00','00','99','bb','68','00','00','00','00','06','00','00','00','00','00','00','00','00','00','00'}));
cmdDelay  = uint8(hex2dec({'00','00','99','bb','67','00','00','00','00','00','00','00','00','00','00','00','00','00','00','00'}));
cmdTx     = uint8(hex2dec([B, '65','02','00','03','00','01','78','00', repmat({'00'},1,8)]));
cmdSend   = uint8(hex2dec({'00','00','99','bb','64','00','00','00','00','00','00','00','00','00','00','00','00','00','00','00'}));
cmdGet    = uint8(hex2dec([B, '66','01','00','01','00','80','00','F0', repmat({'00'},1,8)]));
pcip = '192.168.1.180'; xsrpip = '192.168.1.166';
prm = GetLTEParams();

% 发送参考OFDM
rng(42);
txIQ = OFDM_TxMain(prm); txIQ = txIQ(:).';

fprintf('==== EVM vs 发送幅度 (验证互调假说, 勿碰天线) ====\n');
amps = [2047 1000 400];
K = 16;
for ai = 1:numel(amps)
    amp = amps(ai);
    % 采集K帧并相干平均
    frames = zeros(1, SAMPLE_LENGTH, K);
    okc = 0;
    for k = 1:K
        rxIQ = udpLoopback(txIQ, amp, cmdSync, cmdRouter, cmdDelay, cmdTx, cmdSend, cmdGet, SAMPLE_LENGTH, pcip, xsrpip);
        if isempty(rxIQ), continue; end
        [rxAl, ~, ~, ~, pha] = syncLTE(rxIQ(:).', txIQ, prm);
        frames(1,:,k) = rxAl .* exp(-1j*pha);
        okc = okc + 1;
    end
    if okc == 0, fprintf('峰值%d: 未收到\n', amp); continue; end
    rxAvg = mean(frames, 3);
    snr = calcEVM(rxAvg, txIQ, prm);
    fprintf('发送峰值%4d: 均衡后数据SNR = %.2f dB\n', amp, snr);
    logResult('evm_amp_sweep', struct('amp',amp,'snr',snr));
end
fprintf('\n>> EVM随幅度下降而改善 -> 互调假说成立, 最优幅度处CRC有望;\n');
fprintf('   若不变 -> 设备固定底噪, 确认设备侧限制。\n');

%% ================= 函数 =================
function snr = calcEVM(rxAvg, txIQ, prm)
% 发送参考数据符号
rng(42);
[bits, info] = TBGen(prm);
bitsCRC = CRCAttach(bits, '24A');
[cbs, segInfo] = CodeBlockSeg(bitsCRC);
cbOutputs = cell(segInfo.C, 1);
for r = 1:segInfo.C
    coded = TurboEncode(cbs{r}, segInfo.K(r));
    cbOutputs{r} = RateMatch(coded, segInfo.K(r), segInfo.C, info.G, info.Qm, prm.NL, prm.RV);
end
codedBits = CodeBlockCat(cbOutputs);
scrambled = Scramble(codedBits, prm);
sym = Modulate(scrambled, prm.Modulation);
[rs0, rs1] = RefSignalGen(prm);
txGrid = ResMap(sym, rs0, rs1, prm);
txDataSyms = ExtractData(txGrid, prm);

% 接收处理 (与OFDM_RxMain一致: 平滑+污染隔离+MMSE)
rxGrid = OFDMDemod(rxAvg, prm);
H = ChanEst(rxGrid, rs0, rs1, prm);
notchMask = false(prm.NSubc,1);
if isfield(prm,'chanSmooth') && prm.chanSmooth > 1
    for l = 1:prm.NSym
        h = H(:,l); h(notchMask) = NaN;
        h = movmean(h, prm.chanSmooth, 'omitnan');
        h(notchMask) = H(notchMask,l);
        H(:,l) = h;
    end
end
pilotIdx1 = prm.PilotSymbols(1)+1; pilotIdx2 = prm.PilotSymbols(2)+1;
residual = [abs(rxGrid(:,pilotIdx1)-H(:,pilotIdx1).*rs0).^2; ...
            abs(rxGrid(:,pilotIdx2)-H(:,pilotIdx2).*rs1).^2];
noiseVar = max(median(residual), 1e-9);
eqGrid = Equalize(rxGrid, H, 'MMSE', noiseVar);
dataSyms = ExtractData(eqGrid, prm);
err = dataSyms - txDataSyms;
snr = 10*log10(mean(abs(txDataSyms).^2) / mean(abs(err).^2));
end

function rxIQ = udpLoopback(txIQ, amp, cmdSync, cmdRouter, cmdDelay, cmdTx, cmdSend, cmdGet, SAMPLE_LENGTH, pcip, xsrpip)
rxIQ = [];
txIQ = txIQ(:).';
dataIQ = zeros(1, SAMPLE_LENGTH*2);
dataIQ(1,1:2:end-1) = real(txIQ); dataIQ(1,2:2:end) = imag(txIQ);
dataIQ = dataIQ .* (amp/max(abs(dataIQ)));
dataIQ = fix(dataIQ);
for n = 1:numel(dataIQ)
    if dataIQ(n) > 2047, dataIQ(n) = 2047;
    elseif dataIQ(n) < 0, dataIQ(n) = 4096 + dataIQ(n); end
end
dataIQ(1,1:2:end-1) = dataIQ(1,1:2:end-1) .* 16;
dataIQ(1,2:2:end) = fix(dataIQ(1,2:2:end)./256) + rem(dataIQ(1,2:2:end),256).*256;
dataIQ = uint16(dataIQ);

udp_obj = udp(xsrpip,13344,'LocalHost',pcip,'LocalPort',12345,'TimeOut',15, ...
    'OutputBufferSize',61440,'InputBufferSize',61440*10);
udp_obj.Timeout = 15;
try
    fopen(udp_obj);
    fwrite(udp_obj, cmdSync,   'uint8');
    fwrite(udp_obj, cmdRouter, 'uint8');
    fwrite(udp_obj, cmdDelay,  'uint8');
    fwrite(udp_obj, cmdTx,     'uint8');
    fwrite(udp_obj, cmdSend,   'uint8');
    SEND_PACKET_LENGTH = 512;
    for pn = 1:fix(SAMPLE_LENGTH*2/SEND_PACKET_LENGTH)
        fwrite(udp_obj, dataIQ(1,((pn-1)*SEND_PACKET_LENGTH+1):(pn*SEND_PACKET_LENGTH)), 'uint16');
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
        N = floor(data_byte_c/4);
        b = double(buf(1:N*4));
        o = (0:N-1)*4;
        iw = b(o+1) + b(o+2)*256;
        qw = b(o+3) + b(o+4)*256;
        iv = floor(iw/16);  qv = floor(qw/16);
        iv(iv>2047) = iv(iv>2047) - 4096;
        qv(qv>2047) = qv(qv>2047) - 4096;
        rxIQ = iv + 1j*qv;
    end
catch
end
try, fclose(udp_obj); catch, end
delete(udp_obj); clear udp_obj;
end
