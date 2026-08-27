function td_lte_visualizer()
% td_lte_visualizer - TD-LTE Tx/Rx 信号可视化界面
% 显示内容:
%   (1) Tx 时域 I/Q          (3) 频谱对比 (Tx / Rx原始, 相对dB)
%   (2) Rx 原始时域 I/Q      (4) 星座图: 理想点 + Rx均衡后符号
%                            (5) 处理信息(CRC/相关/定时/频偏)
% 运行模式:
%   仿真加噪 : txIQ + 高斯白噪声(SNR可调), 验证链路与噪声影响
%   真实环回 : 调用 OFDM_RFLoopback 走XSRP硬件, 自动做定时对齐+去频偏
%   加载数据 : 读取包含 txIQ/rxIQ 的 .mat (如 hw_capture.mat)
% 使用: 在MATLAB中运行 td_lte_visualizer

addpath(genpath(fileparts(mfilename('fullpath'))));   % 添加 Srcs/Tx/Rx 路径

Fs = 30.72e6;          % 采样率 30.72MHz

%% ==================== 界面 ====================
f = figure('Name','TD-LTE Tx/Rx 信号可视化','NumberTitle','off','MenuBar','none', ...
    'Position',[80 60 1500 880], 'Color',[0.94 0.94 0.94]);

% --- 顶部控制条 ---
uicontrol(f,'Style','text','String','运行模式:','Position',[15 845 62 20],'HorizontalAlignment','right');
popMode = uicontrol(f,'Style','popupmenu','String',{'仿真(加噪)','真实环回','加载数据(.mat)'},'Position',[80 843 130 24]);

uicontrol(f,'Style','text','String','调制:','Position',[225 845 40 20],'HorizontalAlignment','right');
popMod  = uicontrol(f,'Style','popupmenu','String',{'QPSK','16QAM','64QAM'},'Position',[268 843 80 24]);

uicontrol(f,'Style','text','String','SNR(dB):','Position',[365 845 55 20],'HorizontalAlignment','right');
sldSNR  = uicontrol(f,'Style','slider','Min',0,'Max',40,'Value',25,'Position',[425 845 140 20]);
txtSNR  = uicontrol(f,'Style','text','String','25 dB','Position',[570 845 50 20]);
set(sldSNR,'Callback',@(s,e) set(txtSNR,'String',sprintf('%.0f dB',get(sldSNR,'Value'))));

btnRun  = uicontrol(f,'Style','pushbutton','String','运行','Position',[640 843 70 28]);

uicontrol(f,'Style','text','String','状态:','Position',[730 845 40 20],'HorizontalAlignment','right');
txtStatus = uicontrol(f,'Style','text','String','就绪 - 选择模式后点"运行"', ...
    'Position',[775 838 700 40],'HorizontalAlignment','left');

% --- 5 个绘图区 ---
axTx    = subplot(2,3,1,'Parent',f);  title(axTx,'Tx 时域 I/Q');
axRxRaw = subplot(2,3,2,'Parent',f);  title(axRxRaw,'Rx 原始时域 I/Q');
axSpec  = subplot(2,3,3,'Parent',f);  title(axSpec,'频谱对比 (相对dB)');
axCon   = subplot(2,3,4,'Parent',f);  title(axCon,'星座图: 理想(x) vs Rx均衡后(.)');
axInfo  = subplot(2,3,5,'Parent',f);  title(axInfo,'处理信息');
axis(axInfo,'off');
grid(axTx,'on'); grid(axRxRaw,'on'); grid(axSpec,'on'); grid(axCon,'on');

set(btnRun,'Callback',@(s,e) runAll());
set(f,'HandleVisibility','on');

%% ==================== 运行主流程 ====================
    function runAll()
        mode = get(popMode,'Value');
        modStr = get(popMod,'String'); modStr = modStr{get(popMod,'Value')};
        snr = get(sldSNR,'Value');

        % 参数
        prm = GetLTEParams();
        prm.Modulation = modStr;
        switch modStr
            case 'QPSK',  prm.Qm = 2;  prm.TBsize = 15840;
            case '16QAM', prm.Qm = 4;  prm.TBsize = 28336;
            case '64QAM', prm.Qm = 6;  prm.TBsize = 55056;
        end

        rng(42);
        txIQ = OFDM_TxMain(prm);
        txIQ = txIQ(:).';

        info = struct('mode',modStr,'snr',snr,'crc',[],'rho',[],'off',[],'cfo',[],'ber',[]);

        %% 获取 rxIQ
        switch mode
            case 1  % 仿真加噪
                N = numel(txIQ);
                Ptx = mean(abs(txIQ).^2);
                Pn  = Ptx / 10^(snr/10);
                nz  = sqrt(Pn/2) * (randn(1,N) + 1j*randn(1,N));
                rxIQ = txIQ + nz;
                info.mode = sprintf('仿真加噪 (SNR=%.0f dB)', snr);
                % 参考比特用于BER
                rng(42); [txBits,~] = TBGen(prm);
                info.ber = 1;   % 占位, 下面覆写

            case 2  % 真实环回
                rxIQ = OFDM_RFLoopback(txIQ, '192.168.1.180', '192.168.1.166');
                rxIQ = rxIQ(:).';
                info.mode = '真实环回 (XSRP)';

            case 3  % 加载数据
                [fn, pth] = uigetfile({'*.mat','MAT文件'},'选择含 txIQ/rxIQ 的数据');
                if isequal(fn,0), set(txtStatus,'String','已取消加载'); return; end
                S = load(fullfile(pth,fn));
                if ~isfield(S,'rxIQ'), set(txtStatus,'String','文件中没有 rxIQ 变量'); return; end
                rxIQ = S.rxIQ(:).';
                if isfield(S,'txIQ'), txIQ = S.txIQ(:).'; end
                info.mode = sprintf('加载 %s', fn);
        end

        %% 绘制时域 (每4点抽1显示)
        t = (0:numel(txIQ)-1)/Fs * 1e6;   % us
        dec = 1:4:numel(txIQ);
        plot(axTx, t(dec), real(txIQ(dec)), 'b', t(dec), imag(txIQ(dec)), 'r');
        xlabel(axTx,'us'); legend(axTx,{'I','Q'},'Location','best');

        plot(axRxRaw, t(dec), real(rxIQ(dec)), 'b', t(dec), imag(rxIQ(dec)), 'r');
        xlabel(axRxRaw,'us'); legend(axRxRaw,{'I','Q'},'Location','best');

        %% 频谱对比 (各自归一化到峰值0dB, 消除TX/RX幅度尺度差异)
        [fA, mA] = specMag(txIQ,  Fs);
        [fB, mB] = specMag(rxIQ,  Fs);
        mA = mA - max(mA);
        mB = mB - max(mB);
        plot(axSpec, fA/1e6, mA, 'g', fB/1e6, mB, 'b');
        xlabel(axSpec,'MHz'); ylabel(axSpec,'相对dB');
        legend(axSpec,{'Tx','Rx原始'},'Location','best');
        xlim(axSpec,[-15.36 15.36]); ylim(axSpec,[-70 5]);

        %% 真实/加载模式: 稳健同步(syncLTE: 频偏搜索+互相关) + 削波检测
        procIQ = rxIQ;
        if mode == 2 || mode == 3
            clipWarn = checkClip(rxIQ);
            if ~isempty(clipWarn), info.clip = clipWarn; end
            try
                [rxAl, rho, cfoHz, delay] = syncLTE(rxIQ, txIQ, prm);
                info.off = delay; info.cfo = cfoHz; info.rho = rho;
                procIQ = rxAl;                 % 星座图/CRC用对齐+去频偏信号
            catch ME
                info.err = ME.message;
            end
        end

        %% 星座图 (Rx均衡后)
        ptsIdeal = idealConstellation(modStr);
        [dataSyms, crcOK] = rxConstellation(procIQ, prm);
        plot(axCon, real(ptsIdeal), imag(ptsIdeal), 'rx', ...
            real(dataSyms(1:min(end,8000))), imag(dataSyms(1:min(end,8000))), 'b.');
        xlabel(axCon,'I'); ylabel(axCon,'Q');
        legend(axCon,{'理想','Rx均衡后'},'Location','best');
        info.crc = crcOK;

        %% 仿真模式补BER
        if mode == 1
            try
                [rxBits, ~] = OFDM_RxMain(rxIQ, prm);
                b = min(numel(txBits), numel(rxBits));
                info.ber = mean(txBits(1:b) ~= rxBits(1:b));
            catch
                info.ber = NaN;
            end
        end

        %% 状态文本
        S = sprintf('模式: %s\n调制: %s  TBsize=%d', info.mode, modStr, prm.TBsize);
        if mode == 1
            S = sprintf('%s\nSNR=%.0f dB | CRC=%d | BER=%.4g', S, snr, info.crc, info.ber);
        else
            S = sprintf('%s\nCRC(稳健同步+MMSE)=%d', S, info.crc);
            if isfield(info,'rho'), S = sprintf('%s | 互相关rho=%.3f', S, info.rho); end
            if isfield(info,'off'), S = sprintf('%s\n帧偏移=%d样点 | 频偏=%.1f Hz', S, info.off, info.cfo); end
            if isfield(info,'err'), S = sprintf('%s\n同步出错: %s', S, info.err); end
            if isfield(info,'clip'), S = sprintf('%s\n⚠ %s', S, info.clip); end
        end
        set(txtStatus,'String',S);

        % 记录结果到日志
        if isfield(info,'rho'), rhoL = info.rho; else, rhoL = NaN; end
        if isfield(info,'ber'), berL = info.ber; else, berL = NaN; end
        if isfield(info,'off'), offL = info.off; else, offL = NaN; end
        if isfield(info,'cfo'), cfoL = info.cfo; else, cfoL = NaN; end
        logResult(sprintf('visualizer_%s', modStr), struct('crc',info.crc,'rho',rhoL, ...
            'ber',berL,'off',offL,'cfo',cfoL,'mode',info.mode));

        % 保存当前结果便于复查
        save('viz_result.mat','txIQ','rxIQ','dataSyms','info');
    end
end

%% ==================== 工具函数 ====================
function [f, m] = specMag(x, Fs)
N = numel(x);
X = fft(x(:).', N);
f = (0:N-1)/N*Fs; f(f>Fs/2) = f(f>Fs/2) - Fs;
m = 20*log10(abs(X) + eps);
end

function pts = idealConstellation(mode)
switch mode
    case 'QPSK',  mi = [1 -1];            mq = [1 -1];            s = sqrt(2);
    case '16QAM', mi = [1 3 -1 -3];       mq = [1 3 -1 -3];       s = sqrt(10);
    case '64QAM', mi = [3 1 5 7 -3 -1 -5 -7]; mq = mi;            s = sqrt(42);
    otherwise,    error('未知调制 %s', mode);
end
pts = (mi(:) + 1j*mq(:).') / s;
pts = pts(:);
end

function [dataSyms, crcOK] = rxConstellation(rxIQ, prm)
% 走收端链路(与OFDM_RxMain一致): 去CP+FFT -> LS信道估计+频域平滑 -> MMSE均衡 -> 提取数据符号
rxGrid = OFDMDemod(rxIQ, prm);
[rs0, rs1] = RefSignalGen(prm);
H = ChanEst(rxGrid, rs0, rs1, prm);
% 频域平滑 (与 OFDM_RxMain 一致, 降低LS估计噪声)
notchMask = false(prm.NSubc, 1);
for l = 1:prm.NSym
    h = H(:, l); h(notchMask) = NaN;
    h = movmean(h, prm.chanSmooth, 'omitnan');
    h(notchMask) = H(notchMask, l);
    H(:, l) = h;
end
pilotIdx1 = prm.PilotSymbols(1) + 1; pilotIdx2 = prm.PilotSymbols(2) + 1;
residual = [abs(rxGrid(:,pilotIdx1) - H(:,pilotIdx1).*rs0).^2; ...
            abs(rxGrid(:,pilotIdx2) - H(:,pilotIdx2).*rs1).^2];
noiseVar = max(median(residual), 1e-9);
eqGrid = Equalize(rxGrid, H, 'MMSE', noiseVar);
dataSyms = ExtractData(eqGrid, prm);
try
    [~, crcOK] = OFDM_RxMain(rxIQ, prm);
catch
    crcOK = false;
end
end

function warn = checkClip(rxIQ)
% checkClip - 削波/PAPR检测: 满幅或低PAPR说明接收增益过高(ADC饱和), 提示降增益/幅度
warn = '';
r = abs(rxIQ(:));
rmsv = sqrt(mean(r.^2));
papr = max(r) / max(rmsv, 1e-12);
clipFrac = mean(r > 0.9);
if papr < 1.6 || clipFrac > 0.5
    warn = sprintf('RX削波! PAPR=%.2f(OFDM正常~3), |x|>0.9占%.0f%% -> 降低RX增益/发送幅度', papr, 100*clipFrac);
end
end
