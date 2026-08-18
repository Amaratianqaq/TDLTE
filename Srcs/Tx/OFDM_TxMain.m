function txIQ = OFDM_TxMain(prm)
% OFDM_TxMain - 发送端总入口, 串联全部发送链路模块
% 处理链路 (按调用顺序):
%   TBGen(信源) -> CRCAttach(加CRC) -> CodeBlockSeg(码块分割)
%   -> TurboEncode(Turbo编码) -> RateMatch(速率匹配, 含子块交织)
%   -> CodeBlockCat(码块级联) -> Scramble(加扰) -> Modulate(调制)
%   -> RefSignalGen(生成导频) -> ResMap(资源映射) -> OFDMMod(OFDM调制)
%   -> txIQ
% 输入:
%   prm - 参数结构体 (由 GetLTEParams 返回)
% 输出:
%   txIQ - 时域复数IQ信号 (1ms子帧, 30720样点)

end
