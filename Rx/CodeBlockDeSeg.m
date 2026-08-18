function [bits, crcOK] = CodeBlockDeSeg(cbDecoded, info)
% CodeBlockDeSeg - 码块去分段 + TB-CRC校验
% 输入:
%   cbDecoded - cell数组, 每个元素是译码后的码块bit
%   info      - CodeBlockSeg返回的info结构体
% 输出:
%   bits  - 恢复的传输块bit列向量
%   crcOK - 全局CRC校验是否通过 (logical)

end
