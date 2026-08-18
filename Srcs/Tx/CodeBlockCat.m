function out = CodeBlockCat(cbOutputs)
% CodeBlockCat - 码块级联, 按顺序拼接各码块速率匹配输出 (3GPP 36.212 5.1.5)
% 输入:
%   cbOutputs - cell数组, 每个元素是一个码块的速率匹配输出 (列向量)
% 输出:
%   out - 拼接后的bit列向量, 长度 G = sum(E_r)

out = [];
for r = 1:numel(cbOutputs)
    out = [out; cbOutputs{r}(:)];
end
end
