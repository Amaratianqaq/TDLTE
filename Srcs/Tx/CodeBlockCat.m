function out = CodeBlockCat(cbOutputs)
% CodeBlockCat - 码块级联, 按顺序拼接各码块速率匹配输出 (3GPP 36.212 5.1.5)
% 输入:
%   cbOutputs - cell数组, 每个元素是一个码块的速率匹配输出 (列向量)
% 输出:
%   out - 拼接后的bit列向量, 长度 G = sum(E_r)

% 计算总长度并预分配
totalLen = 0;
for r = 1:numel(cbOutputs)
    totalLen = totalLen + numel(cbOutputs{r});
end
out = zeros(totalLen, 1);

% 填充数据
idx = 1;
for r = 1:numel(cbOutputs)
    len = numel(cbOutputs{r});
    out(idx:idx+len-1) = cbOutputs{r}(:);
    idx = idx + len;
end
end
