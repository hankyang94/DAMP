function s_next = UpdateState(s_curr,s_dot,TimeStep)

% n = length(s_dot);
% s_dot = s_dot + 1e-5 * randn(n,1);

s_next = s_curr + TimeStep*s_dot;

s_next(4:7) = normalize(s_next(4:7));
end