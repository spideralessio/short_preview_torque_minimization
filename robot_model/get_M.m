function M = get_M(T, q_dot)

[rows, ~] = size(q_dot);
M = sym(zeros(rows, rows));
% childs = children(expand(T));
childs = feval(symengine, '(V) -> if testtype(V,"_plus") then [op(V)] else [V] end_if;', expand(T));
for r = 1:rows
    for c = 1:rows
        for idx=1:length(childs)
            fact = q_dot(r)*q_dot(c);
            has_childs = has(childs(idx), fact);
            if (has_childs)
                M(r,c) = M(r,c) + childs(idx)/fact;
            end
        end
        if (r ~= c)
           M(r,c) = M(r,c)/2;
        end
    end
end

M = simplify(M*2);