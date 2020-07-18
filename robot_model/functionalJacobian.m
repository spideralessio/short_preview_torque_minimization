function J = functionalJacobian(f,v)

J = sym(zeros(length(f), length(v)));
for idx=1:length(f)
    J(idx,:) = functionalDerivative(f(idx), v).';
end

