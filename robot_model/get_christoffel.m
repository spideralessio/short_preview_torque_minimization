function [c, C] = get_christoffel(M, q, q_dot)
[njoints, ~] = size(q);

C = cell(1,njoints);
c = sym(zeros(njoints,1));


for idx=1:njoints
    f1 = functionalJacobian(M(:,idx),q);
    f2 = f1.';
    f3 = sym(zeros(njoints, njoints));
    for joint_idx=1:njoints
        f3(:,joint_idx) = functionalJacobian(M(:, joint_idx), q(idx));
    end
    C{idx} = (f1 + f2 - f3)/2;
    
    ci = q_dot.' * C{idx} * q_dot;
    c(idx) = ci;
end  