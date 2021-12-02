function no_slip = no_slip_constraints_matrix(parameters)
% alpha,beta,l,d

no_slip=[];
% no slip matrix
for i=1:size(parameters,1)
    alpha   = parameters(i,1);
    beta    = parameters(i,2);
    l       = parameters(i,3);
    row_vector = [-cos(alpha+beta), sin(alpha +beta), 1, l*sin(beta)] ;
    no_slip = [ no_slip; row_vector];
end
end