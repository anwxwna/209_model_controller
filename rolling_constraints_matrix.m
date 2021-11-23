function roll_const = rolling_constraints_matrix(parameters)
% alpha,beta,l,d

% rolling constraints
roll_const=[];

for i=1:size(parameters,1)
    alpha   = parameters(i,1);
    beta    = parameters(i,2);
    l       = parameters(i,3);
    row_vector = [ -sin(alpha +beta) cos(alpha+beta) l*cos(beta)];
    roll_const = [ noslip; {row_vector} ];
end
end


