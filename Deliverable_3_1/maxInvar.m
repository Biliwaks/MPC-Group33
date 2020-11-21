%Function to compute maximum invariant set within the constraints X

function [Xf,i] = maxInvar(A, X)

% Compute the maximal invariant set
i = 0;
while 1
    i = i + 1;
    prevX = X;
    [T,t] = double(X);
    preX  = polytope(T*A,t);
    X     = intersect(X, preX);

    if isequal(prevX, X)
        break
    end

end

Xf = X;

end

