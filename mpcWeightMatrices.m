function [Q,R] = mpcWeightMatrices(numWMR)
    Q ={}; R = {};
    q1 = 10; q2 = 30; q3 = 50;
    r1 = 1;
    
    for i = 1:numWMR
        if i == 1
            Q{i} = q2;
            R{i} = r1;
        else
            if i == 2
                Q{i} = diag([q1 q2 q2]);
                R{i} = diag([r1 r1]);
            else
                Q{i} = diag([q1 q1 q2 q3 q3 q3]);
                R{i} = diag([r1 r1 r1]);
            end
        end
    end
end

