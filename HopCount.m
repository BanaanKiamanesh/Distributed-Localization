function [N, SP] = HopCount(NW, Source, Targets)
    N = zeros(numel(Targets), 2);
    
    for i = 1:numel(Targets)
        [SP, ~] = shortestpath(NW.NetworkG, Source, Targets(i));
        N(i, 2) = numel(SP);
        N(i, 1) = Targets(i);
    end
end