function D = Dist(Node1, Node2, Type)
    if Type == 1            % Measured Distance
        D = norm([Node1.X - Node2.X, Node1.Y - Node2.Y], 2);
    elseif Type == 2        % Estimated Distance
        D = norm([Node1.Xhat - Node2.Xhat, Node1.Yhat - Node2.Yhat], 2);
    else
        error("Wrong Distance Type!")
    end
end