classdef Node
    properties
        X
        Y
        ID

        Xhat
        Yhat

        PR = 200 % Perception Radius
    end

    methods
        function obj = Node(ID)
            if nargin == 1
                obj.ID = ID;
            end
        end
    end
end