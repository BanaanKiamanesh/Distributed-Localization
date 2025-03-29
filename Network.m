classdef Network < handle
    properties
        N = 200      

        Nodes       Node
        NetworkG    
        Ref                 % Reference Nodes
    end

    methods
        function Obj = Network(Room)
            % Generate Random Node Positions
            Obj.Nodes = repmat(Node, Obj.N, 1);
            for i = 1:Obj.N
                tmpNode = Node(i);
                tmpNode.X = unifrnd(Room.XL, Room.XH);
                tmpNode.Y = unifrnd(Room.YL, Room.YH);

                Obj.Nodes(i) = tmpNode;
            end

            Obj.NetworkG = Obj.GetNetworkGraph;
            Obj.FindRefPoints;
        end

        function L = GetNeighbors(Obj, ID)
            PR = Obj.Nodes(ID).PR;

            L = [];
            for i = 1:Obj.N
                D = Dist(Obj.Nodes(ID), Obj.Nodes(i), 1);
                if D < PR
                    L = [L, i]; %#ok<AGROW>
                end
            end

            L = setdiff(L, ID);
        end

        function G = GetNetworkGraph(Obj)
            if isempty(Obj.NetworkG)
                Edges = [];
                for i = 1:Obj.N
                    L = Obj.GetNeighbors(i);

                    tmpEdges = [i * ones(numel(L), 1), L(:)];
                    Edges = [Edges; tmpEdges]; %#ok<AGROW>
                end

                G = graph(Edges(:,1), Edges(:,2));
                G.Nodes.X = [Obj.Nodes.X]';
                G.Nodes.Y = [Obj.Nodes.Y]';

                Obj.NetworkG = G;
            else
                G = Obj.NetworkG;
            end
        end

        function D = GetMinDegree(Obj)
            D = min(degree(Obj.GetNetworkGraph) / 2);
        end

        function plot(Obj, ColorScheme, Type)
            G = Obj.GetNetworkGraph;
            Edges = table2array(G.Edges);

            % Plot the Edges
            hold on
            for i = 1:size(Edges, 1)
                N1 = Edges(i, 1);
                N2 = Edges(i, 2);

                if Type == 1
                    plot([Obj.Nodes(N1).X, Obj.Nodes(N2).X], ...
                        [Obj.Nodes(N1).Y, Obj.Nodes(N2).Y], ...
                        'Color', ColorScheme.Edges, 'LineWidth', 0.5, ...
                        'HandleVisibility', 'off');
                elseif Type == 2

                    plot([Obj.Nodes(N1).Xhat, Obj.Nodes(N2).Xhat], ...
                        [Obj.Nodes(N1).Yhat, Obj.Nodes(N2).Yhat], ...
                        'Color', ColorScheme.Edges, 'LineWidth', 0.5, ...
                        'HandleVisibility', 'off');
                end
            end

            if Type == 1
                scatter([Obj.Nodes.X], [Obj.Nodes.Y], 100, ColorScheme.Nodes, 'filled', 'o', 'DisplayName', 'Actual Pos');
                text([Obj.Nodes.X], [Obj.Nodes.Y] + 4, string([Obj.Nodes.ID]));
            elseif Type == 2
                scatter([Obj.Nodes.Xhat], [Obj.Nodes.Yhat], 100, ColorScheme.Nodes, 'filled', 'o', 'DisplayName', 'Actual Pos');
                text([Obj.Nodes.Xhat], [Obj.Nodes.Yhat] + 4, string([Obj.Nodes.ID]));
            end
            
            axis(axis + [-1, 1, -1, 1]*2);
            hold off
            axis equal
            axis tight
            axis off
        end

        function FindRefPoints(Obj)
            %%% Step 1
            % n0 Determination
            n0 = 1;

            % n1 Determination
            H0 = HopCount(Obj, n0, 2:Obj.N);
            [~, id] = max(H0(:, 2));
            n1 = H0(id, 1);

            %%% Step 2
            H1 = HopCount(Obj, n1, setdiff(1:Obj.N, [n0, n1]));
            [~, id] = max(H1(:, 2));
            n2 = H1(id, 1);

            %%% Step 3
            H1 = HopCount(Obj, n1, setdiff(1:Obj.N, [n0, n1, n2]));
            H2 = HopCount(Obj, n2, setdiff(1:Obj.N, [n0, n1, n2]));

            MinSS = abs(H1(:, 2) - H2(:, 2));           % Min Search Space
            MinIdx = find(MinSS == min(MinSS));

            MaxSS = H1(MinIdx, 2) + H2(MinIdx, 2);      % Max Search Space
            MaxIdx = find(MaxSS == max(MaxSS));
            n3 = H1(MinIdx(MaxIdx(1)), 1);

            %%% Step 4
            H1 = HopCount(Obj, n1, setdiff(1:Obj.N, [n0, n1, n2, n3]));
            H2 = HopCount(Obj, n2, setdiff(1:Obj.N, [n0, n1, n2, n3]));
            H3 = HopCount(Obj, n3, setdiff(1:Obj.N, [n0, n1, n2, n3]));

            MinSS = abs(H1(:, 2) - H2(:, 2));           % Min Search Space
            MinIdx = find(MinSS == min(MinSS));

            MaxSS = H3(MinIdx, 2);                      % Max Search Space
            MaxIdx = find(MaxSS == max(MaxSS));
            n4 = H1(MinIdx(MaxIdx(1)), 1);

            % %%% Step 5
            H1 = HopCount(Obj, n1, setdiff(1:Obj.N, [n0, n1, n2, n3, n4]));
            H2 = HopCount(Obj, n2, setdiff(1:Obj.N, [n0, n1, n2, n3, n4]));
            H4 = HopCount(Obj, n4, setdiff(1:Obj.N, [n0, n1, n2, n3, n4]));

            MinSS = abs(H1(:, 2) - H2(:, 2));           % Min Search Space
            MinIdx = find(MinSS == min(MinSS));

            MaxSS = abs(H3(MinIdx, 2) - H4(MinIdx, 2)); % Max Search Space
            MaxIdx = find(MaxSS == max(MaxSS));
            n5 = H1(MinIdx(MaxIdx(1)), 1);

            Obj.Ref = [n0, n1, n2, n3, n4, n5];
        end

        function [PInit, TInit] = CalcInitPos(Obj)
            H1 = HopCount(Obj, Obj.Ref(2), 1:Obj.N);
            H2 = HopCount(Obj, Obj.Ref(3), 1:Obj.N);
            H3 = HopCount(Obj, Obj.Ref(4), 1:Obj.N);
            H4 = HopCount(Obj, Obj.Ref(5), 1:Obj.N);
            H5 = HopCount(Obj, Obj.Ref(6), 1:Obj.N);
            
            % Radio Range
            R = Obj.Nodes(1).PR;

            PInit = H5(:, 2) * R;
            TInit = atan2(H1(:, 2) - H2(:, 2), ...
                          H3(:, 2) - H4(:, 2));
        end

        function Errors = Localize(Obj)
            % Get the Initial Conditions
            [PInit, TInit] = Obj.CalcInitPos;

            % Initialize Cartesian Position
            for i = 1:Obj.N
                Obj.Nodes(i).Xhat = PInit(i) * cos(TInit(i));
                Obj.Nodes(i).Yhat = PInit(i) * sin(TInit(i));
            end

            % Solve the Estimation Process
            K = 1000;
            Errors = zeros(K, 1);
            for i = 1:K                 % Simulation Step
                disp(['-------------- Iteration = ' num2str(i) ' --------------'])
                Force = zeros(2, Obj.N);
                for j = 1:Obj.N         % For Each Node
                    % Neighbors
                    L = Obj.GetNeighbors(j);
                    M = numel(L);

                    % Calculate and Sum All the Forces Acting on it
                    F = zeros(2, M);
                    for k = 1:M
                        % Get the Distances
                        R = Dist(Obj.Nodes(j), Obj.Nodes(L(k)), 1);
                        D = Dist(Obj.Nodes(j), Obj.Nodes(L(k)), 2);

                        % Calculate the Unit Matrix in Their Direction
                        V = [Obj.Nodes(j).Xhat - Obj.Nodes(L(k)).Xhat;
                             Obj.Nodes(j).Yhat - Obj.Nodes(L(k)).Yhat];
                        V = V / norm(V);

                        V(isnan(V)) = 0;

                        % Sum Up All the Forces
                        F(:, k) = -V * (D - R);
                    end
                    Force(:, j) = sum(F, 2) / 2 / M;
                end

                % Move the Nodes
                for j = 1:Obj.N
                    Obj.Nodes(j).Xhat = Obj.Nodes(j).Xhat + Force(1, j);
                    Obj.Nodes(j).Yhat = Obj.Nodes(j).Yhat + Force(2, j);
                end

                % Error Calculation
                Err = 0;
                for j = 1:Obj.N
                    Err = Err + norm([(Obj.Nodes(j).Xhat - Obj.Nodes(j).X), ...
                                      (Obj.Nodes(j).Yhat - Obj.Nodes(j).Y)], 2);
                end

                n = size(table2array(Obj.NetworkG.Edges), 1) / 2;

                Errors(i) = Err / (n * (n-1) / 2);
            end
        end
    end
end