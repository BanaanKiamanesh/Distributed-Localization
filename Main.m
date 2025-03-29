clear
close all
clc
% rng(1)

%% Parameter Init
% Color Schemes
CSActual.Nodes = 'r';
CSActual.Edges = [0, 0, 1, 0.1];

CSEstimated.Nodes = 'b';
CSEstimated.Edges = [0, 1, 1, 0.1];

N = Network(Room);
N.plot(CSActual, 1);
disp(['Min Degree of the Network is ' num2str(N.GetMinDegree) '.'])

Errors = N.Localize;

hold on
N.plot(CSEstimated, 2);

figure
plot(Errors)

