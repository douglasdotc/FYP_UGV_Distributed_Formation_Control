function [Adj_VL, Adj] = graph_create(connections, N)
%GRAPH_CREATE gives two random graphs using Erdos-Renyi-Algorithm of n-vehicles and
%finds minimum directed spanning tree of these two graphs
%
% Inputs:
%     N            - number of vehicles
%     connections  - to leader connected vehicles in graph
% Outputs:
%     Adj_VL    - Consensus adjacency matrix graph with virtual leader connections
%     Adj       - Adjacency matrix graph without virtual leader

graphs = figure;
scrsz = get(groot, 'ScreenSize');
set(graphs, 'Name', 'Communication Topologies', 'NumberTitle', 'off','OuterPosition',[0 0 scrsz(3)/2 scrsz(4)/2]);

% call erdosRenyi 
G = graph(erdosRenyi(N,0.6), 'upper', 'OmitSelfLoops');
subplot(2,1,1);
p = plot(G);
set(gca,'XTickLabel',[],'YTickLabel',[])

% find minimum spanning tree from the first element in the connected list
% and create undirected graph from the root of the tree
[T,pred] = minspantree(G,'Type','forest','Root',connections(1));
highlight(p,T)
subplot(2,1,2);
p_3 = plot(T);
set(gca,'XTickLabel',[],'YTickLabel',[])

% creates last column and row for adjacency matrix
connect = connect_VL(connections,N);

% creates adjacency matrixes
disp('Adjacency matrixes first graph');
Adj = full(T.adjacency)'
Adj_VL = [Adj connect; connect' 0]

end

function [Adj_lColumn] = connect_VL(connections,N)
% creates the last column and row (connections to the VL) of a Adj depending on the
% connections specified
Adj_lColumn = zeros(N,1);
for i=1:1:length(connections)
    Adj_lColumn(connections(i)) = 1;
end
end

function [Adj] = erdosRenyi(N,p)
% creates random adjacency matrix according to Erdos-Renyi algorithm
Adj = triu(rand(N)<p,1); % upper triangular to avoid self-loops
Adj = Adj+Adj';
end


