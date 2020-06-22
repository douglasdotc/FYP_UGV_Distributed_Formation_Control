function [r_rel_1, r_rel_2] = create_r_relative(N)
% creates three formations of n-vehicles in different shapes
%
% Inputs:
%     N - number of vehicles
% Outputs:
%     r_rel_1 - first formation here line-shape
%     r_rel_2 - second formation here parabolic-shape

% Line Shape
length = 0.5;
theta = 0;
x_line = linspace(length,length/N,N)*cos(theta);
y_line = linspace(length,length/N,N)*sin(theta);
r_rel_line = [x_line; y_line; theta*ones(1,N)];

% Polygon Shape
rad = 0.5;
r_rel_ploy = zeros(3,N);
for i=1:1:N
    theta = 2*pi*i/N;
    r_rel_ploy(:,i) = [rad*cos(theta); rad*sin(theta); theta];
end

r_rel_2 = r_rel_line;
r_rel_1 = r_rel_ploy;

% plot of formations
pos = figure;
scrsz = get(groot,'ScreenSize');
set(pos, 'Name', 'Relative positions2 / Formation', 'NumberTitle', 'off', 'OuterPosition',[0 scrsz(4)/2 scrsz(3)/2 scrsz(4)/2]);

for i=1:1:N
    subplot(1,2,1);
    scatter(r_rel_1(1,i), r_rel_1(2,i),'o','filled');
    hold on;
    subplot(1,2,2);
    scatter(r_rel_2(1,i), r_rel_2(2,i),'o','filled');
    hold on;
end

subplot(1,2,1);
plot(0,0,'x');
subplot(1,2,2);
plot(0,0,'x');
end