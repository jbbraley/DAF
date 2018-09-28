L = 0:100;

point = 1/50*L(1:51);
point = vertcat(point', point(end-1:-1:1)');

A = 4/(L(end))^2;
B = 4/L(end);
dist = -A*L.^2+B*L;

%dist = sin(pi/L(end)*L);

p2 = cumtrapz(cumtrapz(point));
d2 = cumtrapz(cumtrapz(dist'));

series = {'point load'; 'distributed load'};

figure('Position',[100 50 500 300])
ah1 = axes;
plot(L,[point dist'])
xlabel('distance')
ylabel('moment')
legend(series,'location','south')
ah1.XTick = [0 50 100];
ah1.XTickLabel = {'0'; 'L/2'; 'L'};

figure('Position',[100 50 500 300])
ah = axes;
plot(L,p2-p2(end)/L(end)*L')
hold all 
plot(L,d2-d2(end)/L(end)*L')
xlabel('distance')
ylabel('deflection')
legend(series,'location','north')
ah.XTick = [0 50 100];
ah.XTickLabel = {'0'; 'L/2'; 'L'};