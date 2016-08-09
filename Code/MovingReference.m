%% Set structural characteristics
% Length in feet
L = 150;
% Mode number
M = 8;
% Corresponding Natural frequency (Hz)
freq_b = 0.5;

%% Set Traveler characteristics
% Velocity (feet per second)
vel_t = 25;

%% Set visualization scale (amplitude of shape)
sc = 10;
%% set structure acceleration amplitude
AA = 10;

% strcture position vector
dx = 1; %ft
x = 0:dx:L;

%Spatial frquency
freq_sp = M/2/L; 


% Time vector
dt = L/vel_t/100;
t = 0:dt:L/vel_t;

% structure vertical displacement
% y_b = sc*sin(2*pi*freq_sp*x);
n=1;
for ii = t
    y_b(:,n) = AA*sin(2*pi*freq_b*ii)*sin(2*pi*freq_sp*x);
    n=n+1;
end
% Travelor horizontal position
x_t = vel_t*t;

%Traveler vertical position
for ii = 1:length(t)
y_t(ii) = AA*sin(2*pi*freq_b*t(ii))*sin(2*pi*freq_sp*x_t(ii));
end

%% Plots
FH = figure;
fh1 = subplot(2,1,1);
%Plot shape
ah1 = plot(x,y_b(:,1));
axis equal

title(['Bridge Frequency: ' num2str(freq_b) 'Hz Vehicle Velocity: ' num2str(vel_t) ' fps']);
hold all
xlabel('Distance (ft.)');
fh2 = subplot(2,1,2);
%Plot traveler acceleration
ah2 = plot(t,y_t);
hold all
xlabel('Time (sec)');
%Plot structure acceleration
acc_dof = AA*sin(2*pi*freq_b*t);
plot(t,acc_dof,'r');

%Plot time bars
ylim1 = get(fh1,'ylim');
ylim2 = get(fh2,'ylim');
bar1 = plot(fh1,[0 0],ylim1,'k');
bar2 = plot(fh2,[0 0],ylim2,'k');
%Plot structure record location
spot = scatter(fh1,L/M/2,AA*sin(2*pi*freq_sp*L/M/2),'r','o');
spot2 = scatter(fh2,0, 0, 'r', 'o');
m=1;
for ii = t
    set(bar1,'XData',[vel_t*ii vel_t*ii])
    set(bar2,'XData',[ii ii])
    set(ah1,'YData',y_b(:,m))
    set(spot,'YData',AA*sin(2*pi*freq_b*ii)*sin(2*pi*freq_sp*L/M/2))
    set(spot2,'XData', ii)
    set(spot2, 'YData', acc_dof(m))
    drawnow 
    pause(1/10)
    m=m+1;
end
hold off