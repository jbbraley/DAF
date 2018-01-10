% spline fit the ramp profile
xp = [0 25 60 75 200 1544]*12;
yp = [0 0 6 0 0 0];

new_disp = spline(xp,yp,dist);

figure
plot(dist/12,new_disp)
hold all
plot(dist/12,disp)
ah = gca;
ah.XLim = [0 200];

%try conv
window = 30*12/resolution;
kernel = ones(1, window) / window;
ww = conv(disp, kernel, 'same');
plot(dist/12,ww)

ww_new = ww/max(ww)*ramp_height;
plot(dist,ww_new)

%% try just a ramp down 
disp = zeros(length(dist),1);
disp(1:76) = linspace(ramp_height,0,76);
win_size = 60;
window = win_size*12/resolution;
kernel = ones(1, window) / window;
ww = conv(disp, kernel, 'same');

ww(1:window/2) = linspace(ramp_height,ww(window/2),window/2);
disp = ww-max(ww);

figure
plot(dist/12,disp)
