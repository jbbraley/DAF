% profile from straightedge

% straightedge specifications
s_length = 10; % ft
s_dev = 0.125; % in

% profile specification
% spatial wavelength
pro_wav = 28; % ft
dx = 1; %in

% fit surface to deviation
% sin
A = s_dev/(1-sin(2*pi/(pro_wav*12)*(pro_wav*12/4+s_length*12/2)));

% plot profile
dist = 0:1200;
prof = A*sin(2*pi/(pro_wav*12)*dist);
% figure
% plot(dist,prof)
% hold all
% line([pro_wav*12/4-s_length*12/2 pro_wav*12/4+s_length*12/2], A*sin(2*pi/(pro_wav*12)*([pro_wav*12/4-s_length*12/2 pro_wav*12/4+s_length*12/2])))


% filter out straightedge deviation
[prof2 exitf] = straightedge_filter(prof,dist,s_length*12,s_dev);

figure
plot(dist,horzcat(prof',prof2'))
% 
% for ii = 1:(length(prof)-s_length*12/dx)
%     ind1 = ii; ind2 = ii+s_length*12/dx-1;
%     slope = linspace(prof(ind1),prof(ind2),s_length*12/dx);
%     push_ind = find(prof(ind1:ind2)-slope+s_dev<0);
%     prof(push_ind+ii-1) = slope(push_ind)-s_dev;
%     pull_ind = find(prof(ind1:ind2)-slope-s_dev>0);
%     prof(pull_ind+ii-1) = slope(pull_ind)+s_dev;
%     
% end
% plot(dist,prof)
