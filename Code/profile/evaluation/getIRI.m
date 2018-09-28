function [IRI,yy] = getIRI(profile,dist,sample_length,x0)
%GETIRI Computes the international roughness index of the profile over the
%specified sample length by simulating the "golden car" over the profile
%(returns IRI in inch/inch)
% Code validated against IRI computed by proVAL 3.6.1
if nargin<4
    x0 = [];
end
if nargin<3 || isempty(sample_length)
    sample_length = dist(end-1)-dist(1);
end

% % filter tire effect (moving average of 250 mm)
dd = mean(diff(dist));
lb = 9.84252; % inches
kk = max(1,nearest(lb/dd));
psmooth = ones(length(profile),kk)*profile(end);
for ii = 1:kk
    psmooth(1:end-(ii-1),ii) = profile(ii:end);
end
profile = sum(psmooth,2)/kk;

% initiate golden car state space modeling object
model = gcar();
% assign golden car parameters
model.k =  224.8089431/39.37007874*569.7; % (lb/in)
model.c = 224.8089431/39.37007874*54; % (lb-s/in)
model.ms = 19841.6; % (lb)
model.kus = 224.8089431/39.37007874*5877; % (lb/in)
model.mus = 2976.241; % (lb)
model.vel = 80*10.9361; % 80km/hr (in/sec)
model.dist = dist;
model.profile = profile;

% Set initial conditions
if isempty(x0)
    % sprung and unsprung mass start at elevation of first profile point
    % sprung and unsprung mass velocity start at average profile velocity over
    % the first 0.5 sec
    dt = dd/model.vel;
    begin_disp = profile(1);
    approach = 0.5*model.vel; app_el = nearest(approach/dd);
    begin_vel = mean(diff(profile(1:app_el))/dt);
    model.x0 = [begin_disp 0 begin_vel begin_vel]';
else
    model.x0 = x0;
end

% Run Simulation
yy = model.simulate;

% Compute IRI
% compute over sample length
travel = diff(yy(:,5));

nel = nearest(sample_length/dd+1);
left = rem(length(travel),nel);
IRI = 1/sample_length.*sum(reshape(abs(travel(1:end-left)),nel,[]),1);
if left
    IRI(end+1) = 1/(dist(end)-dist(end-left+1)).*sum(abs(travel(end-left+1:end)),1);
end

