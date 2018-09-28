%% Setup profile parameters
% uses results from "spectral_density for comparison plots

B  = 1 ; % Sampling Interval (in)
L  = 400;  % Length Of Road Profile (ft)
N  = L*12/B; %  Number of data points
C = exp(-15.2); % roughness coefficient (not important)
w = 2.5; % waviness
move_increment = 12; % inches
move_length = 100*12; % inches

% convert to meters
B= B*0.0254;
L = L*0.3048;

%% Generate profile
% Construct amplitudes and phase angles for each spatial frequency
dn = 1/(L);  % Frequency Band
n  = dn : dn : N*dn; % Spatial Frequency Band (start higher in frequency to avoid large global elevation changes. Min freq>=1/20m)
n = n(find(1./n>=20,1,'last'):end);
nangle = n*2*pi; % angular frequency band
psd = C*n.^(-w); % fitted psd value for each frequency band
del_angle = (nangle(end)-nangle)/(N-1);
Amp1 = sqrt(psd.*del_angle/pi); % amplitude
x = 0:B:L-B; % Abscissa Variable from 0 to L
prof_dist = (x*39.3701)'; % in.
rng('shuffle') %shuffle random number generator
phi = 2*pi*rand(size(n)); % Random Phase Angle (uniformly ditributed)

% Sum sinusoids at each profile step
hx = zeros(size(x));
for jj=1:length(x)
    hx(jj) = sum(Amp1.*sin(nangle*x(jj) - phi));
end
profile = hx'-hx(1);    

%% establish remaining model parameters
fnb = 3.0; % natural frequency of bridge
fnv = 3.2; % natural frequency of vehicle
mb = 2000; % mass of bridge (slinch)
approach_length = 100; % length of approach profile (ft)
blength = 140; % ft
mt = 200; % mass of vehicle (slinch)
dt = 0.2; % vehicle damping ratio
vel = 720; %in/sec

wnb = fnb*2*pi;
wnv = fnv*2*pi;


%% Vehicle-Bridge
% Initiate state space models
vb_ss = ss_bridge_vehicle();

% populate parameters:
vb_ss.L = blength*12;

vb_ss.kt = (wnv)'.^2.*mt; % suspension stiffness for vehicle
vb_ss.mt = mt*vb_ss.gravity;
vb_ss.mb = mb*vb_ss.gravity;
vb_ss.ct = dt*2*sqrt(vb_ss.kt*vb_ss.mt/vb_ss.gravity);
vb_ss.EI = mb*vb_ss.L^3*wnb^2/pi^4;
vb_ss.vel = vel;


%% Vehicle model (for initial conditions)
veh = qcarSDF();
veh.ms = mt*veh.gravity;
veh.k = (wnv)'.^2.*mt; 
veh.c = dt*2*sqrt(veh.k*veh.ms/veh.gravity);
veh.vel = vel;    


LL_disp = -vb_ss.mt*vb_ss.L^3/(48*vb_ss.EI);
DL_disp = -5*vb_ss.mb*vb_ss.L^3/(384*vb_ss.EI);

iter = move_length/move_increment;

for ii = 1:(iter+1)
    % adjust beginning of profile    
      
    approach_ind = find(prof_dist>=(ii-1)*move_increment,1,'first'):find(prof_dist>=(approach_length*12+(ii-1)*move_increment),1,'first');
    bridge_ind = find(prof_dist>=(approach_length*12+(ii-1)*move_increment),1,'first'):find(prof_dist>=((approach_length+blength)*12+(ii-1)*move_increment),1,'first');

    % convert to inches
    prof_elev(:,ii) = (profile*39.3701); % in.
    
    %% Compute dynamic amplification due to profiles
    veh.dist = prof_dist(approach_ind);
    veh.profile = prof_elev(approach_ind,ii);
    vb_ss.dist = prof_dist(bridge_ind)-prof_dist(bridge_ind(1));
    vb_ss.profile = prof_elev(bridge_ind,ii);
    
    % get initial conditions
    yv0 = veh.simulate;
    % set initial conditions
    vb_ss.x0(3) = yv0(end,2); 
    vb_ss.x0(4) = yv0(end,1)-vb_ss.mt/vb_ss.kt; % make vehicle pre-displaced by self-weight
    yvb(:,:,ii) = vb_ss.simulate;
    Damp_LL(ii) = min(yvb(:,1,ii))/LL_disp;
    Damp_DL(ii) =  min(yvb(:,1,ii))/DL_disp;    
end
%
figure
plot((0:move_increment:(iter)*move_increment)/12,Damp_LL)
ylabel('Bridge Displacement Amplification')
xlabel('Profile position offset (ft)')

figure
plot(prof_dist/12, profile*39.3701)
xlabel('Distance (ft)')
ylabel('Elevation (in)')

figure
histogram(Damp_LL(1:iter))
xlabel('Bridge Displacement Amplification')

figure
[~, big_ind] = max(Damp_LL);
[~, small_ind] = min(Damp_LL);
plot(prof_dist, prof_elev(:,[big_ind small_ind]))
xlabel('distance (in)')
ylabel('elevation (in)')


figure
plot(vb_ss.time,permute(yvb(:,1,[big_ind small_ind]),[1 3 2])/LL_disp)
ylabel('Bridge Displacement Amplification')
xlabel('time (sec)')