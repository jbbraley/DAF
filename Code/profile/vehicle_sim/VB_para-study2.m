
%% set up Run parameters
prof_amp = 0.1:0.1:0.5;
fnb = 1:5; % natural frequency of bridge
mb = 1000:200:2000; % mass of bridge (slinch)
mt = 60:20:200; % mass of vehicle (slinch)
db = 0.01; % bridge damping ratio
dt = 0.1; % vehicle damping ratio

%% Load profile
pro_file = file();
prof_wav = 360; %in wavelength
pro_file.name = 'pure-360in_0.5in.csv';
pro_file.path = 'C:\Users\John\Projects_Git\DAmp\profiles\artificial\harmonic2';
file_cont = dlmread(pro_file.fullname,',');

profile = file_cont(:,2);
%start profile at zero amplitude and zero slope
flat_ind = 1;
if sign(diff(profile(1:2)))>0
    flat_ind = find(diff(profile)<=0,1,'first');
else
    flat_ind = find(diff(profile)>=0,1,'first');
end
profile = profile(flat_ind:end)-profile(flat_ind);

% initiate vehicle-bridge object
vbs = VB();
vbs.profile = profile; 
vbs.dist = file_cont(flat_ind:end,1);
prof_amplitude = max(vbs.profile)-mean(vbs.profile);

%% populate dependent parameters
kt = (fnb*2*pi)'.^2.*mt; % suspension stiffness for vehicle
kb = (fnb*2*pi)'.^2.*mb; % stiffness of bridge (idealized as sprung mass)
ct = dt*2*sqrt(kt.*mt); % damping ratio of truck (lb.s./in)
cb = db*2*sqrt(kb.*mb); % damping ratio of bridge

%% Simulates a 2-DOF system subjected to a given profile
bridge_accel = [];
truck_accel = [];
bridge_disp = [];
truck_disp = [];
for prof = 1:length(prof_amp)
    elev_factor = prof_amp(prof)/prof_amplitude;
    vbs.profile = profile*elev_factor;
    for bmass = 1:length(mb)
        % bridge mass
        vbs.mb = mb(bmass)*386.09;
        for bridge = 1:length(fnb)
            % bridge parameters
            vbs.kb = kb(bridge,bmass);
            vbs.cb = cb(bridge,bmass);  
            % compute speed to create appropriate forcing freq
            vbs.vel = prof_wav*fnb(bridge);
            for vehicle = 1:length(mt)
                % vehicle parameters
                vbs.kt = kt(bridge,vehicle); % lb/in
                vbs.mt = mt(vehicle)*386.09;  % vehicle weight
                vbs.ct = ct(bridge,vehicle); % lb.s/in
                %% simulate for each speed
                yy = vbs.simulate;

                %% %% Gather results
                bridge_accel(:,vehicle,bridge,bmass,prof) = [0; diff(yy(:,4))/diff(vbs.time(1:2))];
                truck_accel(:,vehicle,bridge,bmass,prof) = [0; diff(yy(:,2))/diff(vbs.time(1:2))];
                bridge_disp(:,vehicle,bridge,bmass,prof) = yy(:,3);
                truck_disp(:,vehicle,bridge,bmass,prof) = yy(:,1);
                tt(:,bridge) = vbs.time;
            end
        end
    end
end

%% Examine maximum values

b_accel_max = permute(max(bridge_accel,[],1),[2 3 4 5 1]);
t_accel_max = permute(max(truck_accel,[],1),[2 3 4 5 1]);
b_disp_max = permute(max(bridge_disp,[],1),[2 3 4 5 1]);
t_disp_max = permute(max(truck_disp,[],1),[2 3 4 5 1]);
% freq = speed_range/(prof_wav);

%% Plot bridge displacement as a function of parameters
amp_ddl = [];
var = [];

for bmass = 1:length(mb)
    for bridge = 1:length(fnb)
        % compute bridge deflection amplification
        amp_ddl(:,bridge,bmass,:) = b_disp_max(:,bridge,bmass,:)/(mb(bmass)*386.06/kb(bridge,bmass));
        for vehicle = 1:length(mt)
            for prof = 1:length(prof_amp)
                % compute variable (profile acceleration)
                var(vehicle,bridge,bmass,prof) = prof_amp(prof)*(fnb(bridge))^2; %*(mt(vehicle))/mb(bmass);
            end
        end
    end
end
%% Determine slope/coefficients of damp vs profile accel
% dimensions to examine
dim = [1 3];
% DL deflection amplification
% damp = permute(amp_ddl,[2 4 1 3]);
% xx = permute(var,[2 4 1 3]);
% 
% damp = reshape(damp,[],prod(sz(dim)));
% xx = reshape(xx,[],prod(sz(dim)));

sz = size(var);
dims = 1:ndims(var);
inds = repmat({1},1,ndims(var));
for jj = dims
    inds{jj} = 1:sz(jj);
end

clear damp xx coeff params
for ii = 1:sz(dim(1))
    inds{dim(1)} = ii;
    for jj = 1:sz(dim(2))
        inds{dim(2)} = jj;
        damp(:,ii,jj) = reshape(amp_ddl(inds{:}),[],1);
        xx(:,ii,jj) = reshape(var(inds{:}),[],1);
        
%         % compute slope of each data set (xx,damp)
%         fit_obj = fit(xx(:,ii,jj),damp(:,ii,jj),'poly2','Lower', [-5 0 0],'Upper', [0 1 0]); %slope(ii,jj) = regress(damp,xx);
%         coeff(:,ii,jj) = [fit_obj.p1; fit_obj.p2];
%         params(:,ii,jj) = [mt(ii); mb(jj)];
    end
end

% % p1
% figure
% plot(permute(params(1,:,1),[2 3 1]),permute(coeff(1,:,:),[2 3 1]));
% leg1 = [repmat('bridge-mass:',length(mb),1) num2str(mb')];
% legend(leg1)
% xlabel('vehicle mass (slinch)')
% ylabel('p1 coefficient (p1*x^2)')
% figure
% plot(permute(params(2,1,:),[3 2 1]),permute(coeff(1,:,:),[3 2 1]),'o-')
% leg2 = [repmat('vehicle-mass:',length(mt),1) num2str(mt')];
% legend(leg2)
% xlabel('bridge mass (slinch)')
% ylabel('p1 coefficient (p1*x^2)')
% 
% 
% %p2
% figure
% plot(permute(params(1,:,1),[2 3 1]),permute(coeff(2,:,:),[2 3 1]));
% leg1 = [repmat('bridge-mass:',length(mb),1) num2str(mb')];
% legend(leg1)
% xlabel('vehicle mass (slinch)')
% ylabel('p2 coefficient (p2*x)')
% figure
% plot(permute(params(2,1,:),[3 2 1]),permute(coeff(2,:,:),[3 2 1]),'o-')
% leg2 = [repmat('vehicle-mass:',length(mt),1) num2str(mt')];
% legend(leg2)
% xlabel('bridge mass (slinch)')
% ylabel('p2 coefficient (p2*x)')


%% curve fit

ydat = reshape(damp,[],1); % amplification of dead load deflection
xdat = reshape(xx,[],1); % profile acceleration
dat1 = reshape(repmat(params(1,:,:),size(xx,1),1,1),[],1); % vehicle mass (slinch)
dat2 = reshape(repmat(params(2,:,:),size(xx,1),1,1),[],1); % bridge mass (slinch)

% fit function
fitf{1} = @(c,X)...
    (c(1)*X(:,2).^2+c(2)*X(:,3)+c(3)).*X(:,1).^2+(c(4)*X(:,2)+c(5)*X(:,3).^2+c(6)).*X(:,1);

mod = model();
mod.fun = fitf{1};
mod.num_c = 6;
mod.X = [xdat dat1 dat2];
mod.y = ydat;
% [cc, R, Err] = mod.fit;
% figure
% plot(mod.y,mod.fun(cc,mod.X))

% compare other model forms
fitf{2} = @(c,X)...
    (c(1)*X(:,2).^2+c(2)*X(:,3)+c(3)).*X(:,1).^2+(c(4)*X(:,2)+c(5)./X(:,3)+c(6)).*X(:,1); %
fitf{3} = @(c,X)...
    (c(4)*X(:,2)+c(5)*X(:,3).^2+c(6)).*X(:,1);
fitf{4} = @(c,X)...
    (c(4)*X(:,2)+c(5)./X(:,3)+c(6)).*X(:,1); %
fitf{5} = @(c,X)...
    (c(4)*X(:,2)./X(:,3)+c(6)).*X(:,1); %
clear cc
for model_fun = 1:length(fitf)
    mod.fun = fitf{model_fun};
    [cc(:,model_fun), ~, Err] = mod.fit; 
    MSE(model_fun) = Err.MSE;
end

% figure
% for fun = 1:size(cc,2)
%     scatter(fitf{fun}(cc(:,fun),mod.X),mod.y,'.')
%     hold all
% end
% plot([0 2],  [0 2],'black')

% Legends and labels
fun_name = [repmat('model ',size(cc,2),1) num2str((1:size(cc,2))') repmat(' | MSE=',size(cc,2),1) num2str(MSE','%.2e')];
legend(fun_name)    
xlabel('model predicted values');
ylabel('actual values')

% look at final number (with rounded coefficients)
final_cc = [2e-4 32 7.8e-2]
final_y = fitf{4}([0 0 0 final_cc],mod.X);
figure
scatter(mod.y,final_y,'.')
final_MSE = immse(final_y,mod.y)
hold on
scatter(mod.y,fitf{4}(cc(:,4),mod.X),'.')
plot([0 2],  [0 2],'black')
ylabel('model predicted values');
xlabel('actual values')