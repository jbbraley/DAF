
%% set up Run parameters
prof_amp = 0.1:0.1:0.5;
fnb = 1:5; % natural frequency of bridge
mb = 1000:200:2000; % mass of bridge (slinch)
mt = 60:20:200; % mass of vehicle (slinch)
db = [0.01 .02 .03 .04]; %:0.01:0.03; % bridge damping ratio
dt = 0.10:0.10:0.40; % vehicle damping ratio
freq_factor = linspace(0.8,1,5); % in/sec to be added to the target speed
wn = fnb*2*pi;

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
kt = (wn)'.^2.*mt; % suspension stiffness for vehicle
kb = (wn)'.^2.*mb; % stiffness of bridge (idealized as sprung mass)
% ct = dt*2*sqrt(kt.*mt); % damping ratio of truck (lb.s./in)
% cb = db*2*sqrt(kb.*mb); % damping ratio of bridge

%% Simulates a 2-DOF system subjected to a given profile
for prof = 1:length(prof_amp)
    elev_factor = prof_amp(prof)/prof_amplitude;
    vbs.profile = profile*elev_factor;
    for bmass = 1:length(mb)
        % bridge mass
        vbs.mb = mb(bmass)*386.09;
        for bridge = 1:length(fnb)
            % bridge parameters
            vbs.kb = kb(bridge,bmass);
%            % compute speed to create appropriate forcing freq
%             freq_prof(vehicle,bridge,bmass) = fnb(bridge);
            for bdamp = 1:length(db)
                %calculate bridge damping coefficient
                vbs.cb = db(bdamp)*2*sqrt(vbs.kb*vbs.mb/386.09);  
                for vehicle = 1:length(mt)
                    % vehicle parameters
                    vbs.kt = kt(bridge,vehicle); % lb/in
                    vbs.mt = mt(vehicle)*386.09;  % vehicle weight
%                     freq_prof(:,vehicle,bridge,bmass) = wn_2dof(mt(vehicle),mb(bmass),vbs.kt, vbs.kb); %profile frequency
                    freq_range = fnb(bridge)*freq_factor; %[linspace(freq_prof(1,vehicle,bridge,bmass)-diff(freq_prof(:,vehicle,bridge,bmass))/2,freq_prof(2,vehicle,bridge,bmass)+diff(freq_prof(:,vehicle,bridge,bmass))/2,5) fnb(bridge)];
                    for vdamp = 1:length(dt)
                        vbs.ct = dt(vdamp)*2*sqrt(vbs.kt*vbs.mt/386.09); % lb.s/in
                        for speed = 1:length(freq_range)
                            %% simulate for each speed
                            vbs.vel = prof_wav*freq_range(speed);
                            yy = vbs.simulate;

                            %% %% Gather results
                            bridge_accel(:,speed) = [0; diff(yy(:,4))/diff(vbs.time(1:2))];
                            truck_accel(:,speed) = [0; diff(yy(:,2))/diff(vbs.time(1:2))];
                            bridge_disp(:,speed) = yy(:,3);
                            truck_disp(:,speed) = yy(:,1);
                            
                            
                        end
                        % save maximum values
                        b_accel_max(vehicle,bridge,bmass,bdamp,vdamp,prof) = max(max(bridge_accel,[],1));
                        t_accel_max(vehicle,bridge,bmass,bdamp,vdamp,prof) = max(max(truck_accel,[],1));
                        [b_disp_max(vehicle,bridge,bmass,bdamp,vdamp,prof), max_disp_ind(vehicle,bridge,bmass,bdamp,vdamp,prof)]  = max(max(bridge_disp,[],1));
                        b_disp_max_diff(vehicle,bridge,bmass,bdamp,vdamp,prof) = max(bridge_disp(:,end),[],1)/max(max(bridge_disp,[],1));
                        t_disp_max(vehicle,bridge,bmass,bdamp,vdamp,prof)  = max(max(truck_disp,[],1));
                    end
                end
            end
            tt(:,bridge) = vbs.time;
        end
    end
end

% %% Examine maximum values
% 
% b_accel_max = permute(max(bridge_accel,[],1),[2 3 4 5 6 7 1]);
% t_accel_max = permute(max(truck_accel,[],1),[2 3 4 5 6 7 1]);
% b_disp_max = permute(max(bridge_disp,[],1),[2 3 4 5 6 7 1]);
% t_disp_max = permute(max(truck_disp,[],1),[2 3 4 5 6 7 1]);
% % freq = speed_range/(prof_wav);
%% Look at forcing freq. that causes max bridge displacement
% Forcing Freq
figure
histogram(freq_factor(reshape(max_disp_ind,[],1)),[freq_factor-diff(freq_factor(1:2)/2) freq_factor(end)+diff(freq_factor(1:2)/2)],'Normalization','probability');
xlabel('Fraction of Bridge Natural Freq.');
ylabel('Relative Probability')
title('Forcing Frequency that Causes Maximum Bridge Displacement')
% error using system nat. freq.
figure
histogram(1-reshape(b_disp_max_diff,[],1),10,'Normalization','probability');

%% compute bridge displacement and profile acceleration
for bmass = 1:length(mb)
    for bridge = 1:length(fnb)
        % compute bridge deflection amplification
        amp_ddl(:,bridge,bmass,:,:,:) = b_disp_max(:,bridge,bmass,:,:,:)/(mb(bmass)*386.06/kb(bridge,bmass));
        for bdamp = 1:length(db)
            for vehicle = 1:length(mt)
                for vdamp = 1:length(dt)
                    for prof = 1:length(prof_amp)
                        % compute variable (profile acceleration)
                        var(vehicle,bridge,bmass,bdamp,vdamp,prof) = prof_amp(prof)*(fnb(bridge))^2; %freq_prof(vehicle,bridge,bmass)^2; %*(mt(vehicle))/mb(bmass);
                        var2(vehicle,bridge,bmass,bdamp,vdamp,prof) = prof_amp(prof)*(fnb(bridge)*freq_factor(max_disp_ind(vehicle,bridge,bmass,bdamp,vdamp,prof)))^2;
                    end
                end
            end
        end
        
    end
end

%% Assemble parameters
% dimensions to examine
% dim = [1 2 3 4 5];
% sz = size(var);
% dims = 1:ndims(var);
% inds = repmat({1},1,ndims(var));
% for jj = dims
%     inds{jj} = 1:sz(jj);
% end
% 
% clear damp xx x2 coeff params
% for ii = 1:sz(dim(1))
%     inds{dim(1)} = ii;
%     for jj = 1:sz(dim(2))
%         inds{dim(2)} = jj;
%         for kk = 1:sz(dim(3))
%             inds{dim(3)} = kk;
%             for pp = 1:sz(dim(4))
%                 inds{dim(4)} = pp;   
%                 for qq = 1:sz(dim(5))
%                     inds{dim(5)} = qq;
%                     damp(:,ii,jj,kk,pp,qq) = reshape(amp_ddl(inds{:}),[],1); % displacmenet amplification
%                     xx(:,ii,jj,kk,pp,qq) = reshape(var(inds{:}),[],1); % profile acceleration variable
%                     x2(:,ii,jj,kk,pp,qq) = reshape(var2(inds{:}),[],1); % profile acceleration variable
%                     params(:,ii,jj,kk,pp,qq) = [mt(ii); mb(kk); db(pp); dt(qq)];  % *2*wn(jj)*mb(kk) *2*mt(ii)*wn(jj)
%                 end
%             end
%         end
%     end
% end


% % error vs parameter
% figure
% scatter(reshape(repmat(amp_ddl,1,1,1,1,1,1),[],1), reshape(b_disp_max_diff,[],1))
% scatter(reshape(repmat(mb./mt',1,5,1,4,4,5),[],1), reshape(b_disp_max_diff,[],1))

 %% Determine slope/coefficients of damp vs profile accel
% % figure
% % plot_dim = 6;
% % for ii = 1:size(xx,plot_dim)
% %     plot(permute(xx(:,end,1,1,1,ii),[1 3 2 4 5 6]),permute(damp(:,1,1,1,1,ii),[1 3 2 4 5 6]),'-o')
% %     hold all
% % end
% 
% % prof. accel. var. uses actual frequency of profile
% % parameter defaults
% plot_dim = 3;
% param_def = [8 3 6 2 2]; % [mt fnb mb db dt]
% dims = 1:ndims(x2);
% sz = size(x2);
% plot_inds = repmat({1},1,ndims(var));
% for jj = dims
%     if jj==1 || jj==plot_dim
%         plot_inds{jj} = 1:sz(jj);
%     else
%         plot_inds{jj} = param_def(jj-1);
%     end
% end
% 
% % figure
% 
% dim_inds = 2:ndims(x2);
% dim_inds = [1 plot_dim dim_inds(dim_inds~=plot_dim)];
% plot(permute(x2(plot_inds{:}),dim_inds),permute(damp(plot_inds{:}),dim_inds),'-o')
% 
% xlim([0 5]);
% xlabel('Profile Acceleration in/sec^2')
% ylabel('Bridge Displacement/Dead Load Displacement')
% leg = {'vehicle mass'; 'bridge natural frequency'; 'bridge mass'; 'bridge damping ratio'; 'vehicle damping ratio'};
% par_name = {'mt'; 'fnb'; 'mb'; 'db'; 'dt'};
% legend([repmat([leg{plot_dim-1} ': '],size(x2,plot_dim),1) num2str(eval(par_name{plot_dim-1})')])
% 
% hold all
%% examine slope of linear fit (plotted against parameters)

dim = [1 3 4 5];
sz = size(var);
dims = 1:ndims(var);
inds = repmat({1},1,ndims(var));
for jj = dims
    inds{jj} = 1:sz(jj);
end
inds_master = inds;
clear damp xx x2 coeff params
for ii = 1:sz(dim(1))
    inds{dim(1)} = ii;
    for jj = 1:sz(dim(2))
        inds{dim(2)} = jj;
        for kk = 1:sz(dim(3))
            inds{dim(3)} = kk;
            for pp = 1:sz(dim(4))
                inds{dim(4)} = pp;
                damp(:,ii,jj,kk,pp) = reshape(amp_ddl(inds{:}),[],1);
                xx(:,ii,jj,kk,pp) = reshape(var(inds{:}),[],1);
                x2(:,ii,jj,kk,pp) = reshape(var2(inds{:}),[],1); % profile acceleration variable
%                 % compute slope of each data set (xx,damp)
%                 fit_obj = fit(x2(:,ii,jj,kk,pp),damp(:,ii,jj,kk,pp),'poly1','Lower', [0 0],'Upper', [20 0]); %slope(ii,jj) = regress(damp,xx);
%                 coeff(ii,jj,kk,pp) = [fit_obj.p1];
                params(:,ii,jj,kk,pp) = [mt(ii); mb(jj); db(kk); dt(pp)];
            end
        end
    end
end

% plot slopes vs each parameter

%truck mass
figure
plot(repmat(mt',1,size(coeff,2)),coeff(:,:,2,2),'-o')
xlabel('Truck mass (slinch)')
ylabel('coefficient value')
title('Truck mass effect on slope')
legend([repmat(['bridge mass: '],size(coeff,2),1) num2str(eval(par_name{3})')])
%bridge mass
figure
plot(repmat(mb',1,size(coeff,1)),permute(coeff(:,:,2,2),[2 1 3 4]),'-o')
xlabel('Bridge mass (slinch)')
ylabel('coefficient value')
title('Bridge mass effect on slope')
legend([repmat('truck mass: ',size(coeff,1),1) num2str(eval(par_name{1})')])
% bridge damping
figure
plot(repmat(db',1,size(coeff,2)),permute(coeff(8,:,:,2),[3 2 1 4]),'-o')
xlabel('Bridge damping ratio')
ylabel('coefficient value')
legend([repmat('bridge mass: ',size(coeff,2),1) num2str(eval(par_name{3})')])
% truck damping
figure
plot(repmat(dt',1,size(coeff,2)),permute(coeff(8,:,2,:),[4 2 1 3]),'-o')
xlabel('Truck damping ratio')
ylabel('coefficient value')
legend([repmat('bridge mass: ',size(coeff,2),1) num2str(eval(par_name{3})')])
figure
plot(repmat(dt',1,size(coeff,3)),permute(coeff(8,6,:,:),[4 3 1 2]),'-o')
xlabel('Truck damping ratio')
ylabel('coefficient value')
legend([repmat('bridge damping: ',size(coeff,3),1) num2str(eval(par_name{4})')])
% Truck mass/Bridge Mass
figure
for ii = 1:length(dt)
    scatter(reshape(mt'./mb,[],1), reshape(coeff(:,:,2,ii),[],1))
    hold all
end
xlabel('Truck Mass/Bridge Mass')
ylabel('coefficient value')
legend([repmat(['bridge mass: '],size(coeff,2),1) num2str(eval(par_name{3})')])



%% curve fit

ydat = reshape(damp,[],1); % amplification of dead load deflection
xdat = reshape(x2,[],1); % profile acceleration
dat1 = reshape(repmat(params(1,:,:,:,:),size(xx,1),1,1,1,1),[],1); % vehicle mass (slinch)
dat2 = reshape(repmat(params(2,:,:,:,:),size(xx,1),1,1,1,1),[],1); % bridge mass (slinch)
dat3 = reshape(repmat(params(3,:,:,:,:),size(xx,1),1,1,1,1),[],1); % bridge damping ratio
dat4 = reshape(repmat(params(4,:,:,:,:),size(xx,1),1,1,1,1),[],1); % vehicle damping ratio


%% fit function
fitf{1} = @(c,X)...
    (c(1)./X(:,3) + c(2)./X(:,4) + c(3)).*X(:,1)./X(:,2).*X(:,5)+c(4); %
fitf{2} = @(c,X)...
    (c(1)./X(:,3) + c(2)./X(:,4) + c(3)*X(:,1)./X(:,2).^2+c(4)).*X(:,5)+c(5); %
fitf{3} = @(c,X)...
    (c(1)./X(:,3) + c(2)*X(:,1)./(X(:,4))./X(:,2) + c(3)).*X(:,5)+c(4); %
fitf{4} = @(c,X)...
    (c(1)*X(:,1)./(X(:,4))./X(:,2)./X(:,3) + c(2)).*X(:,5)+c(3); %

mod = model();
mod.fun = fitf{4};
mod.num_c = 3;
mod.c_start = [];
mod.X = [dat1 dat2 dat3 dat4 xdat];
mod.y = ydat;
[cc, R, Err] = mod.fit;
figure
scatter(mod.y,mod.fun(cc,mod.X),'.')
hold on
plot([0 4],  [0 4],'black')
ylabel('model predicted values');
xlabel('actual values')

% Plot actual vs predicted by parameter
plot_dim = 3; % bridge mass
% reshape data such that it is nxlength(mb)
for ii = 1:length(mb)
end

%% ______________________________________________%%
%%%%%%%%%%%%% Work completed up to here%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
final_cc = [0.3 2.9e-4 4.3e-3 0.029]
final_y = fitf{1}(final_cc,mod.X);
final_MSE = immse(final_y,mod.y);

figure
scatter(mod.y,final_y,'.')
hold on
plot([0 2],  [0 2],'black')
ylabel('model predicted values');
xlabel('actual values')

figure
scatter(mod.y, (final_y-mod.y)./mod.y)


%% compare other model forms
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

figure
for fun = 1:size(cc,2)
    scatter(mod.y,fitf{fun}(cc(:,fun),mod.X),'.')
    hold all
end
plot([0 2],  [0 2],'black')

% Legends and labels
fun_name = [repmat('model ',size(cc,2),1) num2str((1:size(cc,2))') repmat(' | MSE=',size(cc,2),1) num2str(MSE','%.2e')];
legend(fun_name)    
ylabel('model predicted values');
xlabel('actual values')

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