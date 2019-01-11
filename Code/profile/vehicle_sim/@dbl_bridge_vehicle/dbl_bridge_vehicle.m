classdef dbl_bridge_vehicle < handle
%% classdef ss_bridge_vehicle
% 
% 
% 
% author: John Braley
% create date: 2018-07-26 05:43:45.258
% classy version: 0.1.2

%% object properties
	properties
        EI              % flexural rigidity (lb-in^2)
        L               % span length (in) (length of single span)
        profile         % elevation profile over which the model traverses (in);
        dist            % vector specifying location of each profile point (in)
        vel             % vehicle speed (in/sec)
        kt               % suspension stiffness (lb/in)
        ct               % damping coefficient (lb-s/in)
        mt              % mass of truck (lb)
        mb              % mass of single span of bridge (lb)
        db = 0;              % bridge percent damping
        x               % location to record bridge results (in)
        x0 =[];         % initial state conditions [bridge velocity, bridge disp, vehicle velocity (vert.), vehicle disp (vert)]
        gravity = 386.09; % in/sec^2
        ssfun = @(z,u,A,B,F) A*z+B*u+F;
        ssout_fun = @(z,u,C,D) C*z+D*u;
        bridge_start = 0;    % location of beginning of bridge (in)
	end

%% dependent properties
	properties (Dependent)
        time
        ssmodel
        ssmodel_off
        ssout
        dt
        max_deflection
        fnb  % bridge natural frequency (Hz)
	end

%% private properties
	properties (Access = private)
	end

%% constructor
	methods
		function self = ss_bridge_vehicle()
		end
	end

%% ordinary methods
	methods 
        
	end % /ordinary

%% dependent methods
	methods 
        function max_deflection = get.max_deflection(self)
            max_deflection = self.mt*self.L^3/(pi^4*self.EI); % self.mt*self.L^3*0.015/(self.EI);
        end
        
        function fnb = get.fnb(self)
           fnb = pi/2*sqrt(self.EI/(self.mb/self.gravity*self.L^3)); 
        end
               
        function time = get.time(self)
        %% time - get time based on distance and velocity
            if isempty(self.dist) || isempty(self.vel)
                time = [];
            else
                if any(round(diff(self.dist),5)~=round(diff(self.dist(1:2)),5)) || self.dist(1)~=0
                    fprintf('distance must start from zero and be evenly spaced, constructing new distance vector based on average distance step\n')
                    dx = round(mean(diff(self.dist)),5);
                    self.dist = (0:dx:(length(self.dist)-1)*dx)';
                end
                time = (self.dist-self.bridge_start)/self.vel;
            end
        end
        
        function dt = get.dt(self)
            dt = diff(self.time(1:2));
        end
        
        function ssmodel = get.ssmodel(self)
           %%  contains the matrices A & B for input into the ssfun
           A = @(t) [-self.ct/(self.mb/self.gravity)*(sin(pi*self.vel*t/self.L))^2-2*self.db*(2*pi*self.fnb) -self.kt/(self.mb/self.gravity)*(sin(pi*self.vel*t/self.L))^2-pi^4*self.EI/(self.mb/self.gravity*self.L^3) self.ct/(self.mb/self.gravity)*(sin(pi*self.vel*t/self.L)) self.kt/(self.mb/self.gravity)*(sin(pi*self.vel*t/self.L));...
               1 0 0 0;...
               self.ct/(self.mt/self.gravity)*sin(pi*self.vel*t/self.L) self.kt/(self.mt/self.gravity)*sin(pi*self.vel*t/self.L) -self.ct/(self.mt/self.gravity) -self.kt/(self.mt/self.gravity);...
               0 0 1 0];
           B = @(t) [-self.ct/(self.mb/self.gravity)*sin(pi*self.vel*t/self.L) -self.kt/(self.mb/self.gravity)*sin(pi*self.vel*t/self.L);... 
               0 0;...
               self.ct/(self.mt/self.gravity) self.kt/(self.mt/self.gravity);...
               0 0];
           
           F = [0; 0; -self.gravity; 0];
           
           ssmodel = {A; B; F};
        end
        
        function ssmodel_off = get.ssmodel_off(self)
           %% contains matrices for input into ssfun for when the vehicle is not on the bridge (i.e. before or after)
           A = [-2*self.db*(2*pi*self.fnb) -pi^4*self.EI/(self.mb/self.gravity*self.L^3) 0 0;...
               1 0 0 0;...
               0 0 -self.ct/(self.mt/self.gravity) -self.kt/(self.mt/self.gravity);...
               0 0 1 0];
           B = [0 0;... 
               0 0;...
               self.ct/(self.mt/self.gravity) self.kt/(self.mt/self.gravity);...
               0 0];
           
           F = [0; 0; -self.gravity; 0];
           
           ssmodel_off = {A; B; F};           
        end
        
        function ssout = get.ssout(self)
           C = @(t) [0 1 0 0;... % bridge displacement
               1 0 0 0;... % bridge velocity
               0 0 0 1;...  % vehicle displacement
               0 0 1 0;...    % vehicle velocity
               self.ct*sin(pi*self.vel*t/self.L) self.kt*sin(pi*self.vel*t/self.L) -self.ct -self.kt]; % contact force
           C2 = [0 1 0 0;...
               1 0 0 0;...
               0 0 0 1;...
               0 0 1 0;...
               0 0 -self.ct -self.kt];% contact force
           
           D = zeros(size(C,1),2);
           D(5,:) = [self.ct self.kt];
           
%            F = [0; 0; 0; 0; -self.mt];
           
           ssout = {C; C2; D};
        end
	end % /dependent

%% static methods
	methods (Static)
	end % /static

%% protected methods
	methods (Access = protected)
	end % /protected

end
