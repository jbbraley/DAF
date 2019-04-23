classdef vb_base < handle
%% classdef vb_base
% vehicle bridge object
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
        dt
        fnb  % bridge natural frequency (Hz)
        bridge_time
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
        
        function set.dt(self,value)
%             self.dt = value;
            % interpolate based on time step
            new_time = self.time(1):value:self.time(end);
            new_dist = new_time*self.vel+self.bridge_start;
            self.profile = interp1(self.dist,self.profile,new_dist)';
            self.dist = new_dist';               
        end
        
       
	end % /dependent

%% static methods
	methods (Static)
	end % /static

%% protected methods
	methods (Access = protected)
	end % /protected

end
