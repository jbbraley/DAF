classdef sgl_bridge_vehicle < vb_base
%% classdef ss_bridge_vehicle
% 
% 
% 
% author: John Braley
% create date: 2018-07-26 05:43:45.258
% classy version: 0.1.2

%% object properties
	properties 
        window_report   % portion of response time history to consider
	end

%% dependent properties
	properties (Dependent)
        ssmodel
        ssmodel_off
        ssout
        max_deflection
        DL_disp        
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
            max_deflection = self.mt*self.L^3/(48*self.EI);
        end
        
        function DL_disp = get.DL_disp(self)
            DL_disp = 5*self.mb*self.L^3/(384*self.EI);
        end
               
       function ssmodel = get.ssmodel(self)
           %%  contains the matrices A & B for input into the ssfun
           A = @(t) [-2*self.ct/(self.mb/self.gravity)*(sin(pi*self.vel*t/self.L))^2-self.db*(2*pi*self.fnb) -2*self.kt/(self.mb/self.gravity)*(sin(pi*self.vel*t/self.L))^2-pi^4*self.EI/(self.mb/self.gravity*self.L^3) 2*self.ct/(self.mb/self.gravity)*(sin(pi*self.vel*t/self.L)) 2*self.kt/(self.mb/self.gravity)*(sin(pi*self.vel*t/self.L));...
               1 0 0 0;...
               self.ct/(self.mt/self.gravity)*sin(pi*self.vel*t/self.L) self.kt/(self.mt/self.gravity)*sin(pi*self.vel*t/self.L) -self.ct/(self.mt/self.gravity) -self.kt/(self.mt/self.gravity);...
               0 0 1 0];
           B = @(t) [-2*self.ct/(self.mb/self.gravity)*sin(pi*self.vel*t/self.L) -2*self.kt/(self.mb/self.gravity)*sin(pi*self.vel*t/self.L);... 
               0 0;...
               self.ct/(self.mt/self.gravity) self.kt/(self.mt/self.gravity);...
               0 0];
           
           F = [0; 0; -self.gravity; 0];
           
           ssmodel = {A; B; F};
        end
        
        function ssmodel_off = get.ssmodel_off(self)
           %% contains matrices for input into ssfun for when the vehicle is not on the bridge (i.e. before or after)
           A = [-self.db*(2*pi*self.fnb) -pi^4*self.EI/(self.mb/self.gravity*self.L^3) 0 0;...
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
