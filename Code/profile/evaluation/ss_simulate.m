function response = ss_simulate(ss_model, X)
%% ss_simulate
% 
% 
% 
% author: John Braley
% create date: 2018-12-06 12:20:41.591
% set aside starting parameter values
mb_initial = ss_model.mb;
EI_intial = ss_model.EI;
% update parameter values using adjustment factors
ss_model.mb = mb_initial*X(1);
ss_model.EI = EI_intial*X(2);
% simulate with new parameter values
responses=ss_model.simulate;
static_disp = min(responses(:,6));
response = responses(ss_model.window_report(1):ss_model.window_report(1),1);
% reset model object parameter values to initial values
ss_model.mb = mb_initial;
ss_model.EI = EI_intial;
	
	
end
