
C10 = 100e-6; % roughness coefficient
w = [1.5 2.0 2.5 3.0 3.5 4.0];

n_range = logspace(-3,2)';
psd = C10*(10*n_range).^(-w); 


ah = loglog(n_range,psd);
psd_plot = plotter('thesis_single');
loglog(psd_plot.ah,n_range,psd);
legend = [padarray('w = ',length(w)-1,'replicate','post') num2str(w',2)];

psd_plot.legend = legend;
psd_plot.xlabel = 'Spatial frequency (n) [cycles/m]';
psd_plot.ylabel = 'Displacement PSD (G_d) [m^3]';
psd_plot.refresh

psd_fun = @(n) C10*(10*n).^(-w);


