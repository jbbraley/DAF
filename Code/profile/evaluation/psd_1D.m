function [q , C] = psd_1D(z, PixelWidth, dim)
% calculates averaged 1D PSD along x or y, for a surface height profile or 
% simply for multiple 1D line profiles.
% It is better to remove mean and tilt in your topography/1D line profile,
% before applying this function

% examples:
% [q , C] = psd_1D(z, PixelWidth, 'x')
% [q , C] = psd_1D(z, PixelWidth, 'y')
% ===
% inputs:

% z: height topography (n*m) a matrix of n*m size (SI units, i.e. meters)
% or simply multiple 1D line profiles saved in a matrix; in the latter
% case, be careful on the input "dim" (explained below) which shows where
% your profiles are (in x direction or y direction)

% PixelWidth: size of each Pixel in topography/profile (SI units, i.e.
% meters). If you don't know your pixelwidth just devide topography/profile length by
% number of pixels in length.

% dim: direction of 1D PSD; 'x' or 'y':
% 'x' takes profiles that are parallel to x ; it means it assumes your
% height profiles are z(1,:), z(2,:),...., z(n,:)
% 'y' takes profiles that are parallel to y ; it means it assumes your
% height profiles are z(:,1), z(:,2),...., z(:,m)
% Note That : [n , m ] = size (z);

% ===
% outputs:
% q: wavevectors, which is 2pi/lambda. lambda is wavelength of your
% roughness components. If you are not familiar with concept, refer to the
% appendix in below article to get insight into calculation procedure:
% "Persson, B. N. J., et al. "On the nature of surface roughness with application 
% to contact mechanics, sealing, rubber friction and adhesion." Journal of Physics: 
% Condensed Matter 17.1 (2005): R1."

% C: 1D PSD averaged for multiple line profiles

% in order to plot final result just use:
% loglog(q,C)

% =========================================================================
% Check number of inputs
if nargin ~=3
    error('The code requires exactly 3 inputs. Check code description.');
end
% =========================================================================
if strcmp(dim ,'y')
    z = z';
end

[n,m] = size(z);
a = PixelWidth; % lattice spacing

% =========================================================================
% Window function just in one direction
win = ones(n,1) * (1 - (((0:m-1)-((m-1)/2))/((m+1)/2)).^2); % Welch
% win = ones(n,1) * bartlett(m)';
% win = ones(n,1) * tukeywin(m,0.1)';
z_win = z.* win;

% =========================================================================
% Normalization factor (due to window function)
U = sum(win(1,:).^2)/((m-1));

% =========================================================================
% Calculate 1D PSD
Hm = fft(z_win , [] , 2); % fft of all rows
Cq = (1/U).*((a/m)*(1/(2*pi))).*((abs(fftshift(Hm,2))).^2);

% =========================================================================
% corresponding wavevectors to Cq values after fftshift has been applied
q_1=zeros(m,1);
for k=0:m-1
    q_1(k+1)=(2*pi/m)*(k);
end
q_2 = fftshift(q_1);
q_3 = unwrap(q_2-2*pi);
q = q_3/a;


C = mean(Cq , 1)';
