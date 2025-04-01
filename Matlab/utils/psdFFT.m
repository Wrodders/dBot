function [psdx,freq] = psdFFT(x,fs)
%psdFFT 
N = length(x);
W = hann(N);
xdft = fft(W.*x);
xdft = xdft(1:round((N/2)+1));
psdx = (1/(fs*N)) * abs(xdft).^2;
psdx(2:end-1) = 2*psdx(2:end-1);
freq = 0:fs/(N+1):fs/2;


end