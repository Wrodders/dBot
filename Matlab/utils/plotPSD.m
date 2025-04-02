function plotPSD(psdx,freq)
%plotPSD
semilogx(freq, 10*log10(psdx)); grid on; 
axis("tight");

end