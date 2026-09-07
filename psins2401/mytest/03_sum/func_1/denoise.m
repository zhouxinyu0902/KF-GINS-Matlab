%% 函数
function y=denoise(x,order,fc)
[b, a] = butter(order, fc, 'low');
y = filtfilt(b, a, x);
myfigurestartup(10,4,'prese');
subplot 121,freq(x),hold on,freq(y),
legend('raw','filtered')
subplot 122,plot(x),hold on,plot(y),
legend('raw','filtered')
xygo('dot','data')

function freq(x)
N = length(x);
NFFT = 2^nextpow2(N);
X = fft(x, NFFT);
fs = 2; % 采样率
f = (0:NFFT-1)*(fs/NFFT);
f = f(1:NFFT/2+1);
X = abs(X/NFFT);
plot(f, X(1:NFFT/2+1));
xlabel('Frequency (Hz)');
ylabel('|X(f)|');
title('Frequency Domain Representation');
grid on;
end
end