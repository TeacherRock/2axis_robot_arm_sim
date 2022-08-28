%% Low Pass Filter
function y = lowp(x,f1,f3,rp,rs,Fs)
    % 低通濾波
    % 使用注意事項：通帶或阻帶的截止頻率的?取範圍是不能超?采?率的一半
    % 即 f1, f3 的值都要小於 Fs/2
    % x:需要低通濾波的序列
    % f1：通帶截止頻率
    % f3：阻帶截止頻率
    % rp：???衰?DB??置
    % rs：截止?衰?DB??置
    % FS：序列x的采??率
    % rp=0.1;rs=30;%通帶?衰?DB值和阻帶?衰?DB值
    % Fs %采頻率

    %   [N, Wp] = CHEB1ORD(Wp, Ws, Rp, Rs, 's') does the computation for an
    %   analog filter, in which case Wp and Ws are in radians/second.
    %
    %   % Example 1:
    %   %   For data sampled at 1000 Hz, design a lowpass filter with less than
    %   %   3 dB of ripple in the passband defined from 0 to 40 Hz and at least
    %   %   60 dB of ripple in the stopband defined from 150 Hz to the Nyquist
    %   %   frequency (500 Hz):
    %
    %   Wp = 40/500; Ws = 150/500;
    %   Rp = 3; Rs = 60;
    %   [n,Wp] = cheb1ord(Wp,Ws,Rp,Rs)  % Gives minimum order of filter
    %   [b,a] = cheby1(n,Rp,Wp);        % Chebyshev Type I filter

    wp = 2*pi*f1/Fs;
    ws = 2*pi*f3/Fs;
    % ??切比雪夫濾波器
    [n,wn] = cheb1ord(wp/pi,ws/pi,rp,rs);
    [bz1,az1] = cheby1(n,rp,wp/pi);

    y = filter(bz1,az1,x);    % ?序列 x 濾波後得到的序列 y
end

