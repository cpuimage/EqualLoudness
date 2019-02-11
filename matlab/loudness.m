
fs = 16000; %sampling frequency
//freqs = [  8000 11025 12000 16000 18900 22050 24000  28000 32000 36000 37800 44100 48000 ];
[Ay,By,Ab,Bb]=equalloudfilt(fs); %provides filter coeficents for a equal loudness filter and a 150hz high pass to simulate percieved loudness.


fprintf("By:%.14ff;\n",By)
fprintf("AY:%.14ff;\n",Ay)
fprintf("Bb:%.14ff;\n",Bb);
fprintf("Ab:%.14ff;\n",Ab);
