%% Plotting HR
clear
clc

% Start Serial Communication, via UART
nRF52_DK = serialport('COM9', 115200);
fopen(nRF52_DK);

i = 1;
while(i < 12000) % 10000 samples is one minute

    % Obtain the characters from UART as doubles
    data(i) = str2double(regexp(readline(nRF52_DK),'\d+','match'));
    plot(data, 'r');
    i = i+1;
end

% Close the serial communication
clear nRF52_DK 

%% Plotting HR 2 Devices
clear
clc

nRF52_DK = serialport('COM9', 115200);
fopen(nRF52_DK);

i = 1;
j = 1;
a = 1;

while(i < 10000)
  s = (-1)^i;
  % In the UART terminal, the first value that appears is 
  % from device 1 (Wrist). Therefore, all values with an
  % odd index (ie, i=1,3,5,...) are from device 1. 
  if s == -1
   data(j)=str2double(regexp(readline(nRF52_DK),'\d+','match'));
   j = j+1;
   % All the values with an
   % even index (ie, i=2,4,6,...) are from device 2.
  else 
   data_2(a)=str2double(regexp(readline(nRF52_DK),'\d+','match'));
   a = a+1;
  end
  i = i+1;

end

figure (1)
plot(data,'r');
hold on
plot(data_2, 'b');
legend('Device 1 - Wrist','Device 2 - Finger')
xlabel('Samples (n)'), ylabel('Intensity of Absorved Light (I)')
hold on

clear nRF52_DK