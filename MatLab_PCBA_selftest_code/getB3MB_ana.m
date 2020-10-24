function [val2] = getB3MB_ana(canobj, vars, samps, dly)
%% ***************************************************************
% Retrieve 'samps' samples of first 'vars' B3MB Analog readings. 
% with 'dly' delay between samples. 
% try - catch protected version; will try for 1.5 sec (3 tries)
% instead of passing # of tries. 
%

if vars > 23 
    fprintf('\n ERROR : Number of Variables exceeds Maximum \n\n');
    val2 = 0;
    return;
end

tries = 3; % hard coded, but can be passed in 
attempt = 0; % try number
B3MB_MON_ADDR = 32;  % 0x20 ; B3MB base addr for analog monitors
B3MB_CMD_ADDR = 64; % 0x40 
val = zeros(vars, samps);

for ii = 1:samps
  while(attempt < tries)
    try
      val(:,ii) = canobj.SBCAN_read_inc(1, vars, B3MB_MON_ADDR);
      break
    catch MExc
      disp(MExc);
    end
    attempt = attempt + 1;
    warning('Re-trying sample %u',ii);
    pause(0.5)
  end
  if attempt == tries 
    B3MB_cmd(1, B3MB_CMD_ADDR, B3MB_ALL_LOADS_OFF, 3); % B3MB_ALL_LOADS_OFF cmd. 3 tries. 
    error('Read Error: Exceeded 3 tries. Make sure ALL LOADS are OFF!')
    return % Should never get here
  end
  pause(dly);
end

%% ***************************************************************
% Convert to meaningfull units based on hardware configuration
%
VFS = 1.024;      % Full Scale Voltage, +/-
LSB = VFS/2^15;   % LSB value(only 15 bits because of Single Ended config
Rbt = 200e3;      % Top R    in Vmonitor R divider
Rdv = 2e3;        % Bottom R in Vmonitor R divider
Rsh = 1.5e-3;     % Current Sensing Shunt Resistance
Ign = 20;         % Gain on Ish*Rsh internal to LTC7000 Chip
ILS = LSB/(Ign*Rsh); % Current LSB
% Vref = 2.500;     % Thermistor Reference Voltage
VTfs = 4.096;     % Termistor channel Full Scale Voltage +/-
VTLSB = VTfs/2^15; % Thermistor channel LSB

% Re-order and convert units
val2( 1: 4,:) = val( 1:2: 7,:)*LSB*(Rbt+Rdv)/Rdv;   % in volts at Vbatt(x)
val2( 5: 8,:) = val( 9:2:15,:)*LSB*(Rbt+Rdv)/Rdv;   % in volts at Vload(x)
val2( 9:12,:) = val( 2:2: 8,:)*ILS;                 % in amps  at Ibatt(x)
val2(13:16,:) = val(10:2:16,:)*ILS;                 % in amps  at Iload(x)
% Adding third i2c bus
val2(17, :) = val(17, :)*VTLSB*(Rbt+Rdv)/Rdv;       % in volts at Vbus
val2(18:22, :) = val(18:22, :)*VTLSB;   % in volts at Thermistor channels
val2(23, :) = val(23, :);

pause(0.01)
end

