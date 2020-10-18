function [val2] = getB3MB_ana(canobj, vars, samps, dly)
%% ***************************************************************
% Get some data under current conditions
% Scan through all Loads, 
%

if vars > 23 
    fprintf('\n ERROR : Number of Variables exceeds Maximum \n\n');
    val2 = 0;
    return;
end
B3MB_MON_ADDR = 32;           % start of cache addr for analog monitor

val = zeros(vars, samps);
for ii = 1:samps
  val(:,ii) = canobj.SBCAN_read_inc(1, vars, B3MB_MON_ADDR);
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
Vref = 2.500;     % Thermistor Reference Voltage
VTfs = 4.096;     % Termistor channel Full Scale Voltage +/-
VTLSB = VTfs/2^15; % Thermistor channel LSB

%calm = [ 0.9355,  0.9377,  0.9502,  0.9520, 1.0, 1.0,  0.8991, 0.8838];
%calb = [-0.6444, -0.0622, -0.6491, -0.1496, 0.0, 0.0, -0.3370, 0.0812];

% Re-order and convert units
val2( 1: 4,:) = val( 1:2: 7,:)*LSB*(Rbt+Rdv)/Rdv;   % in volts at Vbatt(x)
val2( 5: 8,:) = val( 9:2:15,:)*LSB*(Rbt+Rdv)/Rdv;   % in volts at Vload(x)
val2( 9:12,:) = val( 2:2: 8,:)*ILS;                 % in amps  at Ibatt(x)
val2(13:16,:) = val(10:2:16,:)*ILS;                 % in amps  at Iload(x)
% Adding third i2c bus
val2(17, :) = val(17, :)*VTLSB*(Rbt+Rdv)/Rdv;         % in volts at Vbus
val2(18:22, :) = val(18:22, :)*VTLSB;               % in volts at Thermistor channels
val2(23, :) = val(23, :);

pause(0.01)

