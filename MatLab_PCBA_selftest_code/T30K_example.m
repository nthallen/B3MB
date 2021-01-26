%%
t30k = load('T30K.DAT');
Rthtab = t30k(:,2); % Thermistor resistance from the table
Ttab = t30k(:,1); % Temperature in Celcius from the table
Rpu = 75e3; % Pullup resistance
Vref = 2.5; % Pullup voltage
%%
% So now if you read a voltage V:
% Rth will be the measured thermistor resistance
% T will be the measured temperature in Celcius
% We know V = Vref * ( Rth / (Rpu + Rth))
% Solving for Rth, we get:
V = ?; % measured voltage
Rth = (V/(Vref-V)) * Rpu;
T = interp1(Rthtab, Ttab, Rth);
