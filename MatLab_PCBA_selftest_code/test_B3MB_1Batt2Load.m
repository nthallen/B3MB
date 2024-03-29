%% ********************************************************************
% Test B3MB 1 Batt 2 Load
%   1 x 2 Switch Matrix         1 Battery Packs to 2 Loads
%    8 Digital Control Points   1 Battery Packs +  2 Loads On/Off control
%   16 Analog Monitors          1 Battery Voltages + 1 Battery Currents +
%                               2 Load Voltages    + 2 Load Currentss
%
%   Test Procedure overview
%   For ii = 1:# of Test Conditions = 5       Vbatt / Rload = Itest[1:5]
%     Set Battery Voltage and Resistive load  condition ii
%     For jj = 1: # Of LOAD_jj      = 2
%       Move Resistive load to LOAD_jj
%       For kk = 1: # of BATT_kk    = 1
%         Set BATT_kk to Load_jj
%         get Num_Samps raw[ii, jj, kk, 1:Num_Samps]
%       end
%     end
%   end

%% ***********************************a*****************************
% Estabish CAN connection to B3MB PCBA
%
sbsl = subbusd_slcan;           % create the subbus_serial_CAN object
sbsl.close;                     % close its serial port
sbsl.open;                      % open its serial port
%
%
CAN_ID = 1; % Set as needed. Default is Test CAN_ID = 1

while(1)
  try
    fprintf(1, 'Attempting CAN ID %d\n', CAN_ID);
    board_ID = sbsl.SBCAN_read_addrs(CAN_ID,2); % board_ID
    if board_ID == 17
      fprintf(1, 'Good Connection with CAN ID %d! B3MB Board ID = %d\n', CAN_ID, board_ID);
  	  break
	else
      fprintf(1, 'CAN ID %d is not B3MB. Returned Board ID = %d\n', CAN_ID, board_ID);
    end
  catch MExc
    disp(MExc.message); % Uncomment to show 
	warning('No response from CAN ID %d\n', CAN_ID); 
  end
  prompt = 'Enter new CAN ID: \n';
  CAN_ID = input(prompt);  % Type in new CAN ID 
end

%% **********************************************************************
% Edited for Single address command word
% Array the commands for convience
% Turn All loads off and turn only BATT_1 ON
%
B3MB_CMD_ADDR = 64; % 0x40

B3MB_BATT1_OFF = 0;
B3MB_BATT1_ON = 1;
B3MB_BATT2_OFF = 2;
B3MB_BATT2_ON = 3;
B3MB_BATT3_OFF = 4;
B3MB_BATT3_ON = 5;
B3MB_BATT4_OFF = 6;
B3MB_BATT4_ON = 7;
B3MB_BATT_OFF = [B3MB_BATT1_OFF, B3MB_BATT2_OFF, B3MB_BATT3_OFF, B3MB_BATT4_OFF];
B3MB_BATT_ON = [B3MB_BATT1_ON, B3MB_BATT2_ON, B3MB_BATT3_ON, B3MB_BATT4_ON];

B3MB_ALL_LOADS_OFF = 16;
B3MB_ALL_LOADS_ON = 17;
B3MB_ALL_BATTS_OFF = 18;
B3MB_ALL_BATTS_ON = 19;

B3MB_cmd(sbsl, CAN_ID, B3MB_ALL_LOADS_OFF, 3); % B3MB_ALL_LOADS_OFF cmd. 3 tries. 
status = sbsl.SBCAN_read_addrs(CAN_ID, 64);
if status < 65280
  warning('\n One or More Load Switch Faults\n')
  pause
end
B3MB_cmd(sbsl, CAN_ID, B3MB_ALL_BATTS_ON, 3); % B3MB_ALL_BATTS_ON cmd. 3 tries. 
pause(0.5)
Vana = getB3MB_ana(sbsl, CAN_ID, 23, 20, .1);  % Take Zero-Current Voltage readings
Vzero = mean(Vana,2);
disp(Vzero); % Display for log

B3MB_cmd(sbsl, CAN_ID, B3MB_BATT1_ON, 3); % B3MB_BATT1_ON cmd. 3 tries. 
B3MB_cmd(sbsl, CAN_ID, B3MB_BATT2_OFF, 3); % B3MB_BATT2_OFF cmd. 3 tries. 
B3MB_cmd(sbsl, CAN_ID, B3MB_BATT3_OFF, 3); % B3MB_BATT3_OFF cmd. 3 tries. 
B3MB_cmd(sbsl, CAN_ID, B3MB_BATT4_OFF, 3); % B3MB_BATT4_OFF cmd. 3 tries. 
pause(0.5)

%% *******************************************************************
% Define Input Voltage and Output Loading Conditions
% and test / data Collection parameters

% For 28V Battery Can
%Vin = [28.8, 28.8, 28.8, 28.8, 28.8];
%Rld = [25.0, 15.0, 10.0, 6.0, 3.0];

% For 100V Battery Can
Vin = [100.8, 100.8, 100.8, 100.8, 100.8];
Rld = [35.0, 25.0, 20.0, 15.0, 10.0];

Icd = Vin./Rld;
JP  = [7, 8, 9, 10]; 

NUM_CONDS = length(Vin); 
NUM_BATTS = 1; % 28V: 4 BATTS 100V: 1 BATT, or # installed
NUM_LOADS = 2;
NUM_VARS  = 23; % all variables
NUM_SAMPS = 30;
NUM_TEMPS = 5;
% DELAY     = 0.078;
DELAY     = 0.090; % Increase to help prevent timeouts
% 5-D sample matrix: raw(Conds, Loads, Batts, Vars, Samps)
raw       = zeros(NUM_CONDS, NUM_LOADS, NUM_BATTS, NUM_VARS, NUM_SAMPS); 
% 4-D signal/noise matrix: signal(Conds, Loads, Batts, Vars)
signal    = zeros(NUM_CONDS, NUM_LOADS, NUM_BATTS, NUM_VARS);
noise     = signal;
Temps = zeros(NUM_TEMPS, NUM_CONDS * NUM_LOADS * NUM_BATTS); 
Ttime = 0;
thermRT = load('T30K.DAT');  % prepare global Therm R vs T table for Temp readings 
global Rtherm;
Rtherm = thermRT(:,2); % Thermistor resistance from the table
global Ttherm;
Ttherm = thermRT(:,1); % Temperature in Celcius from the table

%% **********************************************************
% Do the test here
%
for nc = 1:NUM_CONDS
  condSpec = '\nSet Vin to %3.1f and Set Rload to %2.1f\n';
  fprintf(condSpec, Vin(nc), Rld(nc))
  for nl = 1:NUM_LOADS
    fprintf('\nAttach Rload to connector J%d\n\n', JP(nl));
    fprintf('Hit any Key when ready \n');
    pause
    connSpec='Collecting Data for Condition %1.0f at connector J%1.0f\n';
    fprintf(connSpec, nc, JP(nl));
%  Trying Swithching Loads on after Batts
%    B3MB_cmd(sbsl, CAN_ID, B3MB_ALL_LOADS_ON, 3); % B3MB_ALL_LOADS_ON cmd. 3 tries. 
%    pause(0.5)
    for nb = 1:NUM_BATTS
      B3MB_cmd(sbsl, CAN_ID, B3MB_BATT_ON(nb), 3); % nb B3MB_BATT_ON cmd. 3 tries. 
      pause(0.5)
%  Trying Switching Loads on after Batts:
      B3MB_cmd(sbsl, CAN_ID, B3MB_ALL_LOADS_ON, 3); % B3MB_ALL_LOADS_ON cmd. 3 tries. 
      pause(0.5)
      while(true)
        try
          raw(nc,nl,nb,:,:) = getB3MB_ana(sbsl, CAN_ID, NUM_VARS, NUM_SAMPS, DELAY);
          break
        catch MExc
          disp(MExc);
        end
        B3MB_cmd(sbsl, CAN_ID, B3MB_ALL_LOADS_OFF, 3); % Turn All Loads off for safety. 3 tries. 
        warning('Re-trying BATT %u',nb);
        fprintf('\nHit any Key when ready\n');
        pause
        B3MB_cmd(sbsl, CAN_ID, B3MB_ALL_LOADS_ON, 3); % B3MB_ALL_LOADS_ON cmd. 3 tries. 
        pause(0.5)
      end
      B3MB_cmd(sbsl, CAN_ID, B3MB_ALL_LOADS_OFF, 3); % B3MB_ALL_LOADS_OFF cmd. 3 tries. 
      if nb < NUM_BATTS
        B3MB_cmd(sbsl, CAN_ID, B3MB_BATT_ON(nb+1), 3); % Turn on next Batt. 3 tries. 
      else
        B3MB_cmd(sbsl, CAN_ID, B3MB_BATT_ON(1), 3); % Re-start at Batt 1. 3 tries. 
      end
      pause(0.5)
      B3MB_cmd(sbsl, CAN_ID, B3MB_BATT_OFF(nb), 3); % nb B3MB_BATT_OFF cmd. 3 tries. 
%       Mean and STDev      
      signal(nc,nl,nb,:) = mean(raw(nc,nl,nb,:,:), 5);
      noise(nc,nl,nb,:) = std(raw(nc,nl,nb,:,:),0, 5); % w = 0 for default normalization
      Ttime = Ttime + 1;
      Temps(:, Ttime) = signal(nc,nl,nb,18:22);  % Append Temps to array vs time
      measSpec = 'At BATT %1.0f meas V = %3.2f volts, I = %2.3f amps\n';
      fprintf(measSpec, nb, signal(nc,nl,nb,nb), signal(nc,nl,nb,nb+8));
    end
% Don't think I need:    B3MB_cmd(sbsl, CAN_ID, B3MB_ALL_LOADS_OFF, 3); % B3MB_ALL_LOADS_OFF cmd. 3 tries. 
    pause(10)                           % let loads cool off
  end
end

% Ignoring nreads 
% sbsl.close % ?


%% *********************************************************************
% Plot some Measured vs. Actual Results
% 

% signal(Cnd, Ld, Bat, Var)
figure, hold on  % For all Conditions, Battery Voltage Error by Actual Vin
plot(Vin, signal(:, 1, 1, 1)-Vin', '-ro')
% plot(Vin, signal(:, 1, 2, 2)-Vin', '-rx')
% plot(Vin, signal(:, 1, 3, 3)-Vin', '-r^') % 
% plot(Vin, signal(:, 1, 4, 4)-Vin', '-rs') % 
plot(Vin, signal(:, 2, 1, 1)-Vin', '-bo')
% plot(Vin, signal(:, 2, 2, 2)-Vin', '-bx')
% plot(Vin, signal(:, 2, 3, 3)-Vin', '-b^') % 
% plot(Vin, signal(:, 2, 4, 4)-Vin', '-bs') % 
% plot(Vin, signal(:, 3, 1, 1)-Vin', '-go')
% plot(Vin, signal(:, 3, 2, 2)-Vin', '-gx')
% plot(Vin, signal(:, 3, 3, 3)-Vin', '-g^') % 
% plot(Vin, signal(:, 3, 4, 4)-Vin', '-gs') % 
% plot(Vin, signal(:, 4, 1, 1)-Vin', '-ko')
% plot(Vin, signal(:, 4, 2, 2)-Vin', '-kx')
% plot(Vin, signal(:, 4, 3, 3)-Vin', '-k^') % 
% plot(Vin, signal(:, 4, 4, 4)-Vin', '-ks') % 

hold off, grid on
title('Battery Voltage vs. Actual');
xlabel('Actual Voltage (V)'); ylabel('Measured Battery Voltage (V)');
legend('BATT1xLOAD1', 'BATT1xLOAD2', ...
       'Location','northwest');

% signal(Cnd, Ld, Bat, Var)
figure, hold on  % For all Conditions, Bus Voltage Error by Battery Vin
plot(signal(:, :, 1, 1), signal(:, :, 1, 17)-signal(:, :, 1, 1), '-ro')
% plot(signal(:, :, 2, 2), signal(:, :, 2, 17)-signal(:, :, 2, 2), '-bx')
% plot(signal(:, :, 3, 3), signal(:, :, 3, 17)-signal(:, :, 3, 3), '-g^')
% plot(signal(:, :, 4, 4), signal(:, :, 4, 17)-signal(:, :, 4, 4), '-ks')

hold off, grid on
title('Bus Voltage vs. Actual')
xlabel('Battery Voltage (V)'); ylabel('Measured Bus Voltage Error (V)');
legend('BATT1', ...
       'Location','northwest');

% signal(Cnd, Ld, Bat, Var)
figure, hold on  % For all Conditions, Load Voltage Error by Battery Voltage
plot(signal(:, 1, 1, 1), signal(:, 1, 1, 5)-signal(:, 1, 1, 1), '-ro')
% plot(signal(:, 1, 2, 2), signal(:, 1, 2, 5)-signal(:, 1, 2, 2), '-rx')
% plot(signal(:, 1, 3, 3), signal(:, 1, 3, 5)-signal(:, 1, 3, 3), '-r^') % 
% plot(signal(:, 1, 4, 4), signal(:, 1, 4, 5)-signal(:, 1, 4, 4), '-rs') % 
plot(signal(:, 2, 1, 1), signal(:, 2, 1, 6)-signal(:, 2, 1, 1), '-bo')
% plot(signal(:, 2, 2, 2), signal(:, 2, 2, 6)-signal(:, 2, 2, 2), '-bx')
% plot(signal(:, 2, 3, 3), signal(:, 2, 3, 6)-signal(:, 2, 3, 3), '-b^') % 
% plot(signal(:, 2, 4, 4), signal(:, 2, 4, 6)-signal(:, 2, 4, 4), '-bs') % 
% plot(signal(:, 3, 1, 1), signal(:, 3, 1, 7)-signal(:, 3, 1, 1), '-go')
% plot(signal(:, 3, 2, 2), signal(:, 3, 2, 7)-signal(:, 3, 2, 2), '-gx')
% plot(signal(:, 3, 3, 3), signal(:, 3, 3, 7)-signal(:, 3, 3, 3), '-g^') % 
% plot(signal(:, 3, 4, 4), signal(:, 3, 4, 7)-signal(:, 3, 4, 4), '-gs') % 
% plot(signal(:, 4, 1, 1), signal(:, 4, 1, 8)-signal(:, 4, 1, 1), '-ko')
% plot(signal(:, 4, 2, 2), signal(:, 4, 2, 8)-signal(:, 4, 2, 2), '-kx')
% plot(signal(:, 4, 3, 3), signal(:, 4, 3, 8)-signal(:, 4, 3, 3), '-k^') % 
% plot(signal(:, 4, 4, 4), signal(:, 4, 4, 8)-signal(:, 4, 4, 4), '-ks') % 

hold off, grid on
title('Load Voltage Error vs. Battery Voltage')
xlabel('Battery Voltage (V)'); ylabel('Measured Load Voltage Error (V)');
legend('BATT1xLOAD1', 'BATT1xLOAD2', ...
       'Location','eastoutside');


% *********************************************************************
% Plot Current Data
% 
% signal(Cnd, Ld, Bat, Var)
figure, hold on  % For all Conditions, Battery Current by Load Outputs
plot(signal(:, 1, 1, 5)./Rld', signal(:, 1, 1, 9),  '-ro')
% plot(signal(:, 1, 2, 5)./Rld', signal(:, 1, 2, 10), '-rx')
% plot(signal(:, 1, 3, 5)./Rld', signal(:, 1, 3, 11), '-r^')
% plot(signal(:, 1, 4, 5)./Rld', signal(:, 1, 4, 12), '-rs')
plot(signal(:, 2, 1, 6)./Rld', signal(:, 2, 1, 9),  '-bo')
% plot(signal(:, 2, 2, 6)./Rld', signal(:, 2, 2, 10), '-bx')
% plot(signal(:, 2, 3, 6)./Rld', signal(:, 2, 3, 11), '-b^')
% plot(signal(:, 2, 4, 6)./Rld', signal(:, 2, 4, 12), '-bs')
% plot(signal(:, 3, 1, 7)./Rld', signal(:, 3, 1, 9),  '-go') % 
% plot(signal(:, 3, 2, 7)./Rld', signal(:, 3, 2, 10), '-gx') % 
% plot(signal(:, 3, 3, 7)./Rld', signal(:, 3, 3, 11), '-g^') % 
% plot(signal(:, 3, 4, 7)./Rld', signal(:, 3, 3, 11), '-gs') % 
% plot(signal(:, 4, 1, 8)./Rld', signal(:, 4, 1, 9),  '-ko') % 
% plot(signal(:, 4, 2, 8)./Rld', signal(:, 4, 2, 10), '-kx') % 
% plot(signal(:, 4, 3, 8)./Rld', signal(:, 4, 3, 11), '-k^') % 
% plot(signal(:, 4, 4, 8)./Rld', signal(:, 4, 3, 11), '-ks') % 

hold off, grid on
title('Battery Current Measured vs. Actual');
xlabel('Actual Current (amps)'); ylabel('Measured Current (amps)');
legend('BATT1xLOAD1', 'BATT1xLOAD2', ...
       'Location','eastoutside');

% signal(Cnd, Ld, Bat, Var)
figure, hold on  % For all Conditions, Load Current by Battery Inputs
plot(signal(:, 1, 1, 5)./Rld', signal(:, 1, 1, 13), '-ro')
% plot(signal(:, 1, 2, 5)./Rld', signal(:, 1, 2, 13), '-rx')
% plot(signal(:, 1, 3, 5)./Rld', signal(:, 1, 3, 13), '-r^') % 
% plot(signal(:, 1, 4, 5)./Rld', signal(:, 1, 4, 13), '-rs') % 
plot(signal(:, 2, 1, 6)./Rld', signal(:, 2, 1, 14), '-bo')
% plot(signal(:, 2, 2, 6)./Rld', signal(:, 2, 2, 14), '-bx')
% plot(signal(:, 2, 3, 6)./Rld', signal(:, 2, 3, 14), '-b^') % 
% plot(signal(:, 2, 4, 6)./Rld', signal(:, 2, 4, 14), '-bs') % 
% plot(signal(:, 3, 1, 7)./Rld', signal(:, 3, 1, 15), '-go')
% plot(signal(:, 3, 2, 7)./Rld', signal(:, 3, 2, 15), '-gx')
% plot(signal(:, 3, 3, 7)./Rld', signal(:, 3, 3, 15), '-g^') % 
% plot(signal(:, 3, 4, 7)./Rld', signal(:, 3, 4, 15), '-gs') % 
% plot(signal(:, 4, 1, 8)./Rld', signal(:, 4, 1, 16), '-ko')
% plot(signal(:, 4, 2, 8)./Rld', signal(:, 4, 2, 16), '-kx')
% plot(signal(:, 4, 3, 8)./Rld', signal(:, 4, 3, 16), '-k^') % 
% plot(signal(:, 4, 4, 8)./Rld', signal(:, 4, 4, 16), '-ks') % 

hold off, grid on
title('Load Current Measured vs. Actual');
xlabel('Actual Current (amps)'); ylabel('Measured Current (amps)');
legend('BATT1xLOAD1', 'BATT1xLOAD2', ...
       'Location','eastoutside');

%% ********************************************************************
% Generate Current Calibration factors 
% 1st order (linear, y=mx+b) fit
%
% signal(Cnd, Ld, Bat, Var)
cal = zeros(NUM_BATTS+NUM_LOADS, 2);

pB1L1 = polyfit(signal(:, 1, 1, 5)./Rld',signal(:, 1, 1,  9), 1);
pB1L2 = polyfit(signal(:, 2, 1, 6)./Rld',signal(:, 2, 1,  9), 1);
% pB1L3 = polyfit(signal(:, 3, 1, 7)./Rld',signal(:, 3, 1,  9), 1);
% pB1L4 = polyfit(signal(:, 4, 1, 8)./Rld',signal(:, 4, 1,  9), 1);
%cal(1,:) = (pB1L1+pB1L2+pB1L3+pB1L4)/4;
cal(1,:) = (pB1L1+pB1L2)/2;

% pB2L1 = polyfit(signal(:, 1, 2, 5)./Rld',signal(:, 1, 2, 10), 1);
% pB2L2 = polyfit(signal(:, 2, 2, 6)./Rld',signal(:, 2, 2, 10), 1);
% pB2L3 = polyfit(signal(:, 3, 2, 7)./Rld',signal(:, 3, 2, 10), 1);
% pB2L4 = polyfit(signal(:, 4, 2, 8)./Rld',signal(:, 4, 2, 10), 1);
% cal(2,:) = (pB2L1+pB2L2+pB2L3+pB2L4)/4;

% pB3L1 = polyfit(signal(:, 1, 3, 5)./Rld',signal(:, 1, 3, 11), 1);
% pB3L2 = polyfit(signal(:, 2, 3, 6)./Rld',signal(:, 2, 3, 11), 1);
% pB3L3 = polyfit(signal(:, 3, 3, 7)./Rld',signal(:, 3, 3, 11), 1);
% pB3L4 = polyfit(signal(:, 4, 3, 8)./Rld',signal(:, 4, 3, 11), 1);
% cal(3,:) = (pB3L1+pB3L2+pB3L3+pB3L4)/4;

% pB4L1 = polyfit(signal(:, 1, 4, 5)./Rld',signal(:, 1, 4, 12), 1);
% pB4L2 = polyfit(signal(:, 2, 4, 6)./Rld',signal(:, 2, 4, 12), 1);
% pB4L3 = polyfit(signal(:, 3, 4, 7)./Rld',signal(:, 3, 4, 12), 1);
% pB4L4 = polyfit(signal(:, 4, 4, 8)./Rld',signal(:, 4, 4, 12), 1);
% cal(4,:) = (pB4L1+pB4L2+pB4L3+pB4L4)/4;

% signal(Cnd, Ld, Bat, Var)
cal(5,:) = polyfit(signal(:, 1, :, 5)./Rld',signal(:, 1, :, 13), 1);
cal(6,:) = polyfit(signal(:, 2, :, 6)./Rld',signal(:, 2, :, 14), 1);
% cal(7,:) = polyfit(signal(:, 3, :, 7)./Rld',signal(:, 3, :, 15), 1);
% cal(8,:) = polyfit(signal(:, 4, :, 8)./Rld',signal(:, 4, :, 16), 1);

%% *********************************************************************
% Correct the signal and replot
% Do for all 8 Current signals. 
%
sigcal = signal;
for ii=1:6  % Normally 8
  sigcal(:,:,:,ii+8) = (signal(:,:,:,ii+8)-cal(ii,2))./cal(ii,1);
end 

% signal(Cnd, Ld, Bat, Var)
% sigcal(Cnd, Ld, Bat, Var)
figure, hold on  % For all Conditions, Battery Current by Load Outputs
plot(signal(:, 1, 1, 5)./Rld', sigcal(:, 1, 1, 9),  '-ro')
plot(signal(:, 2, 1, 6)./Rld', sigcal(:, 2, 1, 9),  '-rx')
% plot(signal(:, 3, 1, 7)./Rld', sigcal(:, 3, 1, 9),  '-r^')%
% plot(signal(:, 4, 1, 8)./Rld', sigcal(:, 4, 1, 9),  '-rs')%
% plot(signal(:, 1, 2, 5)./Rld', sigcal(:, 1, 2, 10), '-bo')
% plot(signal(:, 2, 2, 6)./Rld', sigcal(:, 2, 2, 10), '-bx')
% plot(signal(:, 3, 2, 7)./Rld', sigcal(:, 3, 2, 10), '-b^')%
% plot(signal(:, 4, 2, 8)./Rld', sigcal(:, 4, 2, 10), '-bs')%
% plot(signal(:, 1, 3, 5)./Rld', sigcal(:, 1, 3, 11), '-go')
% plot(signal(:, 2, 3, 6)./Rld', sigcal(:, 2, 3, 11), '-gx')
% plot(signal(:, 3, 3, 7)./Rld', sigcal(:, 3, 3, 11), '-g^')%
% plot(signal(:, 4, 3, 8)./Rld', sigcal(:, 4, 3, 11), '-gs')%
% plot(signal(:, 1, 4, 5)./Rld', sigcal(:, 1, 4, 12), '-ko')
% plot(signal(:, 2, 4, 6)./Rld', sigcal(:, 2, 4, 12), '-kx')
% plot(signal(:, 3, 4, 7)./Rld', sigcal(:, 3, 4, 12), '-k^')%
% plot(signal(:, 4, 4, 8)./Rld', sigcal(:, 4, 4, 12), '-ks')%

hold off, grid on
title('Battery Calibrated Current Measured vs. Actual');
xlabel('Actual Current (amps)'); ylabel('Measured Current (amps)');
legend('BATT1xLOAD1', 'BATT1xLOAD2', ...
       'Location','eastoutside');

% signal(Cnd, Ld, Bat, Var)
% sigcal(Cnd, Ld, Bat, Var)
figure, hold on  % For all Conditions, Load Current by Battery Inputs
plot(signal(:, 1, 1, 5)./Rld', sigcal(:, 1, 1, 13), '-ro')
% plot(signal(:, 1, 2, 5)./Rld', sigcal(:, 1, 2, 13), '-rx')
% plot(signal(:, 1, 3, 5)./Rld', sigcal(:, 1, 3, 13), '-r^')%
% plot(signal(:, 1, 4, 5)./Rld', sigcal(:, 1, 4, 13), '-rs')%
plot(signal(:, 2, 1, 6)./Rld', sigcal(:, 2, 1, 14), '-bo')
% plot(signal(:, 2, 2, 6)./Rld', sigcal(:, 2, 2, 14), '-bx')
% plot(signal(:, 2, 3, 6)./Rld', sigcal(:, 2, 3, 14), '-b^')%
% plot(signal(:, 2, 4, 6)./Rld', sigcal(:, 2, 4, 14), '-bs')%
% plot(signal(:, 3, 1, 7)./Rld', sigcal(:, 3, 1, 15), '-go')
% plot(signal(:, 3, 2, 7)./Rld', sigcal(:, 3, 2, 15), '-gx')
% plot(signal(:, 3, 3, 7)./Rld', sigcal(:, 3, 3, 15), '-g^')%
% plot(signal(:, 3, 4, 7)./Rld', sigcal(:, 3, 4, 15), '-gs')%
% plot(signal(:, 4, 1, 8)./Rld', sigcal(:, 4, 1, 16), '-ko')
% plot(signal(:, 4, 2, 8)./Rld', sigcal(:, 4, 2, 16), '-kx')
% plot(signal(:, 4, 3, 8)./Rld', sigcal(:, 4, 3, 16), '-k^')%
% plot(signal(:, 4, 4, 8)./Rld', sigcal(:, 4, 4, 16), '-ks')%

hold off, grid on
title('Load Calibrated Current Measured vs. Actual');
xlabel('Actual Current (amps)'); ylabel('Measured Current (amps)');
legend('BATT1xLOAD1', 'BATT1xLOAD2', ...
       'Location','eastoutside');

%% *****************************************************************
% Calc Reading to Noise Ratio (vs. Full Scale Signal to Noise Ratio
%
% signal(Cnd, Ld, Bat, Var)
% sigcal(Cnd, Ld, Bat, Var)
noise = std(raw,0,5);
figure, hold on  % For all Conditions, Load Current SNR by Battery Inputs
plot(signal(:, 1, 1, 5)./Rld', sigcal(:, 1, 1, 13)./noise(:,1,1,13), '-ro')
% plot(signal(:, 1, 2, 5)./Rld', sigcal(:, 1, 2, 13)./noise(:,1,2,13), '-rx')
% plot(signal(:, 1, 3, 5)./Rld', sigcal(:, 1, 3, 13)./noise(:,1,3,13), '-r^')%
% plot(signal(:, 1, 4, 5)./Rld', sigcal(:, 1, 4, 13)./noise(:,1,4,13), '-rs')%
plot(signal(:, 2, 1, 6)./Rld', sigcal(:, 2, 1, 14)./noise(:,2,1,14), '-bo')
% plot(signal(:, 2, 2, 6)./Rld', sigcal(:, 2, 2, 14)./noise(:,2,2,14), '-bx')
% plot(signal(:, 2, 3, 6)./Rld', sigcal(:, 2, 3, 14)./noise(:,2,3,14), '-b^')%
% plot(signal(:, 2, 4, 6)./Rld', sigcal(:, 2, 4, 14)./noise(:,2,4,14), '-bs')%
% plot(signal(:, 3, 1, 7)./Rld', sigcal(:, 3, 1, 15)./noise(:,3,1,15), '-go')
% plot(signal(:, 3, 2, 7)./Rld', sigcal(:, 3, 2, 15)./noise(:,3,2,15), '-gx')
% plot(signal(:, 3, 3, 7)./Rld', sigcal(:, 3, 3, 15)./noise(:,3,3,15), '-g^')%
% plot(signal(:, 3, 4, 7)./Rld', sigcal(:, 3, 4, 15)./noise(:,3,4,15), '-gs')%
% plot(signal(:, 4, 1, 8)./Rld', sigcal(:, 4, 1, 16)./noise(:,4,1,15), '-ko')
% plot(signal(:, 4, 2, 8)./Rld', sigcal(:, 4, 2, 16)./noise(:,4,2,15), '-kx')
% plot(signal(:, 4, 3, 8)./Rld', sigcal(:, 4, 3, 16)./noise(:,4,3,15), '-k^')%
% plot(signal(:, 4, 4, 8)./Rld', sigcal(:, 4, 4, 16)./noise(:,4,4,15), '-ks')%

hold off, grid on
title('Load Calibrated Current SNR vs. Actual S');
xlabel('Actual Current (amps)'); ylabel('SNR');
legend('BATT1xLOAD1', 'BATT1xLOAD2', ...
       'Location','eastoutside');

%% *****************************************************************
% Temperature Readings vs ~Time
%

figure, hold on  % Plot Temperatures in time order
plot(Temps(1, :), '-b');
plot(Temps(2, :), '-r');
plot(Temps(3, :), '-c');
plot(Temps(4, :), '-y');
plot(Temps(5, :), '-g');

hold off, grid on
title('B3MB Temperatures');
xlabel('Sample (~Time)'); ylabel('Vtemp (°C)');
legend('Temp1', 'Temp2', 'Temp3', 'Temp4', 'Temp5', ...
       'Location','eastoutside');

