%% ********************************************************************
% Test B3MB
%   4 x 4 Switch Matrix         4 Battery Packs to 4 Loads
%    8 Digital Control Points   4 Battery Packs +  4 Loads On/Off control
%   16 Analog Monitors          4 Battery Voltages + 4 Battery Currents +
%                               4 Load Voltages    + 4 Load Currentss
%
%   Test Procedure overview
%   For ii = 1:# of Test Conditions = 6       Vbatt / Rload = Itest[1:6]
%     Set Battery Voltage and Resistive load  condition ii
%     For jj = 1: # Of LOAD_jj      = 4
%       Move Resistive load to LOAD_jj
%       For kk = 1: # of BATT_kk    = 4
%         Set BATT_kk to Load_jj
%         get Num_Samps raw[ii, jj, kk, 1:Num_Samps]
%       end
%     end
%   end

%% ****************************************************************
% Estabish CAN connection to B3MB PCBA
%
sbsl = subbusd_slcan_litch2;    % create the subbus_serial_CAN object
sbsl.close;                     % close its serial port
sbsl.open;                      % open its serial port
val = sbsl.SBCAN_read_addrs(1,2); % board_ID
if val == 14
  fprintf(1, 'Good Connection! Exptd B3MB Board ID = rcvd id = %d\n', val);
else
  fprintf(1, 'Failed, Expected Board ID 14 for B3MB, received %d\n', val);
  while(1)
    pause(1)
  end
end

%% **********************************************************************
% Define Control Point Address Map, Array the addresses for convience
% Turn All loads off and turn BATT_1 ON
%
B3MB_BATT1_ADDR = 56;
B3MB_BATT2_ADDR = 57;
B3MB_BATT3_ADDR = 58;
B3MB_BATT4_ADDR = 59;
BATT=[B3MB_BATT1_ADDR, B3MB_BATT2_ADDR, ...
      B3MB_BATT3_ADDR, B3MB_BATT4_ADDR, 0];

all_loads_on_off(sbsl, 0);                    
sbsl.write_ack(1, B3MB_BATT1_ADDR, 1);
sbsl.write_ack(1, B3MB_BATT2_ADDR, 0);
sbsl.write_ack(1, B3MB_BATT3_ADDR, 0);
sbsl.write_ack(1, B3MB_BATT4_ADDR, 0);
pause(0.5)

%% *******************************************************************
% Define Input Voltage and Output Loading Conditions
% and test / data Collection parameters
%
Vin = [28, 50, 50, 100, 100];
Rld = [25.2, 25.2, 15.2, 15.2,  10.2];
Icd = Vin./Rld;
JP  = [7, 8, 9, 10]; 

NUM_CONDS = length(Vin); 
NUM_BATTS = 4;
NUM_LOADS = 4;
NUM_VARS  = 16;
NUM_SAMPS = 30;
% DELAY     = 0.078;
DELAY     = 0.100; % Increase to prevent timeouts
% 5-D sample matrix: raw(Conds, Loads, Batts, Vars, Samps)
raw       = zeros(NUM_CONDS, NUM_LOADS, NUM_BATTS, NUM_VARS, NUM_SAMPS); 
% 4-D signal/noise matrix: signal(Conds, Loads, Batts, Vars)
signal    = zeros(NUM_CONDS, NUM_LOADS, NUM_BATTS, NUM_VARS);
noise     = signal;

%% **********************************************************
% Do the test here
%
for nc = 1:NUM_CONDS
  condSpec = '\nSet Vin to %3.0f and Set Rload to %2.0f\n\n';
  fprintf(condSpec, Vin(nc), Rld(nc))
  for nl = 1:NUM_LOADS
    fprintf('Attach Rload to connector J%d\n\n', JP(nl));
    fprintf('Hit any Key when ready \n');
    pause
    connSpec='Collecting Data for Condition %1.0f at connector J%1.0f\n';
    fprintf(connSpec, nc, JP(nl));
    all_loads_on_off(sbsl, 1)           % turn all loads on, get some data
    pause(0.5)
    for nb = 1:NUM_BATTS
      sbsl.write_ack(1, BATT(nb), 1);
      pause(0.5)
      raw(nc,nl,nb,:,:) = getB3MB_ana(sbsl, NUM_VARS, NUM_SAMPS, DELAY);
      sbsl.write_ack(1, BATT(nb+1), 1);  % always want one BATT on
      pause(0.5)
      sbsl.write_ack(1, BATT(nb), 0);
      signal(nc,nl,nb,:) = mean(raw(nc,nl,nb,:,:), 5);
      noise(nc,nl,nb,:) = std(raw(nc,nl,nb,:,:),0, 5); % w = 0 for default normalization
 %    noise(nc,:,:,:)  =  std(raw(nc,:,:,:,:));     % ***** CHECK THIS ****
      measSpec = 'At BATT %1.0f meas V = %3.2f volts, I = %2.3f amps\n';
      fprintf(measSpec, nb, signal(nc,nl,nb,nb), signal(nc,nl,nb,nb+8));
    end
    all_loads_on_off(sbsl, 0)           % turn all loads off for safety
    pause(10)                           % let loads cool off
  end
end
sbsl.close


%% *********************************************************************
% Plot some Measured vs. Actual Results
%
% signal(Cnd, Ld, Bat, Var)
figure, hold on  % For all Conditions, Load Voltage by Actual Vin
plot(Vin, signal(:, 1, 1, 5), '-ro')
plot(Vin, signal(:, 2, 1, 6), '-rx')
plot(Vin, signal(:, 3, 1, 7), '-r+') % 
plot(Vin, signal(:, 4, 1, 8), '-rs') % 

plot(Vin, signal(:, 1, 2, 5), '-bo')
plot(Vin, signal(:, 2, 2, 6), '-bx')
plot(Vin, signal(:, 3, 2, 7), '-b+') % 
plot(Vin, signal(:, 4, 2, 8), '-bs') % 

plot(Vin, signal(:, 1, 3, 5), '-go')
plot(Vin, signal(:, 2, 3, 6), '-gx')
plot(Vin, signal(:, 3, 3, 7), '-g+') % 
plot(Vin, signal(:, 4, 3, 8), '-gs') % 

plot(Vin, signal(:, 1, 4, 5), '-ko')
plot(Vin, signal(:, 2, 4, 6), '-kx')
plot(Vin, signal(:, 3, 4, 7), '-k+') % 
plot(Vin, signal(:, 4, 4, 8), '-ks') % 

hold off, grid on
title('Load Voltage Measured vs. Actual')
legend('BATT1xLOAD1', 'BATT1xLOAD2', 'BATT1xLOAD3', 'BATT1xLOAD4', ...
       'BATT2xLOAD1', 'BATT2xLOAD2', 'BATT2xLOAD3', 'BATT2xLOAD4', ...
       'BATT3xLOAD1', 'BATT3xLOAD2', 'BATT3xLOAD3', 'BATT3xLOAD4', ...
       'BATT4xLOAD1', 'BATT4xLOAD2', 'BATT4xLOAD3', 'BATT4xLOAD4', ...
       'Location','northwest');
xlabel('Actual Voltage (V)'); ylabel('Measured Voltage (V)');

% These are pseudo-known currents, as it uses measured voltage. Maybe use
% setpoint voltage? 
% signal(Cnd, Ld, Bat, Var)
figure, hold on  % For all Conditions, Battery Current by Load Outputs
plot(signal(:, 1, 1, 5)./Rld', signal(:, 1, 1, 9),  '-ro')
plot(signal(:, 2, 1, 6)./Rld', signal(:, 2, 1, 9),  '-rx')
plot(signal(:, 3, 1, 7)./Rld', signal(:, 3, 1, 9),  '-r+') % 
plot(signal(:, 4, 1, 8)./Rld', signal(:, 4, 1, 9),  '-rs') % 

plot(signal(:, 1, 2, 5)./Rld', signal(:, 1, 2, 10), '-bo')
plot(signal(:, 2, 2, 6)./Rld', signal(:, 2, 2, 10), '-bx')
plot(signal(:, 3, 2, 7)./Rld', signal(:, 3, 2, 10), '-b+') % 
plot(signal(:, 4, 2, 8)./Rld', signal(:, 4, 2, 10), '-bs') % 

plot(signal(:, 1, 3, 5)./Rld', signal(:, 1, 3, 11), '-go')
plot(signal(:, 2, 3, 6)./Rld', signal(:, 2, 3, 11), '-gx')
plot(signal(:, 3, 3, 7)./Rld', signal(:, 3, 3, 11), '-g+') % 
plot(signal(:, 4, 3, 8)./Rld', signal(:, 4, 3, 11), '-gs') % 

plot(signal(:, 1, 4, 5)./Rld', signal(:, 1, 4, 12), '-ko')
plot(signal(:, 2, 4, 6)./Rld', signal(:, 2, 4, 12), '-kx')
plot(signal(:, 3, 4, 7)./Rld', signal(:, 3, 3, 11), '-k+') % 
plot(signal(:, 4, 4, 8)./Rld', signal(:, 4, 3, 11), '-ks') % 

hold off, grid on
title('Battery Current Measured vs. Actual');
legend('BATT1xLOAD1', 'BATT1xLOAD2', 'BATT1xLOAD3', 'BATT1xLOAD4', ...
       'BATT2xLOAD1', 'BATT2xLOAD2', 'BATT2xLOAD3', 'BATT2xLOAD4', ...
       'BATT3xLOAD1', 'BATT3xLOAD2', 'BATT3xLOAD3', 'BATT3xLOAD4', ...
       'BATT4xLOAD1', 'BATT4xLOAD2', 'BATT4xLOAD3', 'BATT4xLOAD4', ...
       'Location','northwest');
xlabel('Actual Current (amps)'); ylabel('Measured Current (amps)');

% These are pseudo-known currents, as it uses measured voltage. Maybe use
% setpoint voltage? 
% signal(Cnd, Ld, Bat, Var)
figure, hold on  % For all Conditions, Load Current by Battery Inputs
plot(signal(:, 1, 1, 5)./Rld', signal(:, 1, 1, 13), '-ro')
plot(signal(:, 1, 2, 5)./Rld', signal(:, 1, 2, 13), '-rx')
plot(signal(:, 1, 3, 5)./Rld', signal(:, 1, 3, 13), '-r+') % 
plot(signal(:, 1, 4, 5)./Rld', signal(:, 1, 4, 13), '-rs') % 

plot(signal(:, 2, 1, 6)./Rld', signal(:, 2, 1, 14), '-bo')
plot(signal(:, 2, 2, 6)./Rld', signal(:, 2, 2, 14), '-bx')
plot(signal(:, 2, 3, 6)./Rld', signal(:, 2, 3, 14), '-b+') % 
plot(signal(:, 2, 4, 6)./Rld', signal(:, 2, 4, 14), '-bs') % 

plot(signal(:, 3, 1, 7)./Rld', signal(:, 3, 1, 15), '-go')
plot(signal(:, 3, 2, 7)./Rld', signal(:, 3, 2, 15), '-gx')
plot(signal(:, 3, 3, 7)./Rld', signal(:, 3, 3, 15), '-g+') % 
plot(signal(:, 3, 4, 7)./Rld', signal(:, 3, 4, 15), '-gs') % 

plot(signal(:, 4, 1, 8)./Rld', signal(:, 4, 1, 16), '-ko')
plot(signal(:, 4, 2, 8)./Rld', signal(:, 4, 2, 16), '-kx')
plot(signal(:, 4, 3, 8)./Rld', signal(:, 4, 3, 16), '-k+') % 
plot(signal(:, 4, 4, 8)./Rld', signal(:, 4, 4, 16), '-ks') % 

hold off, grid on
title('Load Current Measured vs. Actual');
legend('BATT1xLOAD1', 'BATT2xLOAD1', 'BATT3xLOAD1', 'BATT4xLOAD1', ...
       'BATT1xLOAD2', 'BATT2xLOAD2', 'BATT3xLOAD2', 'BATT4xLOAD2', ...
       'BATT1xLOAD3', 'BATT2xLOAD3', 'BATT3xLOAD3', 'BATT4xLOAD3', ...
       'BATT1xLOAD4', 'BATT2xLOAD4', 'BATT3xLOAD4', 'BATT4xLOAD4', ...
       'Location','northwest');
xlabel('Actual Current (amps)'); ylabel('Measured Current (amps)');

%% ********************************************************************
% Generate Current Calibration factors 
% 1st order (linear, y=mx+b) fit
%
% signal(Cnd, Ld, Bat, Var)
cal = zeros(NUM_BATTS+NUM_LOADS, 2);

pB1L1 = polyfit(signal(:, 1, 1, 5)./Rld',signal(:, 1, 1,  9), 1);
pB1L2 = polyfit(signal(:, 2, 1, 6)./Rld',signal(:, 2, 1,  9), 1);
pB1L3 = polyfit(signal(:, 3, 1, 7)./Rld',signal(:, 3, 1,  9), 1);
pB1L4 = polyfit(signal(:, 4, 1, 8)./Rld',signal(:, 4, 1,  9), 1);
cal(1,:) = (pB1L1+pB1L2+pB1L3+pB1L4)/4;

pB2L1 = polyfit(signal(:, 1, 2, 5)./Rld',signal(:, 1, 2, 10), 1);
pB2L2 = polyfit(signal(:, 2, 2, 6)./Rld',signal(:, 2, 2, 10), 1);
pB2L3 = polyfit(signal(:, 3, 2, 7)./Rld',signal(:, 3, 2, 10), 1);
pB2L4 = polyfit(signal(:, 4, 2, 8)./Rld',signal(:, 4, 2, 10), 1);
cal(2,:) = (pB2L1+pB2L2+pB2L3+pB2L4)/4;

pB3L1 = polyfit(signal(:, 1, 3, 5)./Rld',signal(:, 1, 3, 11), 1);
pB3L2 = polyfit(signal(:, 2, 3, 6)./Rld',signal(:, 2, 3, 11), 1);
pB3L3 = polyfit(signal(:, 3, 3, 7)./Rld',signal(:, 3, 3, 11), 1);
pB3L4 = polyfit(signal(:, 4, 3, 8)./Rld',signal(:, 4, 3, 11), 1);
cal(3,:) = (pB3L1+pB3L2+pB3L3+pB3L4)/4;

pB4L1 = polyfit(signal(:, 1, 4, 5)./Rld',signal(:, 1, 4, 12), 1);
pB4L2 = polyfit(signal(:, 2, 4, 6)./Rld',signal(:, 2, 4, 12), 1);
pB4L3 = polyfit(signal(:, 3, 4, 7)./Rld',signal(:, 3, 4, 12), 1);
pB4L4 = polyfit(signal(:, 4, 4, 8)./Rld',signal(:, 4, 4, 12), 1);
cal(4,:) = (pB4L1+pB4L2+pB4L3+pB4L4)/4;

% signal(Cnd, Ld, Bat, Var)
cal(5,:) = polyfit(signal(:, 1, :, 5)./Rld',signal(:, 1, :, 13), 1);
cal(6,:) = polyfit(signal(:, 2, :, 6)./Rld',signal(:, 2, :, 14), 1);
cal(7,:) = polyfit(signal(:, 3, :, 7)./Rld',signal(:, 3, :, 15), 1);
cal(8,:) = polyfit(signal(:, 4, :, 8)./Rld',signal(:, 4, :, 16), 1);

%% *********************************************************************
% Correct the signal and replot
% Do for all 8 Current signals. 
%
sigcal = signal;
for ii=1:8
  sigcal(:,:,:,ii+8) = (signal(:,:,:,ii+8)-cal(ii,2))./cal(ii,1);
end 

% signal(Cnd, Ld, Bat, Var)
% sigcal(Cnd, Ld, Bat, Var)
figure, hold on  % For all Conditions, Battery Current by Load Outputs
plot(signal(:, 1, 1, 5)./Rld', sigcal(:, 1, 1, 9),  '-ro')
plot(signal(:, 2, 1, 6)./Rld', sigcal(:, 2, 1, 9),  '-rx')
plot(signal(:, 3, 1, 7)./Rld', sigcal(:, 3, 1, 9),  '-r+')
plot(signal(:, 4, 1, 8)./Rld', sigcal(:, 4, 1, 9),  '-rs')

plot(signal(:, 1, 2, 5)./Rld', sigcal(:, 1, 2, 10), '-bo')
plot(signal(:, 2, 2, 6)./Rld', sigcal(:, 2, 2, 10), '-bx')
plot(signal(:, 3, 2, 7)./Rld', sigcal(:, 3, 2, 10), '-b+')
plot(signal(:, 4, 2, 8)./Rld', sigcal(:, 4, 2, 10), '-bs')

plot(signal(:, 1, 3, 5)./Rld', sigcal(:, 1, 3, 11), '-go')
plot(signal(:, 2, 3, 6)./Rld', sigcal(:, 2, 3, 11), '-gx')
plot(signal(:, 3, 3, 7)./Rld', sigcal(:, 3, 3, 11), '-g+')
plot(signal(:, 4, 3, 8)./Rld', sigcal(:, 4, 3, 11), '-gs')

plot(signal(:, 1, 4, 5)./Rld', sigcal(:, 1, 4, 12), '-ko')
plot(signal(:, 2, 4, 6)./Rld', sigcal(:, 2, 4, 12), '-kx')
plot(signal(:, 3, 4, 7)./Rld', sigcal(:, 3, 4, 12), '-k+')
plot(signal(:, 4, 4, 8)./Rld', sigcal(:, 4, 4, 12), '-ks')

hold off, grid on
title('Battery Calibrated Current Measured vs. Actual');
legend('BATT1xLOAD1', 'BATT1xLOAD2', 'BATT1xLOAD3', 'BATT1xLOAD4', ...
       'BATT2xLOAD1', 'BATT2xLOAD2', 'BATT2xLOAD3', 'BATT2xLOAD4', ...
       'BATT3xLOAD1', 'BATT3xLOAD2', 'BATT3xLOAD3', 'BATT3xLOAD4', ...
       'BATT4xLOAD1', 'BATT4xLOAD2', 'BATT4xLOAD3', 'BATT4xLOAD4', ...
       'Location','northwest');
xlabel('Actual Current (amps)'); ylabel('Measured Current (amps)');

% signal(Cnd, Ld, Bat, Var)
% sigcal(Cnd, Ld, Bat, Var)
figure, hold on  % For all Conditions, Load Current by Battery Inputs
plot(signal(:, 1, 1, 5)./Rld', sigcal(:, 1, 1, 13), '-ro')
plot(signal(:, 1, 2, 5)./Rld', sigcal(:, 1, 2, 13), '-rx')
plot(signal(:, 1, 3, 5)./Rld', sigcal(:, 1, 3, 13), '-r+')
plot(signal(:, 1, 4, 5)./Rld', sigcal(:, 1, 4, 13), '-rs')

plot(signal(:, 2, 1, 6)./Rld', sigcal(:, 2, 1, 14), '-bo')
plot(signal(:, 2, 2, 6)./Rld', sigcal(:, 2, 2, 14), '-bx')
plot(signal(:, 2, 3, 6)./Rld', sigcal(:, 2, 3, 14), '-b+')
plot(signal(:, 2, 4, 6)./Rld', sigcal(:, 2, 4, 14), '-bs')

plot(signal(:, 3, 1, 7)./Rld', sigcal(:, 3, 1, 15), '-go')
plot(signal(:, 3, 2, 7)./Rld', sigcal(:, 3, 2, 15), '-gx')
plot(signal(:, 3, 3, 7)./Rld', sigcal(:, 3, 3, 15), '-g+')
plot(signal(:, 3, 4, 7)./Rld', sigcal(:, 3, 4, 15), '-gs')

plot(signal(:, 4, 1, 8)./Rld', sigcal(:, 4, 1, 16), '-ko')
plot(signal(:, 4, 2, 8)./Rld', sigcal(:, 4, 2, 16), '-kx')
plot(signal(:, 4, 3, 8)./Rld', sigcal(:, 4, 3, 16), '-k+')
plot(signal(:, 4, 4, 8)./Rld', sigcal(:, 4, 4, 16), '-ks')

hold off, grid on
title('Load Calibrated Current Measured vs. Actual');
legend('BATT1xLOAD1', 'BATT2xLOAD1', 'BATT3xLOAD1', 'BATT4xLOAD1', ...
       'BATT1xLOAD2', 'BATT2xLOAD2', 'BATT3xLOAD2', 'BATT4xLOAD2', ...
       'BATT1xLOAD3', 'BATT2xLOAD3', 'BATT3xLOAD3', 'BATT4xLOAD3', ...
       'BATT1xLOAD4', 'BATT2xLOAD4', 'BATT3xLOAD4', 'BATT4xLOAD4', ...
       'Location','northwest');
xlabel('Actual Current (amps)'); ylabel('Measured Current (amps)');

%% *****************************************************************
% Calc Reading to Noise Ratio (vs. Full Scale Signal to Noise Ratio
%
% signal(Cnd, Ld, Bat, Var)
% sigcal(Cnd, Ld, Bat, Var)
noise = std(raw,0,5);
figure, hold on  % For all Conditions, Load Current SNR by Battery Inputs
plot(signal(:, 1, 1, 5)./Rld', sigcal(:, 1, 1, 13)./noise(:,1,1,13), '-ro')
plot(signal(:, 1, 2, 5)./Rld', sigcal(:, 1, 2, 13)./noise(:,1,2,13), '-rx')
plot(signal(:, 1, 3, 5)./Rld', sigcal(:, 1, 3, 13)./noise(:,1,3,13), '-r+')
plot(signal(:, 1, 4, 5)./Rld', sigcal(:, 1, 4, 13)./noise(:,1,4,13), '-rs')

plot(signal(:, 2, 1, 6)./Rld', sigcal(:, 2, 1, 14)./noise(:,2,1,14), '-bo')
plot(signal(:, 2, 2, 6)./Rld', sigcal(:, 2, 2, 14)./noise(:,2,2,14), '-bx')
plot(signal(:, 2, 3, 6)./Rld', sigcal(:, 2, 3, 14)./noise(:,2,3,14), '-b+')
plot(signal(:, 2, 4, 6)./Rld', sigcal(:, 2, 4, 14)./noise(:,2,4,14), '-bs')

plot(signal(:, 3, 1, 7)./Rld', sigcal(:, 3, 1, 15)./noise(:,3,1,15), '-go')
plot(signal(:, 3, 2, 7)./Rld', sigcal(:, 3, 2, 15)./noise(:,3,2,15), '-gx')
plot(signal(:, 3, 3, 7)./Rld', sigcal(:, 3, 3, 15)./noise(:,3,3,15), '-g+')
plot(signal(:, 3, 4, 7)./Rld', sigcal(:, 3, 4, 15)./noise(:,3,4,15), '-gs')

plot(signal(:, 4, 1, 8)./Rld', sigcal(:, 4, 1, 16)./noise(:,4,1,15), '-ko')
plot(signal(:, 4, 2, 8)./Rld', sigcal(:, 4, 2, 16)./noise(:,4,2,15), '-kx')
plot(signal(:, 4, 3, 8)./Rld', sigcal(:, 4, 3, 16)./noise(:,4,3,15), '-k+')
plot(signal(:, 4, 4, 8)./Rld', sigcal(:, 4, 4, 16)./noise(:,4,4,15), '-ks')

hold off, grid on
title('Load Calibrated Current SNR vs. Actual S');
legend('BATT1xLOAD1', 'BATT2xLOAD1', 'BATT3xLOAD1', 'BATT4xLOAD1', ...
       'BATT1xLOAD2', 'BATT2xLOAD2', 'BATT3xLOAD2', 'BATT4xLOAD2', ...
       'BATT1xLOAD3', 'BATT2xLOAD3', 'BATT3xLOAD3', 'BATT4xLOAD3', ...
       'BATT1xLOAD4', 'BATT2xLOAD4', 'BATT3xLOAD4', 'BATT4xLOAD4', ...
       'Location','northwest');
xlabel('Actual Current (amps)'); ylabel('SNR');

