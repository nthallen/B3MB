function [Vana, cmd_status] = B3MB_sigcheck(canobj, reps)
%% ***************************************************************
% Will report B3MB analog vars in Vana, and Status bits on 
% cmd_status for 'reps' times every 1 second. (Max 10 min)
%

if reps >  600
   warning('/n Request longer than Max 10 minutes \n\n');
   return;
end

for ii = 1:reps
  Vana = getB3MB_ana(canobj, 23, 1, .1);
  cmd_status = dec2bin(canobj.SBCAN_read_addrs(1, 64)); 
  pause(1)
end

end

