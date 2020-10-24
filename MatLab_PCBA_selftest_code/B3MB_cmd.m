function [ack] = B3MB_cmd(canobj, cmd, tries)
%% **************************************************
% Protected B3MB command
% Will attempt command until correct acknowledge, for 'tries' times 
% Can return one or two part vector:
% [ack, msg] = B3MB_cmd(..) ack, and Error msg, if handled at higher level
% [ack] returns subbus acknowledge only
% canobj = CAN device ID
% cmd = Command number (1 - 23)
% tries = Max number of attempts before error

if cmd > 23 
    error('Unrecognized command');
end

B3MB_CMD_ADDR = 64; % 0x40 
attempt = 0; % write_ack attempts

while(attempt < tries)
  try
    [ack, msg] = canobj.write_ack(1, B3MB_CMD_ADDR, cmd); % Should I not grab returned values here, so that an error happens?
    break
  catch MExc
    disp(MExc);
  end
  % increment attempts, and do something if error
  attempt = attempt + 1;
  warning('Re-trying B3MB Command %u', attempt);
  pause(0.5)
end
if attempt == tries
  error('Switch command Error: Too many failed attempts: %s', msg);
end
end