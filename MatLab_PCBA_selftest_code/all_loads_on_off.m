function all_loads_on_off(canobj, state)

% Edited Single word encoded commands
B3MB_CMD_ADDR = 64; % 0x40
B3MB_ALL_LOADS_OFF_CMD = 16;
B3MB_ALL_LOADS_ON_CMD = 17;

if state == 0 
  canobj.write_ack(1, B3MB_CMD_ADDR, B3MB_ALL_LOADS_OFF_CMD);
elseif state == 1
  canobj.write_ack(1, B3MB_CMD_ADDR, B3MB_ALL_LOADS_ON_CMD);
else
  fprintf(' ** Invalid Command Value **');
end

end