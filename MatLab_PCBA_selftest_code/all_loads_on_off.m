function all_loads_on_off(canobj, state)

B3MB_LOAD1_ADDR = 60;
B3MB_LOAD2_ADDR = 61;
B3MB_LOAD3_ADDR = 62;
B3MB_LOAD4_ADDR = 63;

canobj.write_ack(1, B3MB_LOAD1_ADDR, state);
canobj.write_ack(1, B3MB_LOAD2_ADDR, state);
canobj.write_ack(1, B3MB_LOAD3_ADDR, state);
canobj.write_ack(1, B3MB_LOAD4_ADDR, state);

end