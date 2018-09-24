
local function canHandler( resnum )
	id, typ, msg = can.recv(0)
	ts = tmr.read(1) / tmr.getclock(1)
	print(ts, id, typ, msg)
	can.interrupt(0,1)
end

can.setup( 0, 500000 )

clk = tmr.setclock(1, 1)
cpu.set_int_handler(can.INT_CAN_RX, canHandler)
can.interrupt(0,1)

print("running for some while")

for i=1000000,1,-1 do a=1 end

print("finished")

