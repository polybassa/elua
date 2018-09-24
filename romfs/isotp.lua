local isotp = {}

function isotp.setup()
	tmr.setclock(2, 483)
end

function isotp.delay( ms )
	local start_ms = bit.rshift(tmr.read(2), 2)
	local finish_ms = bit.band(start_ms + ms, bit.rshift(0xffff,2)) 	
	local now = start_ms
	if start_ms <= finish_ms then
		repeat
				now = bit.rshift(tmr.read(2), 2)
		until finish_ms <= now
	else
		repeat
				now = bit.rshift(tmr.read(2), 2)
		until (finish_ms < now) and (now < start_ms)
	
	end
end

local function isotp_send_sf(sid,data)
	local sfdata = string.char(string.len(data)) .. data
	can.send(0, sid, 0, sfdata)
end

local function isotp_send_ff(sid,data)
	can.send(0, sid, 0, data)
end

local fcReceived = false
local fcCanId = 0x00

local function canRecvFcHandler( resnum )
	local canid,_,msg = can.recv(0)
	if canid and canid == fcCanId then
		fcReceived = string.byte(msg, 1) == 0x30
	else
		can.interrupt(0,1) -- enable interrupt for next receive
	end
end

local function isotp_recv_fc_setup(did)
	can.filter(0,0,1,bit.lshift(did,5),bit.lshift(0x7ff,5))
	fcCanId = did
	fcReceived = false
	cpu.set_int_handler(can.INT_CAN_RX, canRecvFcHandler)
	can.interrupt(0,1)
end

local function isotp_recv_fc(did)
	local timeout_ms = 100
	for i=timeout_ms,1,-1 do
		if fcReceived then break end
		isotp.delay(1)
	end
	can.interrupt(0,0) -- disable interrupt
	can.filter(0,0,1,0,0) -- set filter back to default
	return fcReceived
end

local function isotp_send_cf(sid, data)
	local len = string.len(data)
	local counter = 1
	for i=1,len,7 do
		isotp.delay(1)
		local d = string.sub(data, i, math.min(i+6,len))
		can.send(0,sid,0, string.char(counter + 0x20) .. d)
		counter = bit.band((1 + counter), 0x0f)
	end
end

function isotp.send(sid,did,data)
	if string.len(data) <= 7 then
		isotp_send_sf(sid,data)
		return true
	end
	
	local ffdata = string.sub(data,1,6)
	local cfdata = string.sub(data,7)
	local framelen = string.len(data)
	local l1 = string.char(bit.rshift(framelen, 8) + 0x10)
	local l2 = string.char(bit.band(framelen, 0xff))

	isotp_recv_fc_setup(did)
	isotp_send_ff(sid, l1 .. l2 .. ffdata)

	if isotp_recv_fc(did) == false then
		print("No FC received")
		return false
	end

	isotp_send_cf(sid,cfdata)
	return true
end

local function isotp_recv_sf(msg)
	local len = tonumber(string.byte(msg, 1))
	if len > 7 then
		print("Error")
		return nil
	end
	return string.sub(msg,2,len + 1)
end

local function isotp_recv_sendFC(sid)
	can.send(0,sid,0,"\048\000\010") -- will send 30 00 0A
end

local isomsg = {}

local function canRecvCfHandler( resnum )
	local canid, _, msg = can.recv(0)
	if canid and canid == isomsg.did then
		local cfReceived = bit.band(string.byte(msg, 1), 0xf0) == 0x20
		if not cfReceived then
				can.interrupt(0,1)
				return
		end
		local counter = bit.band(string.byte(msg, 1), 0x0f)
		if counter ~= bit.band(isomsg.lastFrameCount + 1, 0xf) then
				isomsg.error = true
				return
		end
		isomsg.lastFrameCount = counter 
		isomsg.data = isomsg.data .. string.sub(msg, 2)
		isomsg.received = true
	end
	can.interrupt(0,1) -- enable interrupt for next receive
end

local function isotp_recv_cf_setup(did, msg)
	local len = bit.lshift(bit.band(string.byte(msg,1),0x0f), 8) + string.byte(msg,2)
	isomsg["did"] = did
	isomsg["len"] = len
	isomsg["lastFrameCount"] = 0
	isomsg["data"] = string.sub(msg,3)
	isomsg["error"] = false
	isomsg["received"] = false
	cpu.set_int_handler(can.INT_CAN_RX, canRecvCfHandler)
	can.interrupt(0,1)
end

local function isotp_recv_cf(sid,did,msg)
	isotp_recv_cf_setup(did,msg)
	isotp_recv_sendFC(sid)

	local timeout_ms = 100
	for i=timeout_ms,1,-1 do
		if isomsg.len == string.len(isomsg.data) then
			can.interrupt(0,0)
			return isomsg.data
		end
		if isomsg.error then
				can.interrupt(0,0)
				return nil
		end
		if isomsg.received then
				i=timeout_ms
				isomsg.received = false
		end
		isotp.delay(1)
	end
	can.interrupt(0,0)
	return nil
end

function isotp.recv(sid,did)
	can.filter(0,0,1,bit.lshift(did,5),bit.lshift(0x7ff,5)) -- setup filter for did. Works only with standard identifiers because of lshift
	local timeout_ms = 200
	for i=timeout_ms,1,-1 do
		canid,_,msg = can.recv(0)
		if canid and canid == did then
			frametype = bit.rshift(bit.band(string.byte(msg, 1), 0xf0),4)
			if frametype == 0 then
				return isotp_recv_sf(msg)
			elseif frametype == 1 then
				return isotp_recv_cf(sid,did,msg)
			end
		end
		isotp.delay(1)
	end
	return nil
end

return isotp
