function delay( ms )
	start = tmr.read(1) / (tmr.getclock(1))
	
	repeat
		now = tmr.read(1) / (tmr.getclock(1))
	until (((start + ms) % 0xffff) / 1000) < now
end

function fromhex(str)
    return (str:gsub('..', function (cc)
        return string.char(tonumber(cc, 16))
    end))
end

function tohex(str)
    return (str:gsub('.', function (c)
        return string.format('%02X', string.byte(c))
    end))
end

function sendTesterPresentTwice()
	delay(50)
	can.send(0, 0x241, 0, fromhex("013E")) 
	delay(50)
	can.send(0, 0x241, 0, fromhex("013E"))
	delay(50)
end

function sendSlcan(str)
	i = string.sub(str, 1, 1)
	if i ~= 't' then
		return
	end

	id = string.sub(str, 2, 4)
	len = tonumber(string.sub(str, 5, 5), 10)
	can.send(0, tonumber(id, 16), 0, fromhex(string.sub(str, 6, 5 + 2 * len)))
end

can.setup( 0, 500000 )
clk = tmr.setclock(1, 1)

