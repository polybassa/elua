can.setup( 0, 500000 )

local i = 100

while i > 1 do
	id,typ,msg = can.recv(0)
	if id 
		then
			i = i - 1
			print(id, msg)
		end
	end

