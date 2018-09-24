dofile("/rom/carseclib.lua")

print("Start SLCAN")
while true do
	line = io.read("*line")
	sendSlcan(line)
end
