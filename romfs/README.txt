gm = require "gmlan"
setup()
setupForFlash();flashFile("/rom/immo.bin",0xfedf1000)
setupForFlash();flashFile("/rom/clear.bin",0xfedf1000)
