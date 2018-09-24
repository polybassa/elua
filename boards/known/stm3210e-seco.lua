-- STM3210E-EVAL build configuration

return {
  cpu = 'stm32f103re',
  components = {
    sercon = { uart = 0, speed = 115200, buf_size = 512},
    wofs = true,
    romfs = true,
    shell = true,
    term = { lines = 25, cols = 80 },
    cints = true,
    luaints = true,
    linenoise = { shell_lines = 10, lua_lines = 50 },
    stm32_enc = false,
    rpc = false,
    adc = false,
    xmodem = true,
    mmcfs = false
  },
  config = {
    egc = { mode = "alloc" },
    vtmr = { num = 4, freq = 10 },
  },
  modules = {
    generic = { 'all', "-i2c", "-net", "-spi", "-pwm", "-rpc", "-fs", "-adc" },
    platform = { 'all', "-enc" }
  },
}                            
