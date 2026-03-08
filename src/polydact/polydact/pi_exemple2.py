from gpiozero import MCP3008
import time
adc = [MCP3008(channel=i) for i in range(8)]

while True:
    #for channel in adc:
    #    print(channel.value)	
    print(adc[0].value)
    time.sleep(.05)
