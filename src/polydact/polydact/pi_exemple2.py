from gpiozero import MCP3008
import time

adc = [MCP3008(channel=i) for i in range(8)]

while True:
    for channel in [1, 2, 3]:
        print(f'{channel}: {adc[channel].value}')

    time.sleep(0.1)
