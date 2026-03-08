import spidev


# Initialize SPI

spi = spidev.SpiDev()

spi.open(0, 0)

spi.max_speed_hz = 1350000


def read_adc(channel):
    if channel < 0 or channel > 7:
        raise ValueError('Channel must be 0-7')

    # SPI command: Start bit (1), single-ended (1), channel (3 bits), MSBF (0)
    adc = spi.xfer2([1, (8 + channel) << 4, 0])

    # Combine 10-bit result from response
    data = ((adc[1] & 3) << 8) + adc[2]

    # Convert to voltage (assuming 3.3V VREF)
    voltage = (data * 3.3) / 1023

    return data, voltage


while True:
    for channel in range(8):
        raw, volts = read_adc(channel)
        print(f'Channel {channel}: {raw}, {volts:.2f}V')
