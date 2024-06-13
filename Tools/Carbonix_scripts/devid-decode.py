# Decode an ArduPilot device id

import argparse

parser = argparse.ArgumentParser(description='Decode a device id')
parser.add_argument('device_id', type=int, help='Device ID to decode')
args = parser.parse_args()

bus_types = {
    0: 'UNKNOWN',
    1: 'I2C',
    2: 'SPI',
    3: 'UAVCAN',
    4: 'SITL',
    5: 'MSP',
    6: 'SERIAL',
    7: 'QSPI',
}

device_id = args.device_id
bus_type = device_id & 0x7
bus = (device_id >> 3) & 0x1f
address = (device_id >> 8) & 0xff
devtype = (device_id >> 16) & 0xff

print(f"Device ID: {device_id}")
print(f"{bus_types[bus_type]}{bus}, address {address}, devtype {devtype}")
