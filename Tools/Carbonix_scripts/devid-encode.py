# Encode an ArduPilot device ID

# prompt the user for the bus type
print("Bus types:")
print("0: UNKNOWN")
print("1: I2C")
print("2: SPI")
print("3: UAVCAN")
print("4: SITL")
print("5: MSP")
print("6: SERIAL")
print("7: QSPI")
bus_type = int(input("Enter the bus type (0-7): "))

# prompt the user for the bus number
bus = int(input("Enter the bus number (0-31): "))
address = int(input("Enter the address (0-255): "))
devtype = int(input("Enter the device type (0-255): "))
device_id = bus_type | (bus << 3) | (address << 8) | (devtype << 16)

print(f"Device ID: {device_id}")
