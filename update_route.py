import serial


def int2bytes(n):
    byte = [chr(n >> i & 0xff) for i in (24, 16, 8, 0)]
    return bytes("".join(byte), 'ascii')


route_name = "153-BRD"
route = ["Berlin", "Bremen", "Wiesbaden", "Hannover", "Mainz", "Magdeburg"]
base_fare = 22
base_half_fare = 11
fare = [
    [22, 22, 22, 27, 37, 44],
    [22, 22, 22, 22, 24, 30],
    [22, 22, 22, 22, 22, 26],
    [27, 22, 22, 22, 22, 22],
    [37, 24, 22, 22, 22, 22],
    [44, 30, 26, 22, 22, 22],
]

half_fare = [
    [11, 11, 11, 14, 19, 22],
    [11, 11, 11, 11, 12, 15],
    [11, 11, 11, 11, 11, 13],
    [14, 11, 11, 11, 11, 11],
    [19, 12, 11, 11, 11, 11],
    [22, 15, 13, 11, 11, 11]
]

uart = serial.Serial('COM8', 9600, timeout=2000)

print("Hello")

uart.write(int2bytes(len(route)))     # send how many station on route to send
if uart.read(4) != int2bytes(len(route)):
    print("send number error")

uart.write(bytes(route_name.ljust(15, '\0'), 'ascii'))
if uart.read(15) != bytes(route_name.ljust(15, '\0'), 'ascii'):
    print("send route_name error")

for station in route:
    uart.write(bytes(station.ljust(16, '\0'), 'ascii'))
    if uart.read(16) != bytes(station.ljust(16, '\0'), 'ascii'):
        print(f"send {station} error")
print("route done")

uart.write(int2bytes(base_fare))
if uart.read(4) != int2bytes(base_fare):
    print(f"send {base_fare} error")

for row in fare:
    for val in row:
        uart.write(int2bytes(val))
        if uart.read(4) != int2bytes(val):
            print(f"send {val} error")
print("fare done")

uart.write(int2bytes(base_half_fare))
if uart.read(4) != int2bytes(base_half_fare):
    print(f"send {base_half_fare} error")

for row in half_fare:
    for val in row:
        uart.write(int2bytes(val))
        if uart.read(4) != int2bytes(val):
            print(f"send {val} error")
print("half fare done")

print("update route done")
uart.close()
