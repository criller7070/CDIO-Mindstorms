import bluetooth
import os

sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
sock.bind(("", bluetooth.PORT_ANY))
sock.listen(1)
print("Listening...")

client, addr = sock.accept()
print("Connected")

while True:
    data = client.recv(1024)
    if not data:
        break
    if data == b"SOUND":
        os.system('beep')
    print(data)
