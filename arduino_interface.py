import serial

class ArduinoInterface:
  def __init__(self):
    self.interface = serial.Serial('/dev/ttyACM0', 57600)

  def send_info(self, axes):
    out = " ".join([str(x) for x in axes])
    self.interface.write(out.encode())
    #print("out: " + out)

  def recv_info(self):
    try:
      print(self.interface.readline().decode())
    except:
      pass

if __name__ == "__main__":
  ser = serial.Serial('/dev/ttyACM0', 9600)
  while(True):
    ser.write("1500 1500 1500 1500 toggleLED".encode())
