import rospy
from pymodbus.client.sync import ModbusTcpClient
from pymodbus.register_read_message import ReadInputRegistersResponse
from math import ceil
import time
import threading

class communication:

   def __init__(self):
      self.client = None
      self.lock = threading.Lock()
      
   def connectToDevice(self, address):
      """Connection to the client - the method takes the IP address (as a string, e.g. '192.168.1.11') as an argument."""
      self.client = ModbusTcpClient(address)

   def disconnectFromDevice(self):
      """Close connection"""
      self.client.close()

   def sendCommand(self, data):   
      """Send a command to the Gripper - the method takes a list of uint8 as an argument. The meaning of each variable depends on the Gripper model (see support.robotiq.com for more details)"""
      #make sure data has an even number of elements   
      if(len(data) % 2 == 1):
         data.append(0)

      #Initiate message as an empty list
      message = []

      #Fill message by combining two bytes in one register
      for i in range(0, len(data)//2):
         message.append((data[2*i] << 8) + data[2*i+1])

      #To do!: Implement try/except
      with self.lock:
         self.client.write_registers(0, message)

   def getStatus(self, numBytes):
      """Sends a request to read, wait for the response and returns the Gripper status. The method gets the number of bytes to read as an argument"""
      numRegs = int(ceil(numBytes/2.0))

      #To do!: Implement try/except 
      #Get status from the device
      with self.lock:
         response = self.client.read_input_registers(0, numRegs)

      #Instantiate output as an empty list
      output = []

      #Fill the output with the bytes in the appropriate order
      for i in range(0, numRegs):
         output.append((response.getRegister(i) & 0xFF00) >> 8)
         output.append( response.getRegister(i) & 0x00FF)
      
      #Output the result
      return output

address = "192.168.0.111"
global counter
counter = 1

rospy.init_node('gripper_sever', anonymous=True)

def main():
    global counter
    if counter == 1:
        sever = communication()
        sever.connectToDevice(address)
        command = bytearray(b'\x09\x10\x03\xE8\x00\x03\x06\x01\x00\x00\x00\x00')
        sever.sendCommand(command)
        sever.disconnectFromDevice()
        counter += 1

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
