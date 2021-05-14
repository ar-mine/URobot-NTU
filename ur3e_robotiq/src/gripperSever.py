# -*- coding: utf-8 -*-
import socket
import time
import sys
import struct
import threading
import signal

import rospy
from sensor_msgs.msg import JointState

# Class for robotiq controller
class RobotiqHand():
    def __init__(self):
        self.so = None
        self._cont = False
        self._sem = None
        self._heartbeat_th = None
        self._max_position = 255
        self._min_position = 0

    def _heartbeat_worker(self):
        while self._cont:
            self.status()
            time.sleep(0.5)

    def connect(self, ip, port):
        self.so = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.so.connect((ip, port))
        self._cont = True
        self._sem = threading.Semaphore(1)
        self._heartbeat_th = threading.Thread(target = self._heartbeat_worker)
        self._heartbeat_th.start()

    def disconnect(self):
        self._cont = False
        self._heartbeat_th.join()
        self._heartbeat_th = None
        self.so.close()
        self.so = None
        self._sem = None

    def _calc_crc(self, command):
        crc_registor = 0xFFFF
        for data_byte in command:
            tmp = crc_registor ^ data_byte
            for _ in range(8):
                if(tmp & 1 == 1):
                    tmp = tmp >> 1
                    tmp = 0xA001 ^ tmp
                else:
                    tmp = tmp >> 1
            crc_registor = tmp
        crc = bytearray(struct.pack('<H', crc_registor))
        return crc

    def send_command(self, command):
        with self._sem:
            crc = self._calc_crc(command)
            data = command + crc
            self.so.sendall(data)
            time.sleep(0.001)
            data = self.so.recv(1024)
        return bytearray(data)

    def status(self):
        command = bytearray(b'\x09\x03\x07\xD0\x00\x03')
        return self.send_command(command)

    def reset(self):
        command = bytearray(b'\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00')
        return self.send_command(command)

    def activate(self):
        command = bytearray(b'\x09\x10\x03\xE8\x00\x03\x06\x01\x00\x00\x00\x00\x00')
        return self.send_command(command)

    def wait_activate_complete(self):
        while True:
            data = self.status()
            if data[5] != 0x00:
                return data[3]
            if data[3] == 0x31 and data[7] < 4:
                return data[3]

    def adjust(self):
        self.move(255, 64, 1)
        (status, position, force) = self.wait_move_complete()
        self._max_position = position
        self.move(0, 64, 1)
        (status, position, force) = self.wait_move_complete()
        self._min_position = position

    def get_position_mm(self, position):
        if position > self._max_position:
            position = self._max_position
        elif position < self._min_position:
            position = self._min_position
        position_mm = 85.0 * (self._max_position - position) / (self._max_position - self._min_position)
        #print 'max=%d, min=%d, pos=%d pos_mm=%.1f' % (self._max_position, self._min_position, position, position_mm)
        return position_mm

    def get_force_mA(self, force):
        return 10.0 * force

    # position: 0x00...open, 0xff...close
    # speed: 0x00...minimum, 0xff...maximum
    # force: 0x00...minimum, 0xff...maximum
    def move(self, position, speed, force):
        #print('move hand')
        command = bytearray(b'\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\x00\x00')
        command[10] = position
        command[11] = speed
        command[12] = force
        return self.send_command(command)

    # result: (status, position, force)
    def wait_move_complete(self):
        while True:
            data = self.status()
            if data[5] != 0x00:
                return (-1, data[7], data[8])
            if data[3] == 0x79:
                return (2, data[7], data[8])
            if data[3] == 0xb9:
                return (1, data[7], data[8])
            if data[3] == 0xf9:
                return (0, data[7], data[8])

HOST = "192.168.0.111"
PORT = 54321

rospy.init_node('gripper_sever')

joint_pub = rospy.Publisher("joint_states", JointState, queue_size=1)
joint_state = JointState()
joint_state.name.append("finger_joint")
joint_state.position.append(0)

rate = rospy.Rate(50) # 50hz

def handler(signal, frame):
    exit(0)

def test():
    global HOST, PORT
    
    print('test_robotiq start')
    hand = RobotiqHand()
    hand.connect(HOST, PORT)

    try:
        print('activate: start')
        hand.reset()
        hand.activate()
        result = hand.wait_activate_complete()
        print('activate: result = 0x{:02x}'.format(result))
        if result != 0x31:
            hand.disconnect()
            return
        print('adjust: start')
        hand.adjust()
        print('adjust: finish')

        
        print('close slow')
        hand.move(255, 0, 1)
        (status, position, force) = hand.wait_move_complete()
        position_mm = hand.get_position_mm(position)
        force_mA = hand.get_force_mA(force)

        if status == 0:
            print('no object detected: position = {:.1f}mm, force = {:.1f}mA '.format(position_mm, force_mA))
        elif status == 1:
            print('object detected closing: position = {:.1f}mm, force = {:.1f}mA '.format(position_mm, force_mA))
            print('keeping')
            time.sleep(5)
        elif status == 2:
            print('object detected opening: position = {:.1f}mm, force = {:.1f}mA '.format(position_mm, force_mA))
        else:
            print('failed')

        print('open fast')
        hand.move(0, 255, 0)
        (status, position, force) = hand.wait_move_complete()
        position_mm = hand.get_position_mm(position)
        force_mA = hand.get_force_mA(force)
        print('position = {:.1f}mm, force = {:.1f}mA '.format(position_mm, force_mA))
    except:
        print('Ctrl-c pressed')

    hand.disconnect()

def main():
    global HOST, PORT 
    hand = RobotiqHand()
    hand.connect(HOST, PORT)

    try:
        print("Connect finish")
        print('activate: start')
        hand.reset()
        hand.activate()
        result = hand.wait_activate_complete()
        # print('activate: result = 0x{:02x}'.format(result))
        if result != 0x31:
            hand.disconnect()
            return
        # print('adjust: start')
        # hand.adjust()
        # print('adjust: finish')
        rospy.set_param("/gripperSever/gripper_pos", 0)


        print("Start publish...")
        while not rospy.is_shutdown():
            # update joint_state
            gripper_pos = rospy.get_param("/gripperSever/gripper_pos")
            gripper_pos = int(gripper_pos)
            hand.move(gripper_pos, 0, 1)
            (status, position, force) = hand.wait_move_complete()
            # print(gripper_pos)

            joint_state.header.stamp = rospy.Time.now()
            joint_state.name[0] = "finger_joint"
            joint_state.position[0] = gripper_pos*46*3.1415/180/255

            # send the joint state and transform
            joint_pub.publish(joint_state)
            
            rate.sleep()
            signal.signal(signal.SIGINT, handler)
    except:
        print('Ctrl-c pressed')

    hand.disconnect()

if __name__ == '__main__':
    main()
    
    


