#! /usr/bin/env python3
# -- coding: UTF-8 
import rospy
import serial
import struct,time
from std_msgs.msg import String,Int32

def cal_crc(byte_array):
    itemp = 0xFFFF
    for byte in byte_array:
        itemp ^= byte
        for _ in range(8):
            if itemp & 0x1:
                itemp >>= 1
                itemp ^= 0xA001
            else:
                itemp >>= 1
    return byte_array+bytes(format(itemp,'X').zfill(4),'utf-8')
def send_cmd(cmd):
    cmd=cal_crc(cmd)
    print('ADP1000 send:',cmd)
    com.write(cmd)
    return receive_cmd()
def receive_cmd():
    data=com.readline().decode('utf-8').strip()
    print('ADP1000 receive:',data)
    return data
def doMsg(msg):
    val=msg.data
    rospy.loginfo(f"ADP command:{val}")
    # if 'state' in cmd and 'check' in cmd:
    #     data=send_cmd(b'>01d')[4:6]
    #     print('ADP1000 state=',data)
    if val==0:
        send_cmd(b'>01G')
        while not rospy.is_shutdown() and send_cmd(b'>01g')[4:6]=='00':
            time.sleep(0.1)
            pass
        send_cmd(b'>01Q')
        while not rospy.is_shutdown() and send_cmd(b'>01q')[4:6]=='01':
            time.sleep(0.1)
            pass
    # elif 'set' in cmd:
    #     v=bytes(format(int(cmd[-6:-2]),'X').zfill(4),'utf-8')
    #     send_cmd(b'>01O'+v+b'1')
    elif 0<val<=5000:
        # v=bytes(format(1,'X').zfill(2),'utf-8')
        # data=send_cmd(b'>01N'+v)[4:6]
        # data=send_cmd(b'>01d')[4:6]
        # if data!='03':
        #     rospy.logwarn("ADP1000 suction without liquid")
        #     return
        v=bytes(format(val,'X').zfill(4),'utf-8')
        data=send_cmd(b'>01n'+v)[4:6]
        while not rospy.is_shutdown() and send_cmd(b'>01d')[4:6]=='00':
            time.sleep(0.1)
            pass
        data=send_cmd(b'>01d')[4:6]
        if data=='01':
            rospy.loginfo("ADP1000 successfully suck")
        elif data=='02':
            rospy.logwarn("ADP1000 suction volume exceeds")
        elif data=='06':
            rospy.logwarn("ADP1000 suction blockage")
        else:
            rospy.loginfo(f"ADP1000 sucktion result={data}")
    elif -5000<=val<0:
        # v=bytes(format(-val,'X').zfill(4),'utf-8')
        v=bytes(format(0,'X').zfill(4),'utf-8') #每次全部吐完
        data=send_cmd(b'>01p'+v)[4:6]
        while not rospy.is_shutdown() and send_cmd(b'>01d')[4:6]=='00':
            time.sleep(0.1)
            pass
        data=send_cmd(b'>01d')[4:6]
        if data=='01':
            rospy.loginfo("ADP1000 successfully expel")
        elif data=='02':
            rospy.logwarn("ADP1000 collision")
        elif data=='05':
            rospy.logwarn("ADP1000 expel volume exceeds")
        else:
            rospy.loginfo(f"ADP1000 expel result={data}")
def ADPtest(cmd):
    msg=String()
    msg.data=cmd
    doMsg(msg)
    time.sleep(1)

if __name__ == "__main__":
    rospy.init_node("adp1000")
    sub = rospy.Subscriber("/adp1000cmd",Int32,doMsg,queue_size=1)
    # com = serial.Serial('/dev/ttyUSB0', 115200)
    # com = serial.Serial('/dev/ADP', 115200)
    com = serial.Serial('/dev/RPGI', 115200)
    rospy.loginfo('ADP1000 launched')
    # time.sleep(2)
    doMsg(Int32(0))
    # doMsg(Int32(3000))
    # doMsg(Int32(-5000))

    # doMsg(Int32(-800))
    # time.sleep(1)
    # for i in range(9):
    #     doMsg(Int32(-1000))
    #     time.sleep(1)

    # doMsg(Int32(-1000))
    # time.sleep(1)
    # doMsg(Int32(-1000))
    # time.sleep(1)
    # doMsg(Int32(0))
    # print(send_cmd(b'>01k270710'))
    # data=send_cmd(b'>01p'+v)[4:6]
    # send_cmd(b'>01H03E81FF600FA0FFF1FA24')#吸液阈值FF60,吐液阈值0FA0
    # send_cmd(b'>01B0030')#吐液速度100uL/s=0064
    # send_cmd(b'>01b')
    # send_cmd(b'>0140030')#吸液速度100uL/s=0064
    # send_cmd(b'>015')
    # send_cmd(b'>01d')
    # send_cmd(b'>01G')
    # while not rospy.is_shutdown():
    #     print('[0]:init')
    #     print('[1]:suck')
    #     print('[2]:expel')
    #     op=int(input("Your operation:"))
    #     if op==0:
    #         ADPtest('init state')
    #     elif op==1:
    #         vol=int(input("volume(uL):"))
    #         ADPtest(f'suck {vol:04d}uL')
    #     elif op==2:
    #         vol=int(input("volume(uL):"))
    #         ADPtest(f'expel {vol:04d}uL')
    #     else:
    #         print('illegal op')

    # ADPtest('check state')
    # ADPtest('set TP 1000uL')
    # time.sleep(5)
    # ADPtest('suck 1000uL')
    # ADPtest('expel 300uL')
    # ADPtest('expel 300uL')
    # ADPtest('expel 400uL')
    # time.sleep(3)
    # ADPtest('init state')
    rospy.spin()
