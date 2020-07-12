import sys
import os

import time
import serial
import serial.tools.list_ports as port_list

# CONSTANTS
timestamp = time.strftime("%Y%m%d-%H%M%S")
filename = 'SAVE - ' + timestamp + '.csv'
savepath = './saves'

def printnewsection():
    print("*---------------------------------------------*")

def sniffports():
    ports = list(port_list.comports())
    print("The following ports are available!")
    for p in ports:
        print (p)

def readoven(ser):
    print("Serial port opened, type CTL+C to exit!")
    print("Please wait until machine is in ready state, then press go")
 
    # WILL REQUIRE ADMIN RIGHTS! Best just to make your own one
    # but that's just my opinion :D
    # os.mkdir(savepath)

    completename = os.path.join(savepath, filename) 
    f = open(completename, "w")

    while (ser.isOpen()):
        buf = ser.readline().decode('ascii')
        buf = buf.rstrip('\r')
        buf = buf.rstrip('\n')
        if buf == 'COMPLETE':
            print("All done bud")
            ser.close()
        f.write(buf)
        print(buf)

def main():
    # This boi doesn't really need to be there, but it's usefulish?

    printnewsection()
    print()
    sniffports()
    print()
    printnewsection()
    print()

    try:
        sys.argv[1]
        port = sys.argv[1]
        
    except Exception as e:
	    print('Please add COMx as arguement!')
	    return 

    ser = serial.Serial(port, 115200)
    readoven(ser)   

if __name__ == '__main__':
    main()