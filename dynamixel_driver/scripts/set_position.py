#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
from optparse import OptionParser

from dynamixel_driver import dynamixel_io

if __name__ == '__main__':
    usage_msg = 'Usage: %prog [options] ID [On|Off]'
    desc_msg = 'Turns the specified Dynamixel servo motor.'
    epi_msg = 'Example, set position to 0: %s --port=/dev/ttyUSB1 --baud=57600 1 0' % sys.argv[0]
    
    parser = OptionParser(usage=usage_msg, description=desc_msg, epilog=epi_msg)
    parser.add_option('-p', '--port', metavar='PORT', default='/dev/ttyUSB0',
                      help='motors of specified controllers are connected to PORT [default: %default]')
    parser.add_option('-b', '--baud', metavar='BAUD', type="int", default=1000000,
                      help='connection to serial port will be established at BAUD bps [default: %default]')
                      
    (options, args) = parser.parse_args(sys.argv)
    
    if len(args) < 3:
        parser.print_help()
        exit(1)
        
    port = options.port
    baudrate = options.baud
    motor_id = int(args[1])
    position = int(args[2])

    try:
        dxl_io = dynamixel_io.DynamixelIO(port, baudrate)
    except dynamixel_io.SerialOpenError, soe:
        print 'ERROR:', soe
    else:
        print 'Setting motor %d to %d' % (motor_id, position)
        if dxl_io.ping(motor_id):
            dxl_io.set_position(motor_id, position)
            print 'done'
        else:
            print 'ERROR: motor %d did not respond. Make sure to specify the correct baudrate.' % motor_id

