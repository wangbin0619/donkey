#!/usr/bin/env python3

import time
import ev3

my_ev3 = ev3.EV3(protocol=ev3.WIFI, host='00:16:53:44:23:17')
my_ev3.verbosity = 1

from ev3mail import *

for i in range(5):

    s = ev3mailbox.encodeMessage(MessageType.Numeric, 'abc', i)

    print(i)
    my_ev3.send_system_cmd(s,False)
    time.sleep(1)

    s = ev3mailbox.encodeMessage(MessageType.Numeric, 'def', i+100)
    my_ev3.send_system_cmd(s, False)




