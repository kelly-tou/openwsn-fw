import sys
import os
if __name__=='__main__':
    here = sys.path[0]
    sys.path.insert(0, os.path.join(here, 'firmware','openos','projects','common'))# contains openwsn module

try:

    import oos_openwsn

    def callback():
       print "callback called"

    # create instance
    mote = oos_openwsn.OpenMote()
    print str(mote)
    
    # install some callbacks
    for i in range(81):
       mote.set_callback(i,callback)
    '''
    # call other methods
    print mote.bsp_timer_isr()
    #print mote.radio_isr_startFrame()
    #print mote.radio_isr_endFrame()
    print mote.radiotimer_isr_compare()
    print mote.radiotimer_isr_overflow()
    '''

except Exception as err:
    raw_input('ERROR: {0}'.format(err))