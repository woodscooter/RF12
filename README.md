RF12
====

Transceivers using the rf module from Hope RF.

The RF12 library has been derived from the JeeLib RF12 wireless driver, 
which is copyright © 2014 Jean-Claude Wippler under the MIT license.

This version is for small 8-bit PIC microprocessors.  The data format has been
changed, to include both source and destination addresses, allowing peer-to-peer
messaging with acknowledgement.  

RF12.c holds all the code for driving the RF12 module.
Tx2.c is for a simple battery-operated transceiver PCB with provision 
    for 2 push buttons and 2 LEDs
RxRly2.c is for a mains-powered PCB with a single relay switching the mains,
    with no isolation from mains voltage.
