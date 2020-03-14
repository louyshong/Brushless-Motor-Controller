#include "messages.h"

Mail<mail_t, 16> mail_box;

RawSerial pc(SERIAL_TX, SERIAL_RX);

void outputToTerminal (void) {
    while(true)
    {
        osEvent evt = mail_box.get();
        if (evt.status == osEventMail) {
            mail_t *mail = (mail_t*)evt.value.p;
            pc.printf("\nNounce Found: %u \n"   , mail->printValue);
            
            mail_box.free(mail);
        }
    }
}

void putMessage(uint8_t counter)
{
   mail_t *mail = mail_box.alloc();
   mail->printValue = counter;
   mail_box.put(mail);
}