//check each mailbox
osEvent evt = mail_box.get();
if (evt.status == osEventMail)
{
    mail_t *mail = (mail_t*)evt.value.p;

    switch(mail->type)
    {
        case(PRINT_MESSAGE):
            pc.printf("\n\r %s\r\n ", mail->message.c_str());
            break;
        case(BITCOIN_NONCE):
            ...
        case(HASH_RATE):
            ...
        case(UPDATED_KEY):
            ...
        case(DISTANCE_STATUS):
            ...
        case(SPEED_STATUS):
            ...
    }

    mail_box.free(mail);
}
