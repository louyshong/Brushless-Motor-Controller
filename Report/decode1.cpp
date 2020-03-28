if (command.back() == '\r') 
{    
    decodeSpeedCommand(command);
    decodePositionCommand(command);
    decodeKeyCommand(command);
    decodeTuneCommand(command);
    putMessage(PRINT_MESSAGE, "Got Message: " + command);
    command = ""; //Reset input