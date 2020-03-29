for(int i = 1; i < input.length(); i++) 
{
    if(input[i] == 'A' || input[i] == 'B' || input[i] == 'C' || input[i] == 'D' || 
        input[i] == 'E' || input[i] == 'F' || input[i] == 'G') {
        tone = "";
        tone += input[i];
    }   
    if(input[i] == '#' || input[i] == '^') {
        tone += input[i];                    
    }
    if(input[i] == '1' || input[i] == '2' || input[i] == '3' || input[i] == '4' ||
        input[i] == '4' || input[i] == '5' || input[i] == '6' || input[i] == '7' ||
        input[i] == '8') {
        tone += input[i]; 
        pc.printf("Tone: %s\n\r", tone.c_str()); 
        
        tonesQ.push(tone);
    
    }
}