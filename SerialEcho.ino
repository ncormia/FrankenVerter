/*
  Serial echo
 Language: Wiring/Arduino
 

 */

int inByte = 0;         // incoming serial byte

void setup()
{
  // start serial port at 9600 bps:
  Serial.begin(9600);
  Serial2.begin(9600);
  Serial.println("listening on Serial2 Rx.");
}

void loop()
{
  // if we get a valid byte, read analog ins:
  if (Serial2.available()) {
    // get incoming byte:
    inByte = Serial2.read();
	
	if (inByte == 0x0D)
		Serial.print("<CR>");
	else if (inByte == 0x0A)
		Serial.println("<LF>");
	else	
    	Serial.write(inByte);
  }
}


/*

$PMRRV210000?092<CR><LF>
$PMRRV210000?092

$PMRRV22V;:<CR><LF>
$PMRRV22V000011

$PMRRV2300<5<CR><LF>
$PMRRV2300<5

$PMRRV2400<6<CR><LF>
$PMRRV28E4?PN<0<CR><LF>
$PMRRV35P4IPR007<CR><LF>

PMRRV2400<6<CR><LF>
$PMRRV28E4?PN<0<CR><LF>
$PMRRV35P4IPR007<CR><LF>
$PMRRV210015?098<CR><LF>
$PMRRV22V;:<CR><LF>
$PMRRV2300<5<CR><LF>
$PMRRV2400<6<CR><LF>
$PMRRV28E4?PN<0<CR><LF>
$PMRRV35P4IPR007<CR><LF>
$PMRRV210015?098<CR><LF>
$PMRRV22V;:<CR><LF>
$PMRRV2300<5<CR><LF>
$PMRRV2400<6<CR><LF>
$PMRRV28E4?PN<0<CR><LF>
$PMRRV35P4IPR007<CR><LF>
$PMRRV210015?098<CR><LF>
$PMRRV22V;:<CR><LF>
$PMRRV2300<5<CR><LF>
$PMRRV2400<6<CR><LF>
$PMRRV28E4?PN<0<CR><LF>
$PMRRV35P4IPR007<CR><LF>
$PMRRV210015?098<CR><LF>
$PMRRV22V;:<CR><LF>
$PMRRV2300<5<CR><LF>
$PMRRV2400<6<CR><LF>
$PMRRV28E4?PN<0<CR><LF>
$PMRRV35P4IPR007<CR><LF>
$PMRRV210015?098<CR><LF>
$PMRRV22V;:<CR><LF>
$PMRRV2300<5<CR><LF>
$PMRRV2400<6<CR><LF>
$PMRRV28E4?PN<0<CR><LF>
$PMRRV35P4IPR007<CR><LF>
$PMRRV210015?098<CR><LF>
$PMRRV22V;:<CR><LF>


*/