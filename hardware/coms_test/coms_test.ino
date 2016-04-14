// If you are here, you are probably very frusturated.

void setup()
{
	Serial.begin(9600);
}

void loop()
{
	if(Serial.available() > 0){
		char in = Serial.read();
		if(in == 'a'){
			Serial.write(in);
		}
		else if(in == 'b'){
			Serial.write(in);
		}
	}
}
