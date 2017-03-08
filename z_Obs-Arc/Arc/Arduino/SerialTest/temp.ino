char commandLetter;  // the delineator / command chooser
char numStr[4];      // the number characters and null
long speed;          // the number stored as a long integer

void serialEvent() {
	// Parse the string input once enough characters are available
	if (Serial.available() >= 4) {
		commandLetter = Serial.read();

		//dump the buffer if the first character was not a letter
		if (!isalpha(commandLetter) {
			// dump until a letter is found or nothing remains
			while ((!isalpha(Serial.peak()) && Serial.available()) {
				Serial.read(); // throw out the letter
			}

			//not enough letters left, quit
			if (Serial.available() < 3) {
				return;
			}
		}

		// read the characters from the buffer into a character array
		for (int i = 0; i < 3; ++i) {
			numStr[i] = Serial.read();
		}

		//terminate the string with a null prevents atol reading too far
		numStr[i] = '\0';

		//read character array until non-number, convert to long int
		speed = atol(numStr);
		Serial.println(speed);
	}
}
