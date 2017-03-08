// DEFINE PINS
const int pin_LED = 13;
const int pin_RewTone = 51;
const int pin_WhiteNoise = 53;
char xbeeIn = ' ';
int compIn = 0;

void setup()
{
  // Initialize serial baud
  // USB
  Serial.begin(9600);
  // Computer
  SerialUSB.begin(57600);
  SerialUSB.setTimeout(100);
  // XBee.
  Serial1.begin(57600);

  // set output pins
  pinMode(pin_LED, OUTPUT);
  pinMode(pin_RewTone, OUTPUT);
  pinMode(pin_WhiteNoise, OUTPUT);

  // turn on white noise
  digitalWrite(pin_WhiteNoise, LOW);

  // Wait for Serial ports to open
  while (!Serial);
  Serial.println("USB Open");
}

void loop()
{

  // Check for computer input
  if (SerialUSB.available() > 0)
  {
    compIn = SerialUSB.read(); // used to read incoming data
    Serial.write(compIn); // print compIn input
    Serial1.write(compIn); // print send to XBee
  }

  // Check for XBee input
  while (Serial1.available() > 0) {
    xbeeIn = Serial1.read();
    if (xbeeIn == '1') {
      digitalWrite(pin_RewTone, HIGH);
      digitalWrite(pin_WhiteNoise, LOW);
      digitalWrite(pin_LED, HIGH);
      Serial.println("Rew tone on");
    }
    else if (xbeeIn == '0') {
      digitalWrite(pin_WhiteNoise, HIGH);
      digitalWrite(pin_RewTone, LOW);
      digitalWrite(pin_LED, LOW);
      Serial.println("Rew tone off");
    }
  }

}




