
// Define all pins
// input pins
const int southInPin = 3; // pin 1 (TX)
const int eastInPin = 2; // pin 0 (RX)
const int northInPin = 1; // pin 2
const int westInPin = 0; // pin 3
// output pins
const int southOutPin = 6;
const int eastOutPin = 7;
const int northOutPin = 8;
const int westOutPin = 9;

// Timer variables
long startTim = millis();
int trigDel = 10; // minimum refractory period between trigger (ms)
int pulseWdth = 50; // length of time output pin is on (ms)

// south vars
volatile long southInLastTim = millis(); // track last time input high for south
volatile long southInNowTim = millis(); // track current time input high for south
volatile long southOutLastTim = millis(); // track last time input high for south
volatile long southOutNowTim = millis(); // track current time output high for south
// east vars
volatile long eastInLastTim = millis(); // track last time input high for east
volatile long eastInNowTim = millis(); // track current time input high for east
volatile long eastOutLastTim = millis(); // track last time input high for east
volatile long eastOutNowTim = millis(); // track current time output high for east
// north vars
volatile long northInLastTim = millis(); // track last time input high for north
volatile long northInNowTim = millis(); // track current time input high for north
volatile long northOutLastTim = millis(); // track last time input high for north
volatile long northOutNowTim = millis(); // track current time output high for north
// west vars
volatile long westInLastTim = millis(); // track last time input high for west
volatile long westInNowTim = millis(); // track current time input high for west
volatile long westOutLastTim = millis(); // track last time input high for west
volatile long westOutNowTim = millis(); // track current time output high for west

/*
// TEST
const int tonePin = 11;
int toneDel = 1000;
long toneTim = millis();
*/

void setup() {
  Serial.begin(30000); // Other baud rates can be used...
  // Set pin direction
  pinMode(southInPin, INPUT);
  pinMode(eastInPin, INPUT);
  pinMode(northInPin, INPUT);
  pinMode(westInPin, INPUT);
  pinMode(southOutPin, OUTPUT);
  pinMode(eastOutPin, OUTPUT);
  pinMode(northOutPin, OUTPUT);
  pinMode(westOutPin, OUTPUT);
  // Set initial output state
  digitalWrite(southOutPin, LOW);
  digitalWrite(eastOutPin, LOW);
  digitalWrite(northOutPin, LOW);
  digitalWrite(westOutPin, LOW);

  // External interrupt
  attachInterrupt(southInPin, southFnc, RISING); // south
  attachInterrupt(eastInPin, eastFnc, RISING); // east
  attachInterrupt(northInPin, northFnc, RISING); // north
  attachInterrupt(westInPin, westFnc, RISING); // west

  /*
    // TEST
    pinMode(tonePin, OUTPUT);
    */
}

void loop() {

  // Set output pins back to low
  // south
  if (southOutNowTim - southOutLastTim > pulseWdth) {
    digitalWrite(southOutPin, LOW); // set back to LOW
  }
  // east
  if (eastOutNowTim - eastOutLastTim > pulseWdth) {
    digitalWrite(eastOutPin, LOW); // set back to LOW
  }
  // north
  if (northOutNowTim - northOutLastTim > pulseWdth) {
    digitalWrite(northOutPin, LOW); // set back to LOW
  }
  // west
  if (westOutNowTim - westOutLastTim > pulseWdth) {
    digitalWrite(westOutPin, LOW); // set back to LOW
  }

  /*
  // TEST
  if (millis() - toneTim > toneDel) {
    tone(tonePin, 350, 50);
    toneTim = millis();
  }
  */
}

// Monitor Phototransducer Inputs
// south
void southFnc()
{
  southInNowTim = millis(); // save current time
  if (southInNowTim - southInLastTim > trigDel) {
    southOutNowTim = millis(); // save current time
    digitalWrite(southOutPin, HIGH); // set out high
    Serial.println("south");
    Serial.println(millis());
  }
  southInLastTim = southInNowTim;
}
// east
void eastFnc()
{
  eastInNowTim = millis(); // save current time
  if (eastInNowTim - eastInLastTim > trigDel) {
    eastOutNowTim = millis(); // save current time
    digitalWrite(eastOutPin, HIGH); // set out high
    Serial.println("east");
    Serial.println(millis());
  }
  eastInLastTim = eastInNowTim;
}
// north
void northFnc()
{
  northInNowTim = millis(); // save current time
  if (northInNowTim - northInLastTim > trigDel) {
    northOutNowTim = millis(); // save current time
    digitalWrite(northOutPin, HIGH); // set out high
    Serial.println("north");
    Serial.println(millis());
  }
  northInLastTim = northInNowTim;
}
// west
void westFnc()
{
  westInNowTim = millis(); // save current time
  if (westInNowTim - westInLastTim > trigDel) {
    westOutNowTim = millis(); // save current time
    digitalWrite(westOutPin, HIGH); // set out high
    Serial.println("west");
    Serial.println(millis());
  }
  westInLastTim = westInNowTim;
}

