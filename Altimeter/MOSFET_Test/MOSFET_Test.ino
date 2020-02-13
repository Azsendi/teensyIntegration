
const int MOSFET_PIN_ONE = 2, MOSFET_PIN_TWO = 3;


void setup() 
{
  pinMode(MOSFET_PIN_ONE, OUTPUT);
  pinMode(MOSFET_PIN_TWO, OUTPUT);
  Serial.begin(9600);
}

void loop()
{

digitalWrite(MOSFET_PIN_ONE,HIGH);
digitalWrite(MOSFET_PIN_TWO,HIGH);
delay(1000);
digitalWrite(MOSFET_PIN_ONE,LOW);
digitalWrite(MOSFET_PIN_TWO,LOW);
delay(1000);

}
