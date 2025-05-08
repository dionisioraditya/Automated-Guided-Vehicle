int motorPin1 = 3;
int motorPin2 = 4;
int pinEnable = 5;
int encoderPinA = 2;
int encoderPinB = 6;
int aX = 0;
int aY = 0;
int value = 0;

int pulsa = 0;
float rps,rpm;
int jumlahPulsa,lastjumlahPulsa;
int interval=500;
unsigned int waktulama, waktusekarang;

void setup() {
  pinMode(motorPin2, OUTPUT);
  pinMode(pinEnable, OUTPUT);
  Serial.begin(9600);
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(0, encoder, CHANGE);
}

void loop() {
  waktusekarang=millis();
  aX = analogRead(A4);
  //aY = analogRead(A5);
  // Serial.print("X Axis = ");
  // Serial.print(aX);
  // Serial.print(", Y Axis = ");
  // Serial.println(aY);
  
  
  // Set RPM target
  int targetPWM = map(aX, 0, 1023, 0 , 255);
  // Serial.println(targetRPM);
  // Menghitung RPM setiap 1 detik
  analogWrite(pinEnable, targetPWM);
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  if (waktusekarang - waktulama >= interval){
    rps = (jumlahPulsa/360.00)/0.5;
    rpm = rps*60;
    Serial.print("Banyaknya Pulsa: ");
    Serial.print(jumlahPulsa);
    Serial.print("t");
    Serial.print("Rotasi Per Detik :");
    Serial.print(rps);
    Serial.print("t");
    Serial.print("Rotasi Per Menit :");
    Serial.println(rpm);
    pulsa=0;
    jumlahPulsa=0;
    waktulama=waktusekarang;
  }
}
void encoder()
{
  if (digitalRead(encoderPinA) == digitalRead(encoderPinB))
  {
    pulsa++;
  }
  else
  {
    pulsa--;
  }
  jumlahPulsa = pulsa*3.86;
}

