int rec_pin_08 = 8;
int rec_pin_09 = 9;
int rec_pin_10 = 10;
int rec_pin_11 = 11;
int val_8 = 0;
int val_9 = 0;
int val_10 = 0;
int val_11 = 0;

void setup() {
  pinMode(rec_pin_08, INPUT);
  pinMode(rec_pin_09, INPUT);
  pinMode(rec_pin_10, INPUT);
  pinMode(rec_pin_11, INPUT);
  Serial.begin(57600);
}

void loop() {
  val_8 = digitalRead(rec_pin_08);
  val_9 = digitalRead(rec_pin_09);
  val_10 = digitalRead(rec_pin_10);
  val_11 = digitalRead(rec_pin_11);
  Serial.print("Val 8: ");
  Serial.println(val_8);

  Serial.print("Val 9: ");
  Serial.println(val_9);

  Serial.print("Val 10: ");
  Serial.println(val_10);

  Serial.print("Val 11: ");
  Serial.println(val_11);
}
