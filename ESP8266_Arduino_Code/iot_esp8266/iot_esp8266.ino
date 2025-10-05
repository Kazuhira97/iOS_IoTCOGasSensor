int count;

void setup() {
  // put your setup code here, to run once:
  delay(100);
  Serial.begin(9600);
  count = 0;

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Hello world");
  Serial.println(count);
  count++;
  delay(1000);
}
