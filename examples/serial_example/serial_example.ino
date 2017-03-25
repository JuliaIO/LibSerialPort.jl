int incomingByte = 0;   // for incoming serial data
String cmd = "";        // Usage of Arduino's String class is OK for an example
unsigned long writeInterval = 50;
unsigned long timeMarker = millis();

void handleByte(byte b)
{
  if (b == '\r' || b == '\n')
  {
    Serial.print("Received ");
    Serial.println(cmd);
    cmd = "";
  }
  else
  {
    cmd += char(b);
  }
}

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  if (Serial.available() > 0)
  {
    incomingByte = Serial.read();
    handleByte(incomingByte);
  }
  if (millis() - timeMarker > writeInterval)
  {
    timeMarker = millis();
    Serial.print(timeMarker);
    Serial.print(" ");
    Serial.println(analogRead(A0));
  }
}