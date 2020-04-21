// Serial loopback example with a few testing options for LibSerialPort.jl

// Use this to introduce latency in the response.
unsigned long responseDelay = 0;

// Use these to generate additional traffic (timestamped ADC values).
bool write_adc = false;
unsigned long writeInterval = 50;  // ms

int incomingByte = 0;   // for incoming serial data
String cmd = "";        // Usage of Arduino's String class is OK for an example
unsigned long timeMarker = millis();

void handleByte(byte b)
{
  if (b == '\r' || b == '\n')
  {
    delay(responseDelay);
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
  Serial.println("\nGreetings from the microcontroller.");
}

void loop()
{
  if (Serial.available() > 0)
  {
    incomingByte = Serial.read();
    handleByte(incomingByte);
  }
  if (write_adc && millis() - timeMarker > writeInterval)
  {
    timeMarker = millis();
    Serial.print(timeMarker);
    Serial.print(" ");
    Serial.println(analogRead(A0));
  }
}
