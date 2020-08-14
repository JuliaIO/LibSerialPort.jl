// Serial loopback example with a few testing options for LibSerialPort.jl

// Use this to introduce latency in the response.
unsigned long responseDelay = 0;

// Use these to generate additional traffic (timestamped ADC values).
bool write_adc = false;
unsigned long writeInterval = 50;  // ms

unsigned long timeMarker = millis();

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  if (Serial.available() > 0)
  {
    delay(responseDelay);
    Serial.write(Serial.read());
  }
  if (write_adc && millis() - timeMarker > writeInterval)
  {
    timeMarker = millis();
    Serial.print(timeMarker);
    Serial.print(" ");
    Serial.println(analogRead(A0));
  }
}
