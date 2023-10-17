#include <DHT.h>
#include <NewPing.h>
#include <WiFiNINA.h>
char ssid[] = "Sarang";
char pass[] = "12345678";
WiFiClient client;
#define ledPin 9
#define DHTPIN 2        // Pin where the DHT22 is connected
#define DHTTYPE DHT22   // Type of DHT sensor
DHT dht(DHTPIN, DHTTYPE);

NewPing sonar(5, 6); // Trigger (5) and Echo (6) pins for the HC-SR04
int ldrPin = A0;       // LDR sensor pin (analog pin A0)
const int currentSensorPin = A3; // Analog pin where the ACS712 OUT pin is connected

long duration;
float distance;

unsigned long LEDStartTime = 0;
float totalPowerConsumed = 0.0;

bool ledIsOn = false; // Track the LED state

void setup() {
  Serial.begin(9600);
  dht.begin();
  pinMode(ledPin, OUTPUT);
  // Connect to Wi-Fi network
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    Serial.println("Attempting to connect to WiFi...");
    delay(1000);
  }
if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected to WiFi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  }
  else {
    Serial.println("Connection failed.");
  }
  Serial.println("Connected to WiFi");
}

void loop() {
  // Read temperature and humidity from the DHT22 sensor
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  // Read light level from the LDR sensor (0-1023, lower values mean more light)
  int lightLevel = analogRead(ldrPin);
  if (lightLevel > 1000) {
    Serial.println("Bedroom lights turned off ");
  }

  // Read gas value from MQ-2 sensor
  int gasValue = analogRead(A1);
  if (gasValue > 200) {
    Serial.println("CAUTION THERE CAN BE SOME ACCIDENT RISK IN THE KITCHEN");
  }

  // Read distance from the ultrasonic sensor (HC-SR04)
  distance = sonar.ping_cm();
  if (distance < 6) {
    if (!ledIsOn) {
      // Turn on the external LED if it's not already on.
      digitalWrite(ledPin, HIGH);
      LEDStartTime = millis();
      ledIsOn = true;
      Serial.println("Someone has entered the room (LIGHTS TURN ON) ");
    }
  } else {
    if (ledIsOn) {
      // Turn off the external LED if it's not already off.
      digitalWrite(ledPin, LOW);
      // Calculate and accumulate the power consumed while the LED was on.
      unsigned long LEDOnTime = millis() - LEDStartTime;
      // Assuming a 5V supply and the sensitivity of your ACS712 model (185mV/A).
      float LEDPowerConsumed = (LEDOnTime / 1000.0) * (5.0 / 185.0);
      totalPowerConsumed += LEDPowerConsumed;
      ledIsOn = false;
    }
  }

  // Print all sensor readings to the serial monitor.
  Serial.println("Temperature: " + String(temperature) + "°C");
  Serial.println("Humidity: " + String(humidity) + "%");
  Serial.println("Light Level: " + String(lightLevel));
  Serial.println("Distance: " + String(distance) + " cm");
  Serial.println("Gas Value: " + String(gasValue));

  // Print total power consumed
  Serial.print("Total Power Consumed: ");
  Serial.print(totalPowerConsumed, 2); // Display up to two decimal places.
  Serial.println(" W");
  Serial.println(" ");
  Serial.println(" ");

  senddata(lightLevel,gasValue,temperature,humidity,totalPowerConsumed);

  delay(1500); // Delay for 1.5 seconds before taking the next reading
}

void senddata(int value1, int value2,float value3,float value4,float value5) {
  if (client.connect("192.168.156.191", 12346)) {
    // Connected to Raspberry Pi
    Serial.println("Connected to Raspberry Pi");

    // Data to send
    String dataToSend = String(value1) + "," + String(value2)+"," + String(value3)+"," + String(value4)+"," + String(value5);

    // Send the data
    client.println(dataToSend);

    // Close the connection
    client.stop();

    // Wait for a moment before sending more data
    delay(5000);
  } else {
    Serial.println("Connection failed");
}
}
  
