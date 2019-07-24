// This #include statement was automatically added by the Particle IDE.
#include <Adafruit_DHT.h>

#define THRESH_HIGH 91
#define THRESH_LOW 37

// Sensor type
#define DHTTYPE DHT22    	// DHT 22 (AM2302)

// DHT22 sensor pinout:
// Pin 1 (on the left): +3.3V
// Pin 2: output
// Pin 4 (on the right): GROUND
#define DHT_5V_PIN D1
#define DHT_SENSOR_PIN D2
#define DHT_GROUND_PIN D4

#define DEVICE_KEY "YOUR_HOLOGRAM_DEVICE_KEY"
#define PHONE_NUMBER "+15555555555"

DHT dht(DHT_SENSOR_PIN, DHTTYPE);


TCPClient client;
const char *server = "cloudsocket.hologram.io";
unsigned int port = 9999;

char message[256]; 

STARTUP(cellular_credentials_set("hologram", "", "", NULL));
STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));
SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

retained float lastHeatIndex = 70;
retained float lastTemperature = 70;
retained bool firstStartRetained = true;
bool firstStart = false;
bool isConnecting = false;
bool tooHigh = false;
unsigned int longSleep = 900;

void setup() {
    // Make sure your Serial Terminal app is closed before powering your device
    Serial.begin(9600);
    // Wait for a USB serial connection for up to 1 second
    //waitFor(Serial.isConnected, 1000);

	if (firstStartRetained) {
		isConnecting = true;
		firstStart = true;
		Particle.connect();
	}
	firstStartRetained = false;
}

void loop() {
    // Give power to the sensor
    pinMode(DHT_5V_PIN, OUTPUT);
    pinMode(DHT_GROUND_PIN, OUTPUT);
    digitalWrite(DHT_5V_PIN, HIGH);
    digitalWrite(DHT_GROUND_PIN, LOW);

    // Wait for the sensor to stabilize
    delay(1000);

    // Initialize sensor
    dht.begin();

    //Serial.print("Measuring temperature");
    // Read Sensor
    float temperature = dht.getTempFarenheit();
    float heatIndex = (dht.getHeatIndex()*9)/5 + 32;
    float humidity = dht.getHumidity();

    if (isConnecting) {
		isConnecting = false;
		//Reset lastHeatIndex, sleep for longer period
		lastHeatIndex = 70;
		lastTemperature = 70;
		if (waitFor(Particle.connected, 180000)) {
			if (client.connect(server, port))
			{
				Serial.println("connected");
				client.print("S");
				client.print(DEVICE_KEY);
				client.print(PHONE_NUMBER);				

				if (firstStart) {
					firstStart = false;
					longSleep = 10;
					if (temperature > 80) {
						sprintf(message, " Test message. Temperature: %0.0fF, Heat Index: %0.0fF, Humidity: %0.0f%%", temperature, heatIndex, humidity);
					} else {
						sprintf(message, " Test message. Temperature: %0.0fF, Humidity: %0.0f%%", temperature, humidity);
					}
				} else if (tooHigh) {
					sprintf(message, " Warning, heat index too high! Temperature: %0.0fF, Heat Index: %0.0fF, Humidity: %0.0f%%", temperature, heatIndex, humidity);
				} else {
					sprintf(message, " Warning, temperature too low! Temperature: %0.0fF, Humidity: %0.0f%%", temperature, humidity);
				}
				client.println(message);
				
				while (client.available())
				{
					char c = client.read();
					Serial.print(c);
				}
				Serial.println();
				Serial.println("disconnecting.");
				client.stop();
			}
			else
			{
				Serial.println("connection failed");
				client.stop();
			}
		} else {
			longSleep = 240;
		}
		if (firstStart) {
			firstStart = false;
			longSleep = 10;
		}
		Particle.disconnect();
		Serial.flush();
        System.sleep(SLEEP_MODE_DEEP, longSleep);
    } else {
        if (temperature > 80 && heatIndex > THRESH_HIGH && lastHeatIndex > THRESH_HIGH) {
            sprintf(message, "Warning, heat index too high: %0.0fF", heatIndex);
            Serial.println(message);
            
            Particle.connect();
            isConnecting = true;
            tooHigh = true;
        } else if (temperature < THRESH_LOW && lastTemperature < THRESH_LOW) {
            sprintf(message, "Warning, temperature too low: %0.0fF", temperature);
            Serial.println(message);
            
            Particle.connect();
            isConnecting = true;
            tooHigh = false;
        } else {
            lastHeatIndex = heatIndex;
            lastTemperature = temperature;
            Serial.flush();
            System.sleep(SLEEP_MODE_DEEP, 120);
        }
    }
    
}
