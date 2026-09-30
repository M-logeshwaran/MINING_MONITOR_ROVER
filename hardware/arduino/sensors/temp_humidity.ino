#include <DHT.h>

#define dataPin A1
#define DHTTYPE DHT22   // Define the specific DHT sensor type

// Create a DHT object instance named 'dht'
DHT dht(dataPin, DHTTYPE); 

void setup() {
  Serial.begin(9600);   // Capitalized 'Serial'
  delay(2000);          // <-- ADD THIS: Gives the DHT22 time to power up stably
  dht.begin();          
  delay(1000);          // Initialize the DHT sensor
}

void loop() {
  // Read temperature and humidity
  float t = dht.readTemperature();
  float h = dht.readHumidity();

  // Check if any reads failed and exit early (to try again)
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    delay(2000);
    return;
  }

  // Print results to the Serial Monitor
  Serial.print("Temperature = ");
  Serial.print(t);
  Serial.print(" *C | ");
  
  Serial.print("Humidity = ");
  Serial.print(h);     // Changed 'f' to 'h'
  Serial.println(" %"); 
  
  delay(2000);         // Wait 2 seconds between readings
}
