//Written by Sergio Lopez Gonzalez 2017
// BSD license, all text above must be included in any redistribution
//this program send values by Mosquitto of Soil moisture,temperature, humidity and barometric pressure
#include <PubSubClient.h>
#include <ESP8266WiFi.h>

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#include <ESP8266WiFiAP.h>
#include <ESP8266WiFiGeneric.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266WiFiScan.h>
#include <ESP8266WiFiSTA.h>
#include <ESP8266WiFiType.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <WiFiServer.h>
#include <WiFiUdp.h>

#include <Wire.h>
#include <Adafruit_BMP085.h>

// Connect VCC of the BMP085 sensor to 3.3V (NOT 5.0V!)
// Connect GND to Ground
// Connect SCL to i2c clock-D1 in NOdeMCU
// Connect SDA to i2c data-D2 in NodeMCU
// EOC is not used, it signifies an end of conversion
// XCLR is a reset pin, also not used here
// Soil moisture connect to A0
// Connect dht to Pin 4
// RTC????
#define wifi_ssid "Orange-1A2B"
#define wifi_password "E7CE2462"
#define mqtt_server "192.168.1.102"
uint16_t mqtt_port=1883;
//#define mqtt_server "mqtt.thingstud.io"
//uint16_t mqtt_port=9001;
#define username="sergiolapria@gmail.com"
#define mqtt_user "sergio"
#define mqtt_password "mieres"
#define pressure_topic "bmp/pressure"
#define temperature_topic "DHT/temperature"
#define soil_topic "sensor/soil"
#define humidity_topic "DHT/humidity"
#define DHTTYPE DHT22 //DHT22(AM2302),AM2321

WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_BMP085 bmp;

int scl=1;
int sda=2;
int DHTPIN=0;
#define DHTTYPE DHT22 //AM23302
DHT_Unified dht(DHTPIN,DHTTYPE);
uint32_t delayMS;  
////
#define D0 16
#define D1 5
#define D2 4
#define D3 0
#define D4 2
#define D5 14
#define D6 12
#define D7 13
#define D8 15
////
void setup_wifi(){
  delay(10);
  //Start connection to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);

  WiFi.begin(wifi_ssid,wifi_password);

  while (WiFi.status() !=WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);
  delayMS=sensor.min_delay/1000;
}
void reconnect(){
  //loop until reconnected
  while (!client.connected()){
    //I don´t use username and password yet , if I´ll want use user and password
    if(client.connect("Sergio",mqtt_user,mqtt_password))
    //if(client.connect("ESP8266Client"))
    {
      if(client.subscribe("raspberry/frio")){
        Serial.println("FRIO");//prueba de subscripcion
      }
      Serial.println("connected");
    }else{
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      //wait 5 seconds before retry
      delay(5000);
    }
  }
}
bool checkBound(float newValue,float prevValue,float maxDiff){
  return !isnan(newValue) &&(newValue<prevValue-maxDiff || newValue>prevValue+maxDiff);
}
long lastMsg=0;
float temp=0.0;
float pressu=0.0;
float diff=1.0;
float newTemp=0.0;
float newPress=0.0;
int soil=0;
int newSoil=0;
float diff2=2.0;
float humidity=0.0;
float newhumidity=0.0;
int err=0;
int MS=0;

void setup() {
  Serial.begin(115200);
  //Wire.begin(2,1);
  if (!bmp.begin()) {
	Serial.println("Could not find a valid BMP085 sensor, check wiring!");
	
  }
  setup_wifi();
  client.setServer(mqtt_server,mqtt_port);
  //client.setServer(mqtt_server,18432);
  sensor_t sensor;
  dht.begin();
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);
  MS=sensor.min_delay/1000;
}
void loop() {
    Serial.println();
    delay(500);
    if(!client.connected()){
      reconnect();
    }
    client.loop();
    long now=millis();
    if (now-lastMsg>1000){
      lastMsg=now;
      newPress=bmp.readPressure();
      newSoil=analogRead(A0);
      delay(MS);
      //Get temperature event and print its value
      sensors_event_t event;
      dht.temperature().getEvent(&event);
      newTemp=event.temperature;
      //Get humidty event and print its value
      dht.humidity().getEvent(&event);
      newhumidity=event.relative_humidity;
      Serial.print("Temp: ");
      Serial.println(newTemp);
      Serial.print("Humidity: ");
      Serial.println(newhumidity);
      Serial.println("Pressure: ");
      Serial.println(newPress);
      Serial.print("Soil :");
      Serial.println(newSoil);
    }
    if(checkBound(newTemp,temp,diff)){
      temp=newTemp;
      Serial.print("New temp: ");
      Serial.println(String(temp).c_str());
      client.publish(temperature_topic,String(temp).c_str(),true);
    }
    if(checkBound(newPress,pressu,diff)){
      pressu=newPress;
      Serial.print("New Press: ");
      Serial.println(String(pressu).c_str());
      client.publish(pressure_topic,String(pressu).c_str(),true);

    }
    if(checkBound(newSoil,soil,diff2)){
      soil=newSoil;
      Serial.print("Soil: ");
      Serial.println(String(soil).c_str());
      client.publish(soil_topic,String(soil).c_str(),true);
    }
    if(checkBound(newhumidity,humidity,diff)){
      humidity=newhumidity;
      Serial.print("HUMIDITY: ");
      Serial.print(String(humidity).c_str());
      Serial.println(" %");
      client.publish(humidity_topic,String(humidity).c_str(),true);
    }
}
