//Written by Sergio Lopez Gonzalez 2017
// BSD license, all text above must be included in any redistribution
#include <PubSubClient.h>
#include <ESP8266WiFi.h>

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

#define wifi_ssid "XXXX"
#define wifi_password "XXXX"
#define mqtt_server "192.168.1.102"
uint16_t mqtt_port=1883;
//#define mqtt_server "mqtt.thingstud.io"
//uint16_t mqtt_port=9001;
#define username="XXXX"
#define mqtt_user "XXX"
#define mqtt_password "XXXX"
#define pressure_topic "bmp/pressure"
#define temperature_topic "bmp/temperature"

WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_BMP085 bmp;
int scl=1;
int sda=2;  
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
}
void reconnect(){
  //loop until reconnected
  while (!client.connected()){
    //I don´t use username and password yet , if I´ll want use user and password
    if(client.connect("Sergio",mqtt_user,mqtt_password))
    //if(client.connect("ESP8266Client"))
    {
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
void setup() {
  Serial.begin(115200);
  //Wire.begin(2,1);
  if (!bmp.begin()) {
	Serial.println("Could not find a valid BMP085 sensor, check wiring!");
	while (1) {}
  }
  setup_wifi();
  client.setServer(mqtt_server,mqtt_port);
  //client.setServer(mqtt_server,18432);
}
void loop() {
  /*
    Serial.print("Temperature = ");
    Serial.print(bmp.readTemperature());
    Serial.print(" *C");
    Serial.print(" /");
    Serial.print("Pressure = ");
    Serial.print(bmp.readPressure());
    Serial.print(" Pa");
    Serial.print("/////");
    // Calculate altitude assuming 'standard' barometric
    // pressure of 1013.25 millibar = 101325 Pascal
    Serial.print("Altitude = ");
    Serial.print(bmp.readAltitude());
    Serial.print(" meters");
    Serial.print(" /");
    Serial.print("Pressure at sealevel (calculated) = ");
    Serial.print(bmp.readSealevelPressure());
    Serial.print(" Pa");
    Serial.print(" /");
  // you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.
    int realSealevelPressure=101500;
    //Serial.print("Real Pressure at sealevel: ");
    //Serial.print(realSealevelPressure);
    //Serial.print(" Pa");
    //Serial.print(" /");
    Serial.print("Real altitude = ");
    Serial.print(bmp.readAltitude(realSealevelPressure));
    Serial.println(" meters");
    */
    Serial.println();
    delay(500);
    if(!client.connected()){
      reconnect();
    }
    client.loop();
    long now=millis();
    if (now-lastMsg>1000){
      lastMsg=now;
      newTemp=bmp.readTemperature();
      newPress=bmp.readPressure();
    }
    if(checkBound(newTemp,temp,diff)){
      temp=newTemp;
      Serial.print("New temp: ");
      Serial.println(String(temp).c_str());
      client.publish(temperature_topic,String(temp).c_str(),true);
      Serial.print("Temp: ");
      Serial.print(temp);
      Serial.println(" ºC");
    }
    if(checkBound(newPress,pressu,diff)){
      pressu=newPress;
      Serial.print("New Press: ");
      Serial.println(String(pressu).c_str());
      client.publish(pressure_topic,String(pressu).c_str(),true);
      Serial.print("Pr: ");
      Serial.print(pressu);
      Serial.println(" Pa");
    }
}

