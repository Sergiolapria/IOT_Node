
//Written by Sergio Lopez Gonzalez 2018
// BSD license, all text above must be included in any redistribution
//this program send values by Mosquitto of Soil moisture,temperature, humidity and barometric pressure

#include <PubSubClient.h>
#include <ESP8266WiFi.h>

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Adafruit_BMP280.h>

#define wifi_ssid "xxxxxxx"
#define wifi_password "xxxxxx"
#define mqtt_server "192.168.1.100" //pi zero server
uint16_t mqtt_port=1883;
///Only for external brokers
#define ext_mqtt_server "m23.cloudmqtt.com"
uint16_t ext_mqtt_port=10760;
#define username="xxxxxx"
#define mqtt_user "xxxxxx"
#define mqtt_password "xxxxxx"
//topics
#define bmp_pressure "bmp/pressure"
#define bmp_temperature "bmp/temperature"
#define dht_temperature "DHT/temperature"
#define sensor_soil "sensor/soil"
#define dht_humidity "DHT/humidity"
#define i0_topic "GPI/i0"
#define i1_topic "GPI/i1"
#define i2_topic "GPI/i2"
#define DHTTYPE DHT22 //DHT22(AM2302),AM2321
#define state_topic "state"
#define ip_topic "IP"

WiFiClient espClient;
PubSubClient client(espClient);
PubSubClient extClient(espClient);

int DHTPIN=0; //D3

#define DHTTYPE DHT22 //AM23302
DHT_Unified dht(DHTPIN,DHTTYPE);
Adafruit_BMP280 bmp;//I2C
  
////PinOut del NodeMCU
#define D0 16 
#define D1 5 //scl
#define D2 4 //sda
#define D3 0 //dht
#define D4 2 //EnableSensor
#define D5 14 //led
#define D6 12 //led
#define D7 13
#define D8 15
////
long lastMsg=0;
float temp=0.0;
float pressure=0.0;
float newTemp=0.0;
float newPressure=0.0;
int soil=0;
int newSoil=0;
float humidity=0.0;
float newhumidity=0.0;
float newSoilValue=0.0;
float temp1=0.0;
float newTemp1=0.0;
float altitud=0.0;
float newAltitud=0.0;

int err=0;
int MS=10;
int timer=100;
int frecu=0;
int lastSend=0;
////
void callback(char* topic,byte* payload,unsigned int length);

void parpadeo(int frec){
  digitalWrite(D5,HIGH);
  digitalWrite(D6,LOW);
  delay(frec);
  digitalWrite(D5,LOW);
  digitalWrite(D6,HIGH);
  delay(frec);
  digitalWrite(D5,HIGH);
  digitalWrite(D6,LOW);
  delay(frec);
  digitalWrite(D5,LOW);
  digitalWrite(D6,HIGH);
  delay(frec);
}
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
  while (!client.connected()||!extClient.connected()){
    //I don´t use username and password yet , if I´ll want use user and password
    if(client.connect("Sergio",mqtt_user,mqtt_password)&& extClient.connect("Sergio",mqtt_user,mqtt_password))
    //if(client.connect("ESP8266Client"))
    {

        client.subscribe(i0_topic);
        client.subscribe(i1_topic);
//        client.publish(ip_topic,String(WiFi.localIP()),true);
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
void setup_sensor(){
  sensor_t sensor;
  dht.begin();
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);
  MS=sensor.min_delay/1000;
  Wire.begin();
  if(!bmp.begin()){
    Serial.println("Error BMP");
    const char* alarm_topic="Fallo sensor presion";
  }
  pinMode(D5,OUTPUT);
  pinMode(D0,OUTPUT);
  pinMode(D6,OUTPUT);
}
void setup() {
  Serial.begin(115200);
  //Wire.begin(2,1);
  setup_wifi();
  client.setServer(mqtt_server,mqtt_port);
  client.setCallback(callback);
  extClient.setServer(mqtt_server,18432);
  setup_sensor();
}
void send_to_node(){
  float diffP=100.00;
  float diffT=0.10;
  float diffH=3.00;
  float diffSoil=1.00;
      if(checkBound(newTemp,temp,diffT)){
      temp=newTemp;  
      client.publish(dht_temperature,String(temp).c_str(),true);
      extClient.publish(dht_temperature,String(temp).c_str(),true);
      parpadeo(timer);
      Serial.println(temp);
      lastMsg=millis();
     }
    if(checkBound(newSoilValue,soil,diffSoil)){
      soil=newSoilValue;
      client.publish(sensor_soil,String(soil).c_str(),true);
      extClient.publish(sensor_soil,String(soil).c_str(),true);
      parpadeo(timer);
      Serial.println(newSoil);
      lastMsg=millis();
    }
    if(checkBound(newhumidity,humidity,diffH)){
      humidity=newhumidity;
      client.publish(dht_humidity,String(humidity).c_str(),true);
      extClient.publish(dht_humidity,String(humidity).c_str(),true);
      parpadeo(timer);
      Serial.println(humidity);
      lastMsg=millis();
    }
    if(checkBound(newPressure,pressure,diffP)){
      pressure=newPressure;
      client.publish(bmp_pressure,String(pressure).c_str(),true);
      extClient.publish(bmp_pressure,String(pressure).c_str(),true);
      parpadeo(timer);
      Serial.println(pressure);
      lastMsg=millis();
    }
    if(checkBound(newTemp1,temp1,diffT)){
      temp1=newTemp1;
      client.publish(bmp_temperature,String(temp1).c_str(),true);
      extClient.publish(bmp_temperature,String(temp1).c_str(),true);
      parpadeo(timer);
      lastMsg=millis();
    }
}
void read_sensors(){
      read_soil();
      read_dht();
      read_bmp();
}
/////Lecturas de los diferentes sensores
void read_soil(){
  digitalWrite(D0,HIGH);
  delay(100);
  newSoil=analogRead(A0);
  delay(MS); //?
  digitalWrite(D0,LOW);
  newSoilValue=newSoil*(100.0/1023.0);
  Serial.print("DIFF SOIL:");
  Serial.println(soil-newSoilValue);
}
void read_dht(){
        //Get temperature event and print its value
        delay(1000);
      sensors_event_t event;
      dht.temperature().getEvent(&event);
      newTemp=event.temperature;
      //Get humidty event and print its value
      dht.humidity().getEvent(&event);
      newhumidity=event.relative_humidity;
      Serial.print("DIFF TEMP:");
      Serial.println(temp-newTemp);
      Serial.print("DIFF HUM:");
      Serial.println(humidity-newhumidity);
}
void read_bmp(){
      newTemp1=bmp.readTemperature();
      newPressure=bmp.readPressure();
      Serial.print("DIFF P:");
      Serial.println(pressure-newPressure);
      newAltitud=bmp.readAltitude();
}
void callback(char* topic, byte* payload, unsigned int length) {
  //client.publish("bmp/pressure",payload, length);
  //payload es un byte array////
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  int i=0;
  for (i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println("");

}
void loop(){

    if(!client.connected()|| !extClient.connected()){
      reconnect();
    }
    
  
    //digitalWrite(D5,LOW);
    client.loop();
    parpadeo(500);
    long now=millis();
    if (now-lastMsg>500){
      read_sensors();      
    }
    send_to_node();   
}
