// Code For the ESP12E to communicated information from MySQMPro -WIfi to Blynk

#define BLYNK_PRINT Serial

#include <SoftwareSerial.h>
SoftwareSerial s(D6,D5); // RX=TX(9), TX=RX(8)
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <SimpleTimer.h>

char auth[] = "Your Blynk Token";
char ssid[] = " Wifi SSID";
char pass[] = " Wifi Password";

BlynkTimer timer; // Create a Timer object called "timer"! 


void sendSensorData()
{

StaticJsonBuffer<500> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(s);
  if (root == JsonObject::invalid()){
    return;
  }
    
root.prettyPrintTo(Serial);  

float mySQMreading =      root["data1"];
float nelm =              root["data2"];
float mlx90614ambient =   root["data3"];  
float mlx90614object =    root["data4"];  
bool  raining =           root["data5"];   
int   volts =             root["data6"];   
float lux  =              root["data7"];
int   skystate   =        root["data8"];  
int   setpoint1  =        root["data9"];
int   setpoint2  =        root["data10"];
int   bme280humidity =    root["data11"];
float bme280temperature = root["data12"];  
float dewpoint =          root["data13"];
int   bme280pressure   =  root["data14"];
String  CloudStatus = "unkown";
String  RainStatus  = "true";

if  (skystate == 0) { CloudStatus = "Clear";}
if  (skystate == 1) { CloudStatus = "Partly-Cloudy";}
if  (skystate == 2) { CloudStatus = "Cloudy";}
if  (skystate == 3) { CloudStatus = "Unknown";}

if (raining == false) { RainStatus = "No"; }
if (raining == true) { RainStatus = "Yes"; }

//Serial.println(CloudStatus);

Blynk.virtualWrite(V1, mySQMreading);
Blynk.virtualWrite(V2, nelm);
Blynk.virtualWrite(V3, mlx90614ambient);
Blynk.virtualWrite(V4, mlx90614object);
Blynk.virtualWrite(V5, RainStatus);
Blynk.virtualWrite(V6, volts);
Blynk.virtualWrite(V7, lux);
Blynk.virtualWrite(V8, CloudStatus);
Blynk.virtualWrite(V9, setpoint1);
Blynk.virtualWrite(V10,setpoint2);
Blynk.virtualWrite(V11,bme280humidity);
Blynk.virtualWrite(V12,bme280temperature);
Blynk.virtualWrite(V13,dewpoint);
Blynk.virtualWrite(V14,bme280pressure);
Blynk.virtualWrite(V15,skystate);
}

void setup() {
  Serial.begin(9600);
  Serial.println("test");
  s.begin(9600);
  while (!Serial) continue;
  Blynk.begin(auth, ssid, pass);
  timer.setInterval(1000L, sendSensorData);
}
 
void loop()
{
  Blynk.run();
  timer.run(); 
}
