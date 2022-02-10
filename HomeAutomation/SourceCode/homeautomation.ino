Source Code for ESP32 #include <WiFi.h>
#include <FirebaseESP32.h> WiFiClient client;
#include <ESP32Servo.h> WiFiServer server(80); #include"Adafruit_MQ.h" #include"AdafruiClient.h #defineAIOadafruit.com"
#define AIO_SERVERPORT 1883 #define AIO_USERNAME	"ASG"
#define AIO_KEY	"aio_jJCo50Q9I2ZZhfrXfXFuPocSmfj


Adafruit_MQTT_Client mqtt (&client,AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Subscribe	relay	=	Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/relay connectWiFi 1"); Adafruit_MQTT_Subscribe relaym = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/relay2")
 
void MQTT_connect(); void ()
String checkClient();

FirebaseData firebaseData;

/* WIFI settings */

const char* ssid = "******";	//WIFI SSID

const char* password = "******";	//WIFI PASSWORD

#define	FIREBASE_HOST	"homeautomation-asg-default- rtdb.firebaseio.com"
#defineFIREBASE_AUTH"*****************************"


/* data received from application */ String data ="";
/* define L298N or L293D motorcontrol pins */
int Relay1 = 15; int Relay2 = 16; int Relay3 = 17;
 
int Relay4 = 18;
static const int servoPin = 26; static const int servoPin1 = 27;
int gas; String car; Servo servo1; Servo servo2;
int angle = 0;

int angleStep =5; int angleMin = 0; int angleMax =90; void setup()
{
/* initialize motor control pins as output */

pinMode(Relay1, OUTPUT)

pinMode(Relay2, OUTPUT); pinMode(Relay3, OUTPUT); pinMode(Relay3, OUTPUT);
pinMode(Relay4,	OUTPUT); pinMode(LED_BUILTIN,OUTPUT);
 
pinMode(23,INPUT); pinMode(22,INPUT); pinMode(13,INPUT); pinMode(21,OUTPUT);		
// voice	led	hall pinMode(25,OUTPUT); //voice fan bedroom servo1.attach(servoPin); servo2.attach(servoPin1); digitalWrite(Relay1,LOW); digitalWrite(Relay2,LOW); digitalWrite(Relay3,LOW); digitalWrite(Relay4,LOW);
/* start server communication */ Serial.begin(9600); connectWiFi();
server.begin();

}
uint32_t x=0; void loop()
 
{
MQTT_connect();
Adafruit_MQTT_Subscribe *subscription; while ((subscription = mqtt.readSubscription(5000)))
{
if (subscription == &relay)
{
Serial.print(F("Got: ")); Serial.println((char *)relay.lastread);

if (!strcmp((char*) relay.lastread, "On"))
{
digitalWrite(24, HIGH);
}
else
{
digitalWrite(24, LOW);
}
}
if (subscription == &relaym)
{
Serial.print(F("Got: ")); Serial.println((char *)relaym.lastread);
if (!strcmp((char*) relaym.lastread, "On"))
{
digitalWrite(25, HIGH);

}
else
{
digitalWrite(25, LOW);
}
}
gas=analogRead(g);


if(gas>2000)
 
{
//Serial.println(gas); Firebase.setString(firebaseData,"iotproject/gas","gas_leaking");
}

else

{

Firebase.setString(firebaseData,"iotproject/gas","gas_not_leaking");

//Serial.print("no gas leakage");

}

if (digitalRead(13)==HIGH)

{

Firebase.setString(firebaseData,"iotproject/pir","intrusion_not_detected");

}

else

{

Firebase.setString(firebaseData,"iotproject/pir","intrusion_detected");

}

if (digitalRead(23)==HIGH)

{

car = "car_not_parked";
 
Firebase.setString(firebaseData,” iot project/car",car);

//Serial.println(car);

}
else
{
car = "car_parked"; Firebase.setString(firebaseData,"iot project/car",car);
//Serial.println(car);

}

if (digitalRead(22)==HIGH)

Firebase.setString(firebaseData,"iotproject/fire","fire_not_detected"); else
{
Firebase.setString(firebaseData,"iotproject/fire","fire_detected");
}
/* If the server available, run the "checkClient" function */ client = server.available();
if (!client) return;
data= checkClient (); Serial.print(data);
/*** Run function according to incoming data from application ****/
if (data == "Relay1ON")
{
digitalWrite(Relay1,HIGH);
}
 
else if (data == "Relay1OFF")
{
digitalWrite(Relay1,LOW);
}
else if (data == "Relay2ON")
{
digitalWrite(Relay2,HIGH);
}
else if (data == "Relay2OFF")
{
digitalWrite(Relay2,LOW);
}
else if (data == "Relay3ON")
{
digitalWrite(Relay3,HIGH);
}
else if (data == "Relay3OFF")
{
digitalWrite(Relay3,LOW);
}
else if (data == "Relay4ON")
{
digitalWrite(Relay4,HIGH);
}
else if (data == "Relay4OFF")
{
digitalWrite(Relay4,LOW);
}
else if (data == "RELAY5SUCCESS")
{
for(int angle = 0; angle <= angleMax; angle +=angleStep)

{

servo1.write(angle); delay(20);
}
 
delay(5000);

for(int angle = 90; angle >= angleMin; angle -=angleStep)

{

servo1.write(angle); delay(20);
}
}
else if (data == "RELAY6SUCCESS")
{
for(int angle = 0; angle <= angleMax; angle +=angleStep)

{

servo2.write(angle); delay(20);
}

delay(5000);

for(int angle = 90; angle >= angleMin; angle -=angleStep)

{

servo2.write(angle); delay(20);
}

}
 
}

}

void connectWiFi()

{

Serial.println("Connecting to WIFI"); WiFi.begin(ssid, password);
while ((!(WiFi.status() == WL_CONNECTED)))

{

delay(300); Serial.print("..");
}

Serial.println(""); digitalWrite(LED_BUILTIN,HIGH);

Serial.println("WiFi connected"); Serial.println("NodeMCU Local IP is : "); Serial.print((WiFi.localIP())); Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH); Firebase.reconnectWiFi(true);
 
}

void MQTT_connect() {

int8_t ret;

// Stop if already connected.

if (mqtt.connected())

{

return;

}

Serial.print("Connecting to MQTT... "); uint8_t retries = 3;
while ((ret = mqtt.connected()) != 0) { // connect will return 0 for connected Serial.println(mqtt.connectErrorString(ret));
Serial.println("Retrying MQTT connection in 5 seconds..."); mqtt.disconnect();
delay(5000); // wait 5 seconds retries--;
if (retries == 0)
{
// basically die and wait for WDT to reset me while (1);
}
}
Serial.println("MQTT Connected!");
}
 

/**** RECEIVE DATA FROM the APP ******/ String checkClient (void)
{

while(!client.available()) delay(1);

String request = client.readStringUntil('\r'); request.remove(0, 5); request.remove(request.length()-9,9); return request;
}

Source Code for Arduino Uno #include<SoftwareSerial.h> #include <Servo.h>
Servo myservo;

int initial_position = 90;

int LDR1 = A0;	//connect The LDR1 on Pin A0	solar

int LDR2 = A1;	//Connect The LDR2 on pin A1	solar

int error = 5; int servopin=3;
 
int sensorPin= A2; //automatic on and off -- A2 int sensorValue = 0;
int ledo = 7;		//automatic on and off -- orange 12v int ledb = 8;	//automatic on and off -- blue 12v


int in = 2;	//physcial intrusion ldr int LEDr = 4;		// ledred
int LEDb = 5;	// ledblue int BUZZ = 6;		//buzzer void setup() { pinMode(in,INPUT); pinMode(ledo, OUTPUT); pinMode(ledb, OUTPUT); digitalWrite(ledo,LOW); digitalWrite(ledb,LOW); pinMode(LEDr,OUTPUT); pinMode(LEDb,OUTPUT);
 


pinMode(BUZZ,OUTPUT);
myservo.attach(servopin); pinMode(LDR1,	INPUT); pinMode(LDR2, INPUT);
myservo.write(initial_position);	//Move servo at 90 degree delay(2000);
Serial.begin(9600);

}

void loop()

{

// put our main code here, to run repeatedly:


int R1 = analogRead(LDR1); // read LDR 1 int R2 = analogRead(LDR2); // read LDR 2 int diff1= abs(R1 - R2);
int diff2= abs(R2 - R1); Serial.println(R1);
Serial.println(R2);
 
if((diff1 <= error) || (diff2 <= error))
{
else { if(R1 R2)

{

initial_position = --initial_position;
}
if(R1 < R2)
{
initial_position = ++initial_position;
}
}
myservo.write(initial_position); delay(100);
sensorValue = analogRead(sensorPin); Serial.println(sensorValue); if(sensorValue < 100)
{

Serial.println("LED light on"); digitalWrite(ledb,LOW); digitalWrite(ledo,HIGH);
}

else
{
 
digitalWrite(ledo,LOW); digitalWrite(ledb,HIGH);
}

if (digitalRead(in)==HIGH)

{

digitalWrite(LEDb,LOW); digitalWrite(LEDr,HIGH); digitalWrite(BUZZ,HIGH);
}

Else

{

digitalWrite(LEDr,LOW); digitalWrite(BUZZ,LOW); digitalWrite(LEDb,HIGH);
}

delay(1000);

}
 

