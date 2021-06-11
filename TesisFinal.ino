/******* LIBRERIAS *******/
#include "secrets.h"
#include "images.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "WiFi.h"
#include <WiFiManager.h> 
#include <Wire.h>
#include "MAX30105.h"
#include "heltec.h"
#include "heartRate.h"
MAX30105 particleSensor;

/***** Definiciones *******/
#define btn 12
#define AWS_IOT_PUBLISH_TOPIC   "data/sensors"
#define AWS_IOT_SUBSCRIBE_TOPIC "data/recibo"
#define BUFFER_LEN  256
#define BAND    915E6 //Banda del LoRa
#define MAX_BRIGHTNESS 255
#define DELAY_BTN 750
#define ESTABLE 7000

/******* VARIABLES ******/
float promtemp,tempC,spo2,hr;
int modo=0;
long lastTime=0, tiempo=0;
char msg[BUFFER_LEN];
String ID;
WiFiManager wifiManager;
WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);
TaskHandle_t Task1;
int estado; // 0 = sin dedo en el sensor 1 = Estabilizando datos  2= Datos estables
/*********** VARIABLES MAX30102 *******/

double avered = 0; double aveir = 0;
double sumirrms = 0;
double sumredrms = 0;
int i = 0;
int Num = 100;//calculate SpO2 by this sampling interval

double ESpO2 = 95.0;//initial value of estimated SpO2
double FSpO2 = 0.7; //filter factor for estimated SpO2
double frate = 0.95; //low pass filter for IR/red LED value to eliminate AC component
#define TIMETOBOOT 3000 // wait for this time(msec) to output SpO2
#define SCALE 88.0 //adjust to display heart beat and SpO2 in the same scale
#define SAMPLING 5 //if you want to see heart beat more precisely , set SAMPLING to 1
#define FINGER_ON 30000 // if red signal is lower than this , it indicates your finger is not on the sensor
#define MINIMUM_SPO2 80.0

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;
uint32_t ledIR;
#define USEFIFO
/******* METODOS *******/

/*RECIBIR MENSAJE AWS */
void pantalla()
{

  // Metodo para imprimir en la pantalla LCD

Heltec.display->clear();
while(estado==1)
{
  Heltec.display -> drawString(6,40,"CALCULANDO...");
}
// AQUI VA LO DE LA PANTALLA + CORAZON
Heltec.display -> drawString(3,20,"BPM: "+String(beatAvg));
Heltec.display -> drawString(3,10,"ID:"+ ID);
Heltec.display -> drawString(3,30,"TEMP: "+String(promtemp));
Heltec.display -> drawString(3,40,"SPO2: "+String(ESpO2));
if(modo==0)
{
  // LORA 
}
if(modo==1)
{
 // WIFI 
}
Heltec.display ->display();
  
}
void messageHandler(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}


void connectAWS()
{
  // Configure WiFiClientSecure to use the AWS IoT device credentials
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);

  // Connect to the MQTT broker on the AWS endpoint we defined earlier
  client.setServer(AWS_IOT_ENDPOINT, 8883);

  // Create a message handler
  client.setCallback(messageHandler);

  Serial.print("Connecting to AWS IOT");

  while (!client.connect(THINGNAME)) {
    Serial.print(".");
    delay(100);
  }

  if(!client.connected()){
    Serial.println("AWS IoT Timeout!");
    return;
  }

  // Subscribe to a topic
  client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);

  Serial.println("AWS IoT Connected!");
}


/* ENVIAR A AWS IOT*/
void publishMessage()
{
  Serial.print("ESTOY ENVIANDO POR AWS"); 
  String strTemp=String(promtemp);
  String strSpo2=String(spo2);
  String strHR=String(hr);  
  snprintf (msg, BUFFER_LEN,"{ \"ID\": \"%s\" , \"Temprature\" : %s , \"Pulsaciones\": %s , \"Oxigenación\": %s }",ID.c_str(),strTemp.c_str(),strHR.c_str(),strSpo2.c_str());
  Serial.println(msg);
  client.publish(AWS_IOT_PUBLISH_TOPIC,msg);
  Serial.print("Mensaje enviado");
}

/* ENVIAR DATOS */
void sendData(void *parameter)
{
  while(1)
  {
  if(estado==2)
  {
  if(modo==0)
  {
    if(WiFi.status()== WL_CONNECTED)
    {
      WiFi.disconnect(true);
    }
  Serial.println("ENVIANDO POR LORA");
  LoRa.beginPacket();
  LoRa.setTxPower(14,RF_PACONFIG_PASELECT_PABOOST);
  LoRa.print(promtemp);
  LoRa.print("#");
  LoRa.print(beatAvg);
  LoRa.print("$");
  LoRa.print(ESpO2);
  LoRa.print("/");
  LoRa.print(ID);
  LoRa.endPacket();
  
  }
  if(modo==1)
  {
   if(WiFi.status()!= WL_CONNECTED)
   {
    //PANTALLA CONECTANDO WIFI
    wifiManager.autoConnect("Covid-Monitor");
    connectAWS();
   }
   publishMessage();
  }
  delay(6000);
  vTaskDelay(10);
  }
  }
}






/* INTERRUPCION */
void isr()
{
  if(millis()-lastTime > DELAY_BTN)
  {
    Serial.println("¡Interrupción!"); // PARA VERIFICAR QUE SE ENTRÓ EN LA INTERRUPCIÓN
    switch(modo){
      case 0: 
        modo = 1;
        // LOGO DE WIFI
        Serial.println("MODO WIFI");
      break;
      case 1: 
        // LOGO DE LORA
        modo=0;
      break;
      default: Serial.println("ERROR EN LA INTERRUPCION");
      break;
      lastTime=millis();
              }
  }
}
/* INICIALIZANDO ADC */
void setupADC()
{
  adcAttachPin(13); // Se usara el pin 13 para ADC
  analogReadResolution(12); // 12 bits de Resolucion
  analogSetClockDiv(255); // 1338mS
}

/* INICIALIZANDO MAX30102 */
void setupMAX30102()
{
  Wire1.begin(21, 22);
while (!particleSensor.begin(Wire1, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30102 was not found. Please check wiring/power/solder jumper at MH-ET LIVE MAX30102 board. ");
    //while (1);
  }

  //Setup to sense a nice looking saw tooth on the plotter
  byte ledBrightness = 0x7F; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  //Options: 1 = IR only, 2 = Red + IR on MH-ET LIVE MAX30102 board
  int sampleRate = 200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 16384; //Options: 2048, 4096, 8192, 16384
  // Set up the wanted parameters
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

  particleSensor.enableDIETEMPRDY();
}
/* INICIALIZANDO INTERRUPCIONES*/
void setupISR()
{
  pinMode(btn, INPUT_PULLDOWN); // El PIN X se define como salida y con resistencia de PULL_DOWN
  attachInterrupt(btn, isr, RISING); // INTERRUPCION CUANDO HAYA FLANCO DE SUBIDA
}

/* CALCULANDO VALORES DE TEMPERATURA */
void leerADC()
{
    promtemp=0;
    for(int i=0;i<100;i++)
      {
      tempC = (3.3 * analogRead(13) * 100.0)/4095.0; // SE LEE EL VALOR ANALOGICO Y SE CALCULA LA TEMPERATURA
      promtemp=tempC+promtemp;  // SE SUMA PARA CALCULAR UN PROMEDIO DE 100 VALORES
      }  
      promtemp=promtemp/100; // SE CALCULA EL PROMEDIO DE LA TEMP
     
}
void leerMax30102()
{
  Serial.println("estoy aqui");
    uint32_t ir, red , green;
  double fred, fir;
  double SpO2 = 0; //raw SpO2 before low pass filtered

#ifdef USEFIFO
  particleSensor.check(); //Check the sensor, read up to 3 samples

  while (particleSensor.available()) {//do we have new data
#ifdef MAX30105
   red = particleSensor.getFIFORed(); //Sparkfun's MAX30105
    ir = particleSensor.getFIFOIR();  //Sparkfun's MAX30105
#else
    red = particleSensor.getFIFOIR(); //why getFOFOIR output Red data by MAX30102 on MH-ET LIVE breakout board
    ir = particleSensor.getFIFORed(); //why getFIFORed output IR data by MAX30102 on MH-ET LIVE breakout board
#endif

    
    
    i++;
    fred = (double)red;
    fir = (double)ir;
    avered = avered * frate + (double)red * (1.0 - frate);//average red level by low pass filter
    aveir = aveir * frate + (double)ir * (1.0 - frate); //average IR level by low pass filter
    sumredrms += (fred - avered) * (fred - avered); //square sum of alternate component of red level
    sumirrms += (fir - aveir) * (fir - aveir);//square sum of alternate component of IR level
    if ((i % SAMPLING) == 0) {//slow down graph plotting speed for arduino Serial plotter by thin out
      if ( millis() > TIMETOBOOT) {
        float ir_forGraph = (2.0 * fir - aveir) / aveir * SCALE;
        float red_forGraph = (2.0 * fred - avered) / avered * SCALE;
        //trancation for Serial plotter's autoscaling
        if ( ir_forGraph > 100.0) ir_forGraph = 100.0;
        if ( ir_forGraph < 80.0) ir_forGraph = 80.0;
        if ( red_forGraph > 100.0 ) red_forGraph = 100.0;
        if ( red_forGraph < 80.0 ) red_forGraph = 80.0;
        //        Serial.print(red); Serial.print(","); Serial.print(ir);Serial.print(".");
        if (ir < FINGER_ON) ESpO2 = MINIMUM_SPO2; //indicator for finger detached
        float temperature = particleSensor.readTemperatureF();
  
        Serial.print(" Oxygen % = ");
        Serial.println(ESpO2);
      }
    }
    if ((i % Num) == 0) {
      double R = (sqrt(sumredrms) / avered) / (sqrt(sumirrms) / aveir);
      // Serial.println(R);
      SpO2 = -23.3 * (R - 0.4) + 100; //http://ww1.microchip.com/downloads/jp/AppNotes/00001525B_JP.pdf
      ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2;//low pass filter
      //  Serial.print(SpO2);Serial.print(",");Serial.println(ESpO2);
      sumredrms = 0.0; sumirrms = 0.0; i = 0;
      break;
    }
    particleSensor.nextSample(); //We're finished with this sample so move to next sample
    //Serial.println(SpO2);
  }
  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }
   Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);

#endif
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Iniciando config");
  Wire1.begin(SDA,SCL);
  Heltec.begin(true,true,true,true,BAND);
  setupADC();
  setupMAX30102();
  setupISR();
  xTaskCreatePinnedToCore(
      sendData, /* Function to implement the task */
      "Task1", /* Name of the task */
      100000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      &Task1,  /* Task handle. */
      0); /* Core where the task should run */
  String subID1= WiFi.macAddress().substring(9,11);
  String subID2= WiFi.macAddress().substring(12,14);
  String subID3= WiFi.macAddress().substring(15,WiFi.macAddress().length());
  ID =subID1+subID2+subID3;
  //wifiManager.autoConnect("Covid-Monitor");
  Serial.println("CONFIGURACION TERMINADA");
  
  


}

void loop() {

 while(ledIR = particleSensor.getFIFOIR() < FINGER_ON)
 {
   
  Heltec.display->clear();
  Heltec.display -> drawString(6,40,"COLOCE EL DEDO EN EL SENSOR");
  Heltec.display ->display();
  estado=0;
   
 }
 if(estado==0)
 {
  estado=1;
  tiempo=millis();
 }
 if(estado==1 && millis()-tiempo>= ESTABLE)
 {
  estado==2;
 }
 leerMax30102();
 leerADC();
 pantalla();
 if(estado==2)
 {
  delay(2000);
 }
 }
