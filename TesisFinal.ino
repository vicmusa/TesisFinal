/******* LIBRERIAS *******/
#include "secrets.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "WiFi.h"
#include <WiFiManager.h> 
#include <Wire.h>
#include "MAX30105.h"
#include "heltec.h"
#include "spo2_algorithm.h"

/***** Definiciones *******/
#define btn 12
#define AWS_IOT_PUBLISH_TOPIC   "data/sensors"
#define AWS_IOT_SUBSCRIBE_TOPIC "data/recibo"
#define BUFFER_LEN  256
#define BAND    915E6 //Banda del LoRa
#define MAX_BRIGHTNESS 255
#define DELAY_BTN 500

/******* VARIABLES ******/
float promtemp,tempC,spo2,hr;
int modo=0;
long lastTime=0;
char msg[BUFFER_LEN];
String ID;
WiFiManager wifiManager;
WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);

/******* METODOS *******/

/*RECIBIR MENSAJE AWS */

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
void sendData()
{
  if(modo==0)
  {
    if(WiFi.status()== WL_CONNECTED)
    {
      WiFi.disconnect(true);
    }
  Serial.println("ENVIANDO POR LORA");
/*LoRa.beginPacket();
  LoRa.setTxPower(14,RF_PACONFIG_PASELECT_PABOOST);
  LoRa.print(promtemp);
  LoRa.print("#");
  LoRa.print(hr);
  LoRa.print("$");
  LoRa.print(spo2);
  LoRa.print("/");
  LoRa.print(ID);
  LoRa.endPacket();
  */
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
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Iniciando config");
  Wire1.begin(SDA,SCL);
  Heltec.begin(true,true,true,true,BAND);
  setupADC();
  setupMAX30102();
  setupISR();
  String subID1= WiFi.macAddress().substring(9,11);
  String subID2= WiFi.macAddress().substring(12,14);
  String subID3= WiFi.macAddress().substring(15,WiFi.macAddress().length());
  ID =subID1+subID2+subID3;
  //wifiManager.autoConnect("Covid-Monitor");
  Serial.println("CONFIGURACION TERMINADA");
  
  


}

void loop() {

 hr=random(80,100);
 spo2=random(95,100);
 promtemp=random(35,38);
 sendData();
}
