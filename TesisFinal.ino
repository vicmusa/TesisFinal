/******LIBRERIAS******/
#include "secrets.h"
#include "images.h"
#include <WiFiManager.h> 
#include <Wire.h>
#include "MAX30105.h"
#include "heltec.h"
#include "heartRate.h"
#include <WiFi.h>
#include <FirebaseESP32.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

MAX30105 particleSensor;

//Proporciona la información del proceso de generación de tokens.
#include "addons/TokenHelper.h"

//Proporciona la información de impresión de la carga útil de la base de datos en tiempo real y otras funciones auxiliares.
#include "addons/RTDBHelper.h"

/******Definiciones******/
#define btn 12               //
#define BAND 915E6           //Banda del LoRa
#define MAX_BRIGHTNESS 255   //
#define DELAY_BTN 750        //
        


/******* VARIABLES ******/
float promtemp,tempC;
int modo=0;
unsigned long lastTime=0, tiempo=0,resta=0,lastAct=0;
String ID;
String path="/Sensores";
int estado=0;           //Se crearon tres estados: 0=sin dedo en el sensor 1=estabilizando datos  2=datos estables
WiFiManager wifiManager;
TaskHandle_t Task1;

// Firebase Data object
FirebaseData firebaseData;
FirebaseJson json;
FirebaseAuth auth;
FirebaseConfig config;

//Variables NTP (Network Time Protocol)
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "south-america.pool.ntp.org");
unsigned long epochTime; 


/******VARIABLES MAX30102******/
double avered = 0; 
double aveir = 0;
double sumirrms = 0;
double sumredrms = 0;
uint32_t DataFinger = 0;
int i = 0;
int Num = 100;              //Se calcula SpO2 por intervalo de muestreo
double ESpO2 = 95.0;        //Valor inicial estimado de SpO2
double FSpO2 = 0.7;         //Factor de filtro para el SpO2 estimado
double frate = 0.95;        //Filtro paso bajo para el valor del LED IR/rojo para eliminar el componente AC

#define TIMETOBOOT 3000    //Tiempo de esperra en mseg para generar un valor de SpO2
#define SCALE 88.0         //Escala ajustable para mostrar el latido del corazón y la SpO2 en la misma escala
#define SAMPLING 5         //Muestreo para observar las pulsaciones con mayor precisión
#define FINGER_ON 30000    //Valor para indicar que el dedo se encuentra en el sensor, si la señal roja es más baja esto indicará que su dedo no está en el sensor
#define MINIMUM_SPO2 80.0

const byte RATE_SIZE = 4;  //Aumente si desea obtener más promedios.
byte rates[RATE_SIZE];     //Matriz de frecuencias cardíacas.
byte rateSpot = 0;         //
long lastBeat = 0;         //Momento en el que ocurre el último latido
float beatsPerMinute;
int beatAvg;
uint32_t ledIR;

#define USEFIFO



// Get hora y fecha
/******METODOS******/

unsigned long get_Time() {
  timeClient.update();
  unsigned long now = timeClient.getEpochTime();
  return now;
}

void pantalla(){           //Método para imprimir en la pantalla LCD
Heltec.display->clear();   //Se limpia la pantalla
if(estado==1)           //Condición: si se encuentra en estado 1 aparecera el siguiente mensaje en la pantalla
{
  Heltec.display -> drawString(6,40,"CALCULANDO..."); 
}
if(estado==2)
{                                                        
Heltec.display -> drawString(3,20,"BPM: "+String(beatAvg));    //Se muestra en la pantalla "BPM:" seguido del valor de pulsaciones por minuto medido.
Heltec.display -> drawString(3,10,"ID:"+ ID);                  //Se muestra en la pantalla "ID:" seguido del número correspondiente.
Heltec.display -> drawString(3,30,"TEMP: "+String(promtemp));  //Se muestra en la pantalla "TEMP:" seguido del valor de temperatura medido.
Heltec.display -> drawString(3,40,"SPO2: "+String(ESpO2));     //Se muestra en la pantalla "SPO2:" seguido del valor de oxigenación en la sangre medido.
Heltec.display->drawXbm(72,11, heartbeat_png_width, heartbeat_png_height, heartbeat_png_bits); //Se muestra la imagen de un corazón
Heltec.display -> drawString(65,0, "Covid-Monitor");           //Se muestra en la pantalla el mensaje "Covid-Monitor" correspondiente al nombre del dispositivo
Heltec.display -> drawString(60,53, "Castillo & Ortiz");       //Se muestra en la pantalla el mensaje "Castillo & Ortiz" correspondiente a los nombres de los creadores
if(modo==0){               //Condición: Si se encuentra en modo 0, correspondiente a LoRa se mostrará en la pantalla el ícono de LoRa
Heltec.display->drawXbm(0,0, lora_width, lora_height, lora_bits);
}
if(modo==1){                //Condición: Si se encuentra en modo 1, correspondiente a Wi-Fi se mostrará en la pantalla el ícono de Wi-Fi
Heltec.display->drawXbm(0,0,logowifi2_width, logowifi2_height, logowifi2_bits);
}
}
Heltec.display ->display(); 
}
void connectFirebase(){                                        //Método para la conexión con la base de datos
  config.api_key = API_KEY;                                    //
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  config.database_url = FIREBASE_HOST;
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h
  Firebase.begin(&config, &auth);
  epochTime=get_Time();
}

void sendFirebase() {                                          //Método para el envío de datos o valores a la base de datos
  String nodo = path + "/"+ID+"/";                             //
  String nodo1 = "/Historicos/"+ID+"/"+String(epochTime)+"/";  //  
  epochTime = get_Time();
  json.add("spo2", ESpO2);
  json.add("hr", beatAvg);
  json.add("temp", promtemp);  
  Firebase.updateNode(firebaseData,nodo,json);
  Firebase.updateNode(firebaseData,nodo1,json);
  }

/******ENVIAR DATOS******/
void sendData(){                                //Método para el envío de datos***************
                                                  //Ciclo
  if(modo==0){
    if(WiFi.status()== WL_CONNECTED){
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
  if(modo==1){
    if(WiFi.status()!= WL_CONNECTED){
      
    //PANTALLA CONECTANDO WIFI
    wifiManager.autoConnect("Covid-Monitor");
    connectFirebase();
    }
    sendFirebase();
  }
  }
  

/******INTERRUPCION******/
void isr(){
  if(millis()-lastTime > 1500){
    Serial.println("¡Interrupción!"); // PARA VERIFICAR QUE SE ENTRÓ EN LA INTERRUPCIÓN
    switch(modo){
      case 0: 
        modo = 1;
        Heltec.display->drawXbm(0,0,logowifi2_width, logowifi2_height, logowifi2_bits);
        Serial.println("MODO WIFI");
      break;
      case 1: 
        Heltec.display->drawXbm(0,0, lora_width, lora_height, lora_bits);
        modo=0;
      break;
      default: Serial.println("ERROR EN LA INTERRUPCION");
      break;
      lastTime=0;
      lastTime=millis();
      }
  }
}
/*****INICIALIZANDO ADC*****/
void setupADC(){                                               //Método para inicializar el ADC
  adcAttachPin(13);                                            //Para ello se usará el pin 13         
  analogReadResolution(12);                                    //Se establecieron 12 bits de resolución
  analogSetClockDiv(255);                                      //Se configuró el clock en 1338mS
}

/* INICIALIZANDO MAX30102 */
void setupMAX30102(){
  Wire1.begin(21, 22);
while (!particleSensor.begin(Wire1, I2C_SPEED_FAST)){ //Use default I2C port, 400kHz speed
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

/******INICIALIZANDO INTERRUPCIONES******/
void setupISR(){
  pinMode(btn, INPUT_PULLDOWN); // El PIN X se define como salida y con resistencia de PULL_DOWN
  attachInterrupt(btn, isr, RISING); // INTERRUPCION CUANDO HAYA FLANCO DE SUBIDA
}

/******CALCULANDO VALORES DE TEMPERATURA******/
void leerADC(){
    promtemp=0;
    for(int i=0;i<100;i++){
      tempC = (3.3 * analogRead(13) * 100.0)/4095.0; // SE LEE EL VALOR ANALOGICO Y SE CALCULA LA TEMPERATURA
      promtemp=tempC+promtemp;  // SE SUMA PARA CALCULAR UN PROMEDIO DE 100 VALORES
      }  
      promtemp=promtemp/100; // SE CALCULA EL PROMEDIO DE LA TEMP     
}

void leerMax30102(){
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
    DataFinger = ir;
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
        if (ir < FINGER_ON)
        {ESpO2 = MINIMUM_SPO2;
        estado = 0;
        
        }//indicator for finger detached
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
      Serial.print(SpO2);Serial.print(",");Serial.println(ESpO2);
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

void setup() {                                                 //Método para la configuración    
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
  Serial.println("CONFIGURACION TERMINADA");
}

void loop() {

  while(estado==0 && DataFinger < FINGER_ON){
  leerMax30102(); 
  Heltec.display->clear();
  Heltec.display -> drawString(6,40,"COLOCE EL DEDO EN EL SENSOR");
  Heltec.display ->display();
  Serial.println();
  Serial.println(String(DataFinger)+ "DATA");
 }
 
 if(estado==0){
  estado=1;
  tiempo=millis();
  Serial.println(String(tiempo));
  Serial.print(String(estado));
 }

 resta=0;
 resta=millis()-tiempo;
 if(estado==1 && resta>12000)
 {
  estado=2;
  Serial.print("ENTRE AQUI");
  delay(2000);
  tiempo=0;
  tiempo=millis();
 }
 
 leerMax30102();
 leerADC();
 if(estado==1)
 {
 pantalla();
 }
 
 if(estado==2){
  if(millis()-lastAct > 5000)
  {
  Serial.println("Actualizo y envio");
  lastAct=millis();
  pantalla();
  sendData();
  
  }
 }
 }
