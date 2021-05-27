/******* LIBRERIAS *******/
#include "secrets.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "WiFi.h"


/***** Definiciones *******/
#define btn 27

/******* VARIABLES ******/
float promtemp,tempC;

/******* METODOS *******/

/* INTERRUPCION */
void isr()
{
Serial.println("¡Interrupción!"); // PARA VERIFICAR QUE SE ENTRÓ EN LA INTERRUPCIÓN 
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


}

void loop() {
  // put your main code here, to run repeatedly:

}
