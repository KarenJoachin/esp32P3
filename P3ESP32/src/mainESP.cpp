//-----------------------------------------------------------------------------
//Universidad del Valle de Guatemala
//BE3015 Electronica Digital 2
//Karen Joachin
// Proyecto 3
//-----------------------------------------------------------------------------
//Librerias
//-----------------------------------------------------------------------------
// para sensor
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

//libreria para enviar datos separados por coma en usart
#include <Separador.h>
// para neopixel
#include <Adafruit_NeoPixel.h>
//-----------------------------------------------------------------------------
//Definiciond de etiquetas
//-----------------------------------------------------------------------------
MAX30105 particleSensor;
#define MAX_BRIGHTNESS 255
//-----------------------------------------------------------------------------
//Prototipo de funciones
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
//Variabls Globales
//-----------------------------------------------------------------------------
int i = 0;
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif


#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

byte pulseLED = 11; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read

// para neopixel
#define PIN        14 // On Trinket or Gemma, suggest changing this to 1
#define NUMPIXELS 1 // Popular NeoPixel ring size
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

Separador s;// es el separador cuando se envien datos
String extraido = "";
String FC = "";
String oxi = "";
//-----------------------------------------------------------------------------
//ISR
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
//Configuracion
//-----------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200); // initialize serial communication at 115200 bits per second:
  Serial2.begin(115200);// con tiva

  //para sensor
  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 no se encontro. Ver si tiene fuente de alimentaicon"));
    while (1);
  }

  Serial.println(F("Coloque su dedo en el sensor mientras se calibra.Presione cualquier tecla para comenzar"));
  while (Serial.available() == 0) ; //wait until user presses a key
  Serial.read();

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384
  // se configura el sensor
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  //para neopixel
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  // END of Trinket-specific code.

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
}

//-----------------------------------------------------------------------------
//loop principal
//-----------------------------------------------------------------------------
void loop()
{
  pixels.clear();
  //''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
  //para sensor
  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample
    // para calibrar el sensor
    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
    // neopixel avisa que ya se calibro
    for (int i = 0; i < NUMPIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(0, 150, 0));
      pixels.show();  // Send the updated pixel colors to the hardware.
    }
    //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  }
  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  while (1)
  {
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }
    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data

      digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample
      //  // *************************UART de esp32******************************
      //enviar datos
      Serial2.print(heartRate, DEC);
      Serial2.print(",");
      Serial2.println(spo2, DEC);
      //*********************************************************************
      // *************************UART de esp32********* lo que se muestra en monitor*************
      if (Serial2.available() > 0) {
        if (Serial2.read() ==1) {
          //NEOPIXEL CAMBIA DE COLOR AL SOLICITAR VALOR!!!
          for (int i = 0; i < NUMPIXELS; i++) {
            pixels.setPixelColor(i, pixels.Color(90, 110, 180));
            pixels.show();  // Send the updated pixel colors to the hardware.
            extraido = Serial2.readStringUntil('\n');
            oxi = s.separa(extraido, ',', 0);
            FC = s.separa(extraido, ',', 1);

            Serial.print("Heart rate= ");
            Serial.println(heartRate, DEC);
            Serial.print("Oximetría= ");
            Serial.println(spo2, DEC);
          }
        }//---------------------------------------------
        if (Serial2.read() ==2) {// me dice que si guarde dato
          for (int i = 0; i < NUMPIXELS; i++) {
            pixels.setPixelColor(i, pixels.Color(190, 10, 80));
            pixels.show();  // Send the updated pixel colors to the hardware.
          }
        }
        maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
      }
    }
    //*********************************************************************
  }
}

