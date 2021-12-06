// cf. https://github.com/platformio/platform-ststm32/tree/master/examples

// cf. build_flags dans platformio.ini
//#define EXEMPLE_ARDUINO
//#define EXEMPLE_MBED

#if defined(EXEMPLE_ARDUINO)

#include <Arduino.h>

#define TEMPO 300

// LED verte LD2 : ARDUINO D13 -> STM32 I/O PA5 (pin 21)
// Bouton B1 : ST32 I/O PC13 (pin 2) of the STM32 microcontroller.

// Shield
static const uint8_t SW1        = PA10;
static const uint8_t SW2        = PB3;
static const uint8_t BUZZER     = PB4;
static const uint8_t RGB_ROUGE  = PC7;
static const uint8_t RGB_BLEUE  = PB6;
static const uint8_t RGB_VERTE  = PA7;
static const uint8_t LED_ROUGE  = PA6;
static const uint8_t LED_BLEUE  = PA5;

uint32_t tempo = TEMPO;
uint32_t choix = 1;

void commanderLedRGBRouge(bool etat)
{
    digitalWrite(RGB_ROUGE, etat ? HIGH : LOW);
}

void commanderLedRGBVerte(bool etat)
{
    digitalWrite(RGB_VERTE, etat ? HIGH : LOW);
}

void commanderLedRGBBleue(bool etat)
{
    digitalWrite(RGB_BLEUE, etat ? HIGH : LOW);
}

void commanderBuzzer(int tonalite)
{
    tone(BUZZER, tonalite, 200);
}

void detecteEntree1()
{
  commanderLedRGBRouge(true);
  commanderBuzzer(147);
  delay(300);
  commanderBuzzer(1047);
  commanderLedRGBRouge(false);
}

void detecteEntree2()
{
  commanderBuzzer(176);
  commanderLedRGBVerte(true);
  delay(300);
  commanderLedRGBVerte(false);
  commanderBuzzer(1760);
}

void setup()
{
  Serial.begin(115200);
  pinMode(PA5, OUTPUT);
  pinMode(PC13, INPUT);
  pinMode(SW1, INPUT);
  pinMode(SW2, INPUT);
  attachInterrupt(digitalPinToInterrupt(SW1), detecteEntree1, FALLING);
  attachInterrupt(digitalPinToInterrupt(SW2), detecteEntree2, FALLING);
  pinMode(BUZZER, OUTPUT);
  pinMode(RGB_ROUGE, OUTPUT);
  pinMode(RGB_VERTE, OUTPUT);
  pinMode(RGB_BLEUE, OUTPUT);
  Serial.println("NUCLEO Ok");
}

void loop()
{
  digitalWrite(PA5, HIGH);
  delay(tempo);
  digitalWrite(PA5, LOW);
  delay(tempo);
  int etatEntree = digitalRead(PC13);
  if(etatEntree == 0)
  {
    choix = (++choix % 3) + 1;
    tempo = choix * TEMPO;
    Serial.println(tempo);
  }
}

#elif defined(EXEMPLE_MBED)
#include "mbed.h"

using namespace std::chrono;

//DigitalIn mybutton(USER_BUTTON);
InterruptIn mybutton(USER_BUTTON);
DigitalOut myled(LED1);

Timer timer;
Timeout timeout;
Ticker ticker;

std::chrono::microseconds t(3000000);

typedef std::chrono::duration<int,std::milli> millisecondes;
//std::chrono::duration<uint32_t, std::milli> tempo(200);
millisecondes tempo(200);

void periode()
{
  tempo = std::chrono::milliseconds(500);
}

void change()
{
  tempo = std::chrono::milliseconds(1000); // 1 sec
}

void pressed()
{
  if(tempo == std::chrono::milliseconds(200))
  {
    tempo = std::chrono::milliseconds(1000); // 1 sec
  }
  else
  {
    tempo = std::chrono::milliseconds(200); // 200 ms
  }
}

int main()
{
    mybutton.fall(&pressed);
    timeout.attach(&change, 3s); // au bout de 3s, passe en clignotement lent
    ticker.attach(&periode, 5s); // cf. detach()

    timer.start();
    ThisThread::sleep_for(100ms);
    timer.stop();
    printf("timer = %llu secondes\n", duration_cast<seconds>(timer.elapsed_time()).count());
    printf("timer = %llu millisecondes\n", duration_cast<milliseconds>(timer.elapsed_time()).count());
    //printf("timer = %d millisecondes\n\r", timer.read_ms()); // obsolete
    printf("timer = %llu microsecondes\n", duration_cast<microseconds>(timer.elapsed_time()).count());
    //printf("timer = %d microsecondes\n\r", timer.read_us()); // obsolete

    timer.start();
    ThisThread::sleep_for(50ms);
    timer.stop();
    printf("timer = %llu millisecondes\n", duration_cast<milliseconds>(timer.elapsed_time()).count());

    timer.reset();
    timer.start();
    ThisThread::sleep_for(100ms);
    timer.stop();
    printf("timer = %llu millisecondes\n", duration_cast<milliseconds>(timer.elapsed_time()).count());

    while (1)
    {
      myled = !myled;
      ThisThread::sleep_for(tempo);
    }

    // Autres
    while(1)
    {
      /*
      if(mybutton == 0)
      {
        myled = !myled;
        printf("Appui bouton - Led %d\n", myled.read());
        ThisThread::sleep_for(200ms); // 200 ms
      }
      */

      /*
      myled = 1;
      ThisThread::sleep_for(1s);
      myled = 0;
      ThisThread::sleep_for(1s);
      */
    }
}

#endif
