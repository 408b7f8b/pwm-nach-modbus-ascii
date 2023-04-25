#include <ArduinoRS485.h>

/*
* (c) 2023 David Breunig
* 25.4.2023 ist darauf ausgelegt aus einem PWM-Signal ein Drehzahlsignal über MODBUS-ASCII über RS485 zu erzeugen.
* Misst Dauer des geschalteten oder abgeschalteten Teils einer PWM-Periode und ermittelt den Lastanteil. Der Lastanteil wird auf 0..MAX_RPM umgelegt und an per RS485/MODBUS-ASCII gesendet.
* Wenn das Signal längere Zeit geschaltet oder abgeschaltet ist, wird MAX_RPM oder 0 gegeben.
* Die Aktivität ist an Richtung und einen Enable-Pin gekoppelt.
*/

int pwm_input = 2;
int dir_input = 3;
int enable_input = 4;
int dutycycle = 0;
int rpm = 0;

#define MAX_RPM 2000

uint8_t genLRC(uint8_t* content, uint8_t elements)
{
  uint8_t tmp = 0;
  for (uint8_t i = 0; i < elements; ++i)
    tmp += content[i];

  uint8_t ret = 0xFF - tmp + 1;
  return ret;
}

void uint16tNachUint8ts(uint16_t* uint16, uint8_t* most8bit, uint8_t* least8bit)
{
  *most8bit = (uint8_t)(*uint16 >> 8);
  *least8bit = (uint8_t)(*uint16 & 0x00FF);
}

void bauMirDochBitteEineASCIIKette(uint8_t* bytekette, uint8_t laenge_bytekette, char* ziel)
{
  uint16_t ziel_pos = 0;
  for (uint8_t pos = 0; pos < laenge_bytekette; ++pos)
  {
    uint8_t tmp = bytekette[pos] >> 4;
    *(ziel+ziel_pos++) = (tmp > 9 ? tmp + 0x37 : tmp + 0x30);

    tmp = bytekette[pos] & 0x0F;
    *(ziel+ziel_pos++) = (tmp > 9 ? tmp + 0x37 : tmp + 0x30);
  }
}

class Nachricht {
  uint8_t Adresse_Zielsystem;
  uint8_t Befehl;
  uint16_t Adresse_Speicher_Zielsystem;
  uint16_t* Woerter;
  uint8_t Woerter_Anzahl;
  
  public:
  ~Nachricht()
  {
    if (Woerter != NULL) free(Woerter);
  };

  Nachricht(uint8_t Adresse_Zielsystem, uint8_t Befehl, uint16_t Adresse_Speicher_Zielsystem, uint16_t* Woerter, uint8_t Woerter_Anzahl)
  {
    this->Adresse_Zielsystem = Adresse_Zielsystem;
    this->Befehl = Befehl;
    this->Adresse_Speicher_Zielsystem = Adresse_Speicher_Zielsystem;
    this->Woerter = (uint16_t*)malloc(sizeof(uint16_t)*Woerter_Anzahl);
    memcpy(this->Woerter, Woerter, sizeof(uint16_t)*Woerter_Anzahl);
    this->Woerter_Anzahl = Woerter_Anzahl;
  }

  void Senden()
  {
    uint8_t laenge = 7; // 1 (ADR) + 1 (CMD) + 2 (DATA_REGISTER) + 2 (DATA) + 1 (LRC)
    if (Woerter_Anzahl > 1)
      laenge += (1 + Woerter_Anzahl*2); // 1 (ADR) + 1 (CMD) + 2 (DATA_REGISTER) + 1 (LRC) + 1 (DATA WORD HIGH NUMBER) + 1 (DATA WORD LOW NUMBER) + 1 (DATA BYTE NUMBER) + 2 FÜR JEDES WORT

    uint8_t Kette[laenge];
    uint8_t pos = 0;

    Kette[pos++] = Adresse_Zielsystem;
    Kette[pos++] = Befehl;
    uint16tNachUint8ts(&Adresse_Speicher_Zielsystem, &(Kette[pos++]), &(Kette[pos++]));

    if (Woerter_Anzahl > 1)
    {
      Kette[pos++] = 0;
      Kette[pos++] = Woerter_Anzahl;
      Kette[pos++] = Woerter_Anzahl;
      for (uint8_t wort = 0; wort < Woerter_Anzahl; ++wort)
        uint16tNachUint8ts(&Woerter[wort], &(Kette[pos++]), &(Kette[pos++]));
    } else {
      uint16tNachUint8ts(&Woerter[0], &(Kette[pos++]), &(Kette[pos++]));
    }
    
    Kette[pos++] = genLRC(&Kette[0], pos);

    const uint8_t ascii_laenge = 2*laenge+1+2;
    char ascii[ascii_laenge];
    ascii[0] = 0x3A;
    
    bauMirDochBitteEineASCIIKette(Kette, pos, &(ascii[1]));
    
    ascii[ascii_laenge-2] = 0x0D;
    ascii[ascii_laenge-1] = 0x0A;
    
    for (pos = 0; pos < ascii_laenge; ++pos)
      RS485.write(ascii[pos]);
  }
};

void AktiviereDrehzahlregelung()
{
  uint16_t regelung = 0x0030;
  Nachricht nachricht(1, 6, 0, &regelung, 1);
  nachricht.Senden();
}

void StelleDrehzahl(int16_t drehzahl)
{
  Serial.println("StelleDrehzahl " + drehzahl);
  uint16_t drehzahl_tmp;
  if (drehzahl >= 0)
  {
    drehzahl_tmp = drehzahl;
  } else {
    drehzahl_tmp = -1*drehzahl;
    drehzahl_tmp = 0xFFFF - drehzahl_tmp + 1;
  }
  
  Nachricht nachricht(1, 6, 307, &drehzahl_tmp, 1);

  nachricht.Senden();
}

void StelleBeschleunigungen(uint16_t beschleunigungszeit_in_ms)
{
  Nachricht nachricht(1, 6, 303, &beschleunigungszeit_in_ms, 1);
  nachricht.Senden();
  
  Nachricht nachricht2(1, 6, 304, &beschleunigungszeit_in_ms, 1);
  nachricht2.Senden();
}



void setup() {
  pinMode(pwm_input, INPUT);
  pinMode(dir_input, INPUT);
  pinMode(enable_input, INPUT);

  RS485.begin(57600, SERIAL_8O1);
  Serial.begin(115200);

  delay(500);
  
  RS485.beginTransmission();
  delay(100);
  AktiviereDrehzahlregelung();
  delay(100);
  StelleBeschleunigungen(1);
  delay(100);
  StelleDrehzahl(0);
  RS485.endTransmission();
}

bool letzte = true;

bool aktiv = false;

int limit = 10000;

unsigned long fallend, steigend;

unsigned long jetzt;

//#define von_fallend_bis_fallend
#define von_steigend_bis_steigend



void startRoutine()
{
  Serial.println("startRoutine");
  while(!digitalRead(enable_input)) {}
  Serial.println("startRoutine: enabled");
  while(!digitalRead(pwm_input)) {}
  Serial.println("startRoutine: pwm aktiv");

  letzte = true;
  steigend = micros();
  Serial.println("startRoutine: steigend = " + steigend);
}

void loop() {
  jetzt = micros();

  if (!digitalRead(enable_input))
  {
    Serial.println("loop: nicht enabled");
    if (aktiv) {
      StelleDrehzahl(0);
      aktiv = false;
    }

    startRoutine();
  } else {
    bool pwm_input_jetzt = digitalRead(pwm_input);

    if (letzte == pwm_input_jetzt)
    {
      if (letzte && jetzt - limit >= steigend)
      {
        StelleDrehzahl(MAX_RPM);        
      } else if (!letzte && jetzt - limit >= fallend) {
        StelleDrehzahl(0);
      }
    } else {
#ifdef von_fallend_bis_fallend
      if (letzte) //fallend
      {
        unsigned long diff_gesamt = jetzt - fallend;
        unsigned long diff_positiv = jetzt - steigend;

        rpm = diff_positiv / diff_gesamt * MAX_RPM;

        fallend = jetzt;
#endif
#ifdef von_steigend_bis_steigend
      if (!letzte)
      {
        unsigned long diff_gesamt = jetzt - steigend;
        unsigned long diff_negativ = jetzt - fallend;

        rpm = diff_negativ / diff_gesamt * MAX_RPM;

        steigend = jetzt;
#endif

        if (digitalRead(dir_input))
        {
          StelleDrehzahl(rpm);
        } else {
          StelleDrehzahl(-1*rpm);
        }
      }

      letzte = pwm_input_jetzt;

#ifdef von_fallend_bis_fallend
      steigend = jetzt;
#endif
#ifdef von_steigend_bis_steigend
      fallend = jetzt;
#endif
    }
  }
}