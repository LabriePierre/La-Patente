//programme pour démarrer un relais suite à la détection d'un signal d'alerte sonore
//Pour la filtration de l'air de l'atelier au démarrage de la machinerie
//Pierre Labrie 2024/02/23
//version pour nano
/* Programmation des paramètres avec Blexar:
1. Connecter le bluetooth nommé Blexar
2. Envoyer la commande P50e10 pour obternir les paramètres courrants
3. Envoyer la commande P30e[valeur] 1 à 30 minutes de délais d'esxtinction
4. Envoyer la commande P40e[valeur] 60, très sensible à 90, peu sensible, pour la sensibilité en DB de capture du son
5. l'écran affichera les valeurs sélectionnées après l'affichage de 1111
*/

#include <time.h>
#include <TM1637Display.h>   //affichage 4X7 segments
#include <SoftwareSerial.h>  // pour module bluetooth
#include <EEPROM.h>          // librairie pour enregistrement dans EEPROM



//nano
#define SENSOR_PIN A0  // analog pin pour capteur sonore.
#define RELAIS_PIN 4   // digital pin du relais
#define CLK 10         //pin CLK pour affichage DEL
#define DIO 11         //Data  pour affichage DEL
#define RX 2.          // pin RX pour software serial
#define TX 3.          // pin TX pour software serial

String strglob = "";
String pin_num_char = "";
int pin_num = 255;
bool analog_bool = false;
bool digital_bool = false;
bool program_bool = false;
unsigned long timer;
long delais = 15;
long delais_temp;
unsigned long delaisExtinction = (delais * 60 * 1000);  //délais en milli-secondes
unsigned long heureOuverture;                           //début de compte
bool relais_1;                                          //indique le relais fermé
int sensibilite = 75;                                   //sensibilité du test en DB
int sensibilite_temp;

time_t now;
tm timeinfo;

//==============================================================================================================
// Global Variables détection sonore
const int sampleWindow = 15;  // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;
int LoopTime = 5000;  // Read continuously in a fast-loop for one second before finding peak & calculating average
int SPLref = 94;      // An arbritary (semi-standard) reference for finding PeakRef and AvgRef. Any known SPL level will work
int PeakRef = 62;     // Measured (or calculated) at SPLref



unsigned long Sum;            //Number of readings (number of loops counted in 5second)
unsigned long ReadStartTime;  //Save/update loop-starting time
unsigned long startMillis;    // Start of sample window
unsigned int peakToPeak;      // peak-to-peak level
float dBSPLPeak;              // Peak dB SPL reading.
float Average;                //Initilize/reset every time before starting while() loop
float dBSPLAvg;               // Average SPL reading
unsigned int signalMax;
unsigned int signalMin;


//==============================================================================================================

//construction des objets type TM1637Display
TM1637Display tm_display = TM1637Display(CLK, DIO);

//construction objet BT
SoftwareSerial ble_device(RX, TX);

// affichage delais
uint8_t DELAI[] = {
  SEG_B | SEG_C | SEG_D | SEG_E | SEG_G,  // d
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G   // E
};

// affichage sensibilite
uint8_t SENSI[] = {
  SEG_A | SEG_F | SEG_G | SEG_C | SEG_D,  // S
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G   // E
};

void setup() {
  //ferme le relais au départ
  digitalWrite(RELAIS_PIN, HIGH);
  pinMode(RELAIS_PIN, OUTPUT);

  pinMode(SENSOR_PIN, INPUT);  // Set the signal pin as input

  Serial.begin(9600);
  delay(100);
  ble_device.begin(9600);
  //luminosité des led
  // Set the brightness to 5 (0=dimmest 7=brightest)
  tm_display.setBrightness(5);
  tm_display.clear();
  tm_display.showNumberDec(8888, true);


  // remonte les paramêtres de délais et de sensibilité
  delais_temp = lectureDeEEPROM(100);
  if (delais_temp > 0) {
    delais = delais_temp;
    delaisExtinction = (delais * 60 * 1000);
  }
  sensibilite_temp = lectureDeEEPROM(200);
  if (sensibilite_temp > 0) { sensibilite = sensibilite_temp; }

  afficheParametres();
}

void loop() {
  //lecture des parametres BT
  controleBT();
  yield();
  //empêche le clignonement du détecteur sonore causé par l'affichage LED
  delay(1000);


  //si le niveau sonore est suffisant démarre le filtre à air pour une période de XX minutes (parametre delais et delaisExtinction)
  if (MesureMoyenneSonore()) {

    //Serial.println("plus de 85db");
    demarreMinuterie();
    digitalWrite(RELAIS_PIN, LOW);
    relais_1 = true;

  } else {
    //Serial.println("moins de 85db");
  }



  // après XX minutes, ferme le filtre à air
  if (relais_1 && arreteMinuterie()) {
    relais_1 = false;
    digitalWrite(RELAIS_PIN, HIGH);
  }

  //Affiche le temps restant
  if (heureOuverture != 0) {
    //timer = obtenirTimer();
    unsigned long restant = delaisExtinction - (timer - heureOuverture);
    tempsRestant(restant);
  } else {
    tempsRestant(0);
  }
}


// démarre le compteur pour arrêter la minuterie
// après un délais déterminé
void demarreMinuterie() {

  ////si l'heure est null seulement, garder l'ancienne heure sinon
  //reprendre une nouvelle heure pour prolonger le fonctionnement
  //if (heureOuverture == 0) {
  heureOuverture = obtenirTimer();
  //}
}

// suite au délais, ferme le filtre à air
bool arreteMinuterie() {

  timer = obtenirTimer();

  if ((heureOuverture != 0) && (timer - heureOuverture > delaisExtinction)) {
    heureOuverture = 0;
    return true;
  }
  return false;
}

// obtention d'un timestamp selon l'horloge interne
long obtenirTimer() {
  timer = millis();
  return timer;
}

//Mesure le maximum sonore pendant un court échantillonnage
//valide les valeurs hors plage et infinies
float MesurePeakSonore() {
  startMillis = millis();  // Start of sample window
  peakToPeak = 0;          // peak-to-peak level
  dBSPLPeak = 0.0;         // Peak dB SPL reading.
  signalMax = 0;
  signalMin = 1024;

  // collect data for 50 mS
  while (millis() - startMillis < sampleWindow) {
    sample = analogRead(SENSOR_PIN);
    if (sample < 1024)  // toss out spurious readings
    {
      if (sample > signalMax) {
        signalMax = sample;  // save just the max levels
      } else if (sample < signalMin) {
        signalMin = sample;  // save just the min levels
      }
    }
  }
  peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude


  //transformation en décibels. Le niveau doit être différent de 0 pour que log10 ne  retourne -inf
  dBSPLPeak = SPLref + 20 * log10((float)peakToPeak / PeakRef);

  //éviter les cas extrèmes. Nécessaire si on obtient des zéros.
  if (isinf(dBSPLPeak) == 0) {
    return dBSPLPeak;
  } else {
    //retouner le minimum de Db
    return 49.9;
  }
}

// Fait la moyenne des lecture sonores pendant 5 secondes
// Retourne vrai ou faux si on dépasse la sensibilité établie
boolean MesureMoyenneSonore() {
  Average = 0.0;                  //Initilize/reset every time before starting while() loop
  dBSPLAvg = 0.0;                 // Average SPL reading
  long Sum = 0;                   //Number of readings (number of loops counted in 5second)
  long ReadStartTime = millis();  //Save/update loop-starting time

  while (millis() - ReadStartTime < LoopTime)  // Normally 5 second
  {
    Average = Average + MesurePeakSonore();
    Sum++;
  }

  dBSPLAvg = Average / Sum;
  Serial.println(dBSPLAvg);

  char f_str[6];                   // prepare character array to send
  dtostrf(dBSPLAvg, 2, 1, f_str);  // format data into char array
  ble_device.write(f_str);         // send to BLExAR

  if (dBSPLAvg > sensibilite) {
    return true;
  } else {
    return false;
  }
}



// Affiche le temps restants de fonctionnement du filtre à air
// après le dernier bruit d'une machine
void tempsRestant(unsigned long currentMillis) {

  unsigned long seconds = currentMillis / 1000;
  unsigned long minutes = seconds / 60;

  currentMillis %= 1000;
  seconds %= 60;
  minutes %= 60;

  // formate les temps pour l'affichage
  int displaytime = (minutes * 100) + seconds;

  // Affiche le temps restants avec remplissage des zéros et deux points séparateur.
  tm_display.showNumberDecEx(displaytime, 0b11100000, true);
}

// Reception des données BlueTooth de controle des parametres
// Connexion avec le nom BLExAR sur l'applicarion Blexar.
void controleBT() {



  while (ble_device.available()) {
    char ble_char = ble_device.read();
    if (ble_char == '\n') {
      //Serial.println(strglob);
      if (analog_bool) {
        pinMode(pin_num, OUTPUT);
        analogWrite(pin_num, strglob.toFloat());
        analog_bool = false;
        pin_num = 255;
      } else if (digital_bool) {

        pinMode(pin_num, OUTPUT);
        if (strglob == "1") {
          digitalWrite(pin_num, HIGH);
        } else if (strglob == "0") {
          digitalWrite(pin_num, LOW);
          pinMode(pin_num, INPUT);
        }
        digital_bool = false;
        pin_num = 255;
      } else if (program_bool) {

        if (pin_num == 30) {  // ajuste délais extinction
          delais = String(strglob).toInt();
          delaisExtinction = (delais * 60 * 1000);
          ecritDansEEPROM(100, delais);
          tm_display.clear();  // Clear the display
          tm_display.showNumberDec(delais, true);
          tm_display.setSegments(DELAI, 2, 0);
          delay(5000);

        } else if (pin_num == 40) {  //ajuste sensibilite 10-90
          sensibilite = String(strglob).toInt();
          ecritDansEEPROM(200, sensibilite);
          tm_display.clear();  // Clear the display
          tm_display.showNumberDec(sensibilite, true);
          tm_display.setSegments(SENSI, 2, 0);
          delay(5000);

        } else if (pin_num == 50) {  //affichage des parametres

          afficheParametres();
        }

        program_bool = false;
        pin_num = 255;
      }
      pin_num = 255;
      strglob = "";
    } else {
      if (analog_bool == false and digital_bool == false and program_bool == false) {
        if (ble_char == 'D') {
          digital_bool = true;
        } else if (ble_char == 'A') {
          analog_bool = true;
        } else if (ble_char == 'P') {
          program_bool = true;
        } else {
        }
      } else {
        if (pin_num == 255) {
          if (ble_char != 'e') {
            pin_num_char += ble_char;
          } else if (ble_char == 'e') {
            pin_num = (String(pin_num_char).toInt());
            pin_num_char = "";
          }
        } else {
          strglob += ble_char;
        }
      }
    }
  }
  strglob = "";
  delay(20);
}

//Écriture en mémoire des paramêtres de sensibilité et de délais
void ecritDansEEPROM(int address, long number) {
  EEPROM.write(address, (number >> 24) & 0xFF);
  EEPROM.write(address + 1, (number >> 16) & 0xFF);
  EEPROM.write(address + 2, (number >> 8) & 0xFF);
  EEPROM.write(address + 3, number & 0xFF);
}



//Lecture de la mémoire des paramètres enregistrés.
long lectureDeEEPROM(int address) {
  return ((long)EEPROM.read(address) << 24) + ((long)EEPROM.read(address + 1) << 16) + ((long)EEPROM.read(address + 2) << 8) + (long)EEPROM.read(address + 3);
}

// Affiche les parametre modifiables par blueTooth
void afficheParametres() {
  tm_display.clear();  // Clear the display
  tm_display.showNumberDec(1111, true);
  delay(1000);
  tm_display.showNumberDec(delais, true);
  tm_display.setSegments(DELAI, 2, 0);
  delay(3000);
  tm_display.showNumberDec(sensibilite, true);
  tm_display.setSegments(SENSI, 2, 0);
  delay(3000);
}
