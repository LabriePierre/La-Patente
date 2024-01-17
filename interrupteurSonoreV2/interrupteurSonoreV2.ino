//programme pour démarrer un relais suite à la détection d'un signal d'alerte sonore
//Pour la filtration de l'air de l'atelier au démarrage de la machinerie
//Pierre Labrie 2023/03/28

#include <ESP8266WiFi.h>
#include <time.h>
#include <TM1637Display.h>

#define RELAIS_PIN 0  // digital pin 0 par défaut pour relais ESP-01
#define SIGNAL_PIN 3  // digital pin pour capteur sonore. Ne pas utiliser la 2 car au reboot démarrera en mode flash si le signal sonore est élevé.
#define CLK 1         //pin CLK pour affichage DEL
#define DIO 2         //pin DATA pour affichage DEL


unsigned long timer;
unsigned long delaisExtinction = 900000;  //0; //15 minutes en milli-secondes
unsigned long heureOuverture;             //début de compte
bool relais_1;                            //indique le relais fermé
const int sampleWindow = 5000;            //temps d'échantillon en milli-secondes

time_t now;
tm timeinfo;

//construction des objets type TM1637Display
TM1637Display tm_display = TM1637Display(CLK, DIO);

void setup() {
  //ferme le relais au départ
  digitalWrite(RELAIS_PIN, HIGH);
  pinMode(RELAIS_PIN, OUTPUT);

  //détecteur sonore dèjà à HIGH au démarrarage
  pinMode(SIGNAL_PIN, INPUT);

  // fermeture du Wifi non requis
  WiFi.mode(WIFI_OFF);
  //Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY);

  //luminosité des led
  // Set the brightness to 5 (0=dimmest 7=brightest)
  tm_display.setBrightness(5);
  tm_display.clear();
  tm_display.showNumberDec(8888, true);
}

void loop() {

  //si le niveau sonore est suffisant démarre le filtre à air pour une période de 30 minutes
  if (verifieChangeSonore()) {

    demarreMinuterie();
    digitalWrite(RELAIS_PIN, LOW);

    relais_1 = true;
  }


  
  //empêche le clignonement du détecteur sonore causé par l'affichage LED
  delay(5000);

  // après 30 minutes, ferme le filtre à air
  if (relais_1 && arreteMinuterie() ) {
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

// Vérification du niveau sonore ambiant
// Assure un réél démarrage pour éviter les interférences (debounce)
boolean verifieChangeSonore() {

  float sample = 0.0;
  float sampleSize = 0.0;

  unsigned long startMillis = millis();  // Start of sample window
  // collect data for 5000 mS
  while (millis() - startMillis < sampleWindow) {
    yield();
    sample += digitalRead(SIGNAL_PIN);

    sampleSize++;
  }

  int result = (sample / sampleSize) * 100;

  if (result < 90) {
    return true;
  }

  return false;
}

// Affiche le temps restants de fonctionnement du filtre à air
//après le dernier bruit d'une machine
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