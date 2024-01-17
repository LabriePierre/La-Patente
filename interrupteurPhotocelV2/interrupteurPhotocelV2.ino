//programme pour démarrer un relais suite à la détection d'un signal d'alerte lumineux d'un LED
//Pierre Labrie 2023/01/08

#include <ESP8266WiFi.h>


#define RELAIS_PIN 0 // digital pin 0 pour relais ESP-01
#define SIGNAL_PIN 2 // digital pin pour capteur photocell



long timer;

long delaisExtinction = 30000;  //30 secondes en milli-secondes
long heureOuverture; //début de compte
bool relais_1; //indique le relais fermé


void setup() {
  digitalWrite(RELAIS_PIN, HIGH);
  pinMode(RELAIS_PIN, OUTPUT);
  pinMode(SIGNAL_PIN, INPUT_PULLUP);

  WiFi.mode(WIFI_OFF); // fermeture du Wifi non requis
}

void loop() {


  if (verifieChangeLumiere()) {

    demarreMinuterie();
    digitalWrite(RELAIS_PIN, LOW);
    relais_1 = true;
  }
  delay(100);


  // après 30 secondes, ferme la lumière
  if (arreteMinuterie() && relais_1) {
    relais_1 = false;
    digitalWrite(RELAIS_PIN, HIGH);
  }
}


// démarre le compteur pour arrêter la minuterie
// après un délais déterminé
void demarreMinuterie() {

  //si l'heure est null seulement, garder l'ancienne heure sinon
  if (heureOuverture == 0) {
    heureOuverture = obtenirTimer();
  }
}

// suite au délais, ferme la lumière
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

// Vérification de la lumière dans photocell
// Assure un réél démarrage pour éviter les interférences (debounce)
boolean verifieChangeLumiere() {
  int intDelay =500;

  bool blnBtn_A = (digitalRead(SIGNAL_PIN) == LOW);
  delay (intDelay);

  bool blnBtn_B  = (digitalRead(SIGNAL_PIN) == LOW);

  if (blnBtn_A == blnBtn_B)
  {
    delay( intDelay);
    bool blnBtn_C  = (digitalRead(SIGNAL_PIN) == LOW);
    if (blnBtn_A == blnBtn_C)
      return blnBtn_A;
  }

  return false;
}