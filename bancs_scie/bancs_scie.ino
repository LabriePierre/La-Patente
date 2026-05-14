// Fonctions de sélection des bancs de scie
// Pierre Labrie — 2026/05/05
// Version optimisée

// ─── Pins ────────────────────────────────────────────────────────────────────
#define PIN_ON_A 2
#define PIN_ON_B 3
#define RELAIS_A 11
#define RELAIS_B 12
#define LED_A 9
#define LED_B 10
#define SENSEUR_COURANT A0
#define DEBUG true


// ─── Paramètres de mesure ────────────────────────────────────────────────────
const float LIMITE_COURANT = 0.35;     // Ampères — seuil de déclenchement
const uint16_t SAMPLE_WINDOW_MS = 15;  // Fenêtre d'un échantillon peak (ms)
const uint16_t MESURE_DUREE_MS = 500;  // Durée totale de mesure (ms)
const uint32_t STOP_MILLIS = 10000UL;  // Durée max des relais (ms)

// ─── État global minimal ─────────────────────────────────────────────────────
bool verrou_A = false;
bool verrou_B = false;
bool relais_A = false;
bool relais_B = false;
uint32_t readStopTime = 0;     // 0 = minuterie inactive
float baselineCourant = 0.0f;  // Courant au repos mesuré juste avant le relais
bool baselinePris = false;

// pour chase light indicateur démarrage manuel
const uint16_t INTERVALLE_LED = 200;  // ms — modifie pour changer la vitesse
uint32_t dernierClignotement = 0;
bool etatLED = false;


// ─── Setup ───────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);

  // Sorties d'abord à LOW, puis configurées OUTPUT (évite glitch)
  digitalWrite(RELAIS_A, LOW);
  digitalWrite(RELAIS_B, LOW);
  pinMode(RELAIS_A, OUTPUT);
  pinMode(RELAIS_B, OUTPUT);

  // Entrées avec pull-up interne
  pinMode(PIN_ON_A, INPUT_PULLUP);
  pinMode(PIN_ON_B, INPUT_PULLUP);

  delay(500);
  Serial.println(F("Setup OK"));
}

// ─── Loop ────────────────────────────────────────────────────────────────────
void loop() {
  // Les pins INPUT_PULLUP sont LOW quand enfoncées → on inverse pour lisibilité
  bool pin_A = digitalRead(PIN_ON_A);  // true = bouton pressé
  bool pin_B = digitalRead(PIN_ON_B);

  // Déverrouillage automatique quand le bouton est relâché
  if (!pin_A) verrou_A = false;
  if (!pin_B) verrou_B = false;

  // ── Logique de sélection de direction ──────────────────────────────────────
  if (pin_A && !pin_B && !verrou_A) {
    if (!baselinePris) {
      capturerBaseline();  // ← une seule fois par cycle
      baselinePris = true;
    }
    if (!courantDetecte()) {
      setRelais(true, false);
      setLED(true, false);
      demarreMinuterie();
    } else {
      verrou_A = true;
      setRelais(false, false);
      baselinePris = false;
      //faire clignoter les LED
      clignoteLED(true, true);
      Serial.println(F("Verrou A — surcourant"));
    }

  } else if (pin_B && !pin_A && !verrou_B) {
    if (!baselinePris) {
      capturerBaseline();  // ← une seule fois par cycle
      baselinePris = true;
    }
    if (!courantDetecte()) {
      setRelais(false, true);
      setLED(false, true);
      demarreMinuterie();
    } else {
      verrou_B = true;
      setRelais(false, false);
      baselinePris = false;
      setLED(false, false);
      //faire clignoter les LED
      clignoteLED(true, true);
      Serial.println(F("Verrou B — surcourant"));
    }

  } else {
    // Pin inactive OU verrou actif → relais ouverts
    setRelais(false, false);
    setLED(false, false);
  }

  // ── Minuterie de sécurité : coupe les relais après STOP_MILLIS ─────────────
  if ((relais_A || relais_B) && minuterieEchouee()) {
    if (relais_A) verrou_A = true;
    if (relais_B) verrou_B = true;
    setRelais(false, false);
    baselinePris = false;
    Serial.println(F("Minuterie — relais coupés"));
  }

  // Pas de delay() ici : la mesure de courant (1 s) constitue déjà la temporisation.
  // Si la mesure n'est pas déclenchée, 50 ms suffisent pour la réactivité.
  delay(50);
}

// ─── Pilotage des relais ──────────────────────────────────────────────────────
// Centralise toutes les écritures sur les relais pour éviter les doublons.
void setRelais(bool a, bool b) {
  relais_A = a;
  relais_B = b;
  digitalWrite(RELAIS_A, a ? HIGH : LOW);
  digitalWrite(RELAIS_B, b ? HIGH : LOW);
}

void setLED(bool a, bool b) {

  digitalWrite(LED_A, a ? HIGH : LOW);
  digitalWrite(LED_B, b ? HIGH : LOW);
}
void clignoteLED(bool a, bool b) {
  uint32_t maintenant = millis();

  if (maintenant - dernierClignotement >= INTERVALLE_LED) {
    dernierClignotement = maintenant;
    etatLED = !etatLED;
    digitalWrite(LED_A, etatLED);
    digitalWrite(LED_B, etatLED);
  }
}
// ─── Minuterie ────────────────────────────────────────────────────────────────
void demarreMinuterie() {
  // Ne démarre que si aucun relais n'était déjà actif (premier appel)
  if (readStopTime == 0) {
    readStopTime = millis();
    if (readStopTime == 0) readStopTime = 1;  // évite la valeur sentinelle
  }
}

bool minuterieEchouee() {
  if (readStopTime != 0 && (millis() - readStopTime > STOP_MILLIS)) {
    readStopTime = 0;
    return true;
  }
  return false;
}

// ─── Mesure peak d'un échantillon ─────────────────────────────────────────────
// Retourne la valeur ADC maximale sur SAMPLE_WINDOW_MS millisecondes.
uint16_t mesurePeak() {
  uint32_t debut = millis();
  uint16_t peakVal = 0;

  while (millis() - debut < SAMPLE_WINDOW_MS) {
    uint16_t s = analogRead(SENSEUR_COURANT);
    if (s > peakVal) peakVal = s;
  }
  return peakVal;  // toujours 0–1023, pas besoin de isinf()
}

// ─── Détection de surcourant ──────────────────────────────────────────────────
// Moyenne les pics ADC sur MESURE_DUREE_MS, calcule le courant réel et
// compare à LIMITE_COURANT. Retourne true si le courant est trop élevé.
bool courantDetecte_old() {
  float vcc_mV = (float)lireVCC_mV();  // ex. 5012 mV
  float vcc = vcc_mV / 1000.0f;        // en Volts

  uint32_t somme = 0;
  uint32_t count = 0;
  uint32_t debut = millis();

  while (millis() - debut < MESURE_DUREE_MS) {
    somme += mesurePeak();
    count++;
  }

  if (count == 0) return false;  // garde-fou

  float adcMoyen = (float)somme / count;
  float tension = adcMoyen * (vcc / 1023.0f);  // Volts
  float repos = vcc / 2.0f;                    // Point zéro ACS712
  float courant = (tension - repos) / 0.185;   // 0.185 V/A pour ACS712-5A

#ifdef DEBUG
  Serial.print(F("VCC: "));
  Serial.print(vcc, 3);
  Serial.print(F("V  Tension: "));
  Serial.print(tension, 3);
  Serial.print(F("V  Courant: "));
  Serial.print(courant, 3);
  Serial.println(F(" A"));
#endif
  //première lecture

  return (fabsf(courant) > LIMITE_COURANT);
}
// ─── Remplacer courantDetecte() par deux fonctions ────────────────────────

// 1. Capture le repos AVANT d'enclencher le relais (appel unique)
void capturerBaseline() {
  float vcc = lireVCC_mV() / 1000.0f;
  uint32_t somme = 0, count = 0;
  uint32_t debut = millis();

  while (millis() - debut < 200) {  // 200 ms suffisent pour le repos
    somme += mesurePeak();
    count++;
  }

  float adcMoyen = (float)somme / count;
  float tension = adcMoyen * (vcc / 1023.0f);
  float repos = vcc / 2.0f;
  baselineCourant = (tension - repos) / 0.185f;  // courant "zéro" réel
}

// 2. Mesure l'écart par rapport à la baseline
bool courantDetecte() {
  float vcc = lireVCC_mV() / 1000.0f;
  uint32_t somme = 0, count = 0;
  uint32_t debut = millis();

  while (millis() - debut < MESURE_DUREE_MS) {
    somme += mesurePeak();
    count++;
  }

  if (count == 0) return false;

  float adcMoyen = (float)somme / count;
  float tension = adcMoyen * (vcc / 1023.0f);
  float repos = vcc / 2.0f;
  float courant = (tension - repos) / 0.185f;

  float deviation = fabsf(courant - baselineCourant);  // ← différentiel
#ifdef DEBUG
  Serial.print(F("Baseline: "));
  Serial.print(baselineCourant, 3);
  Serial.print(F(" A  Courant: "));
  Serial.print(courant, 3);
  Serial.print(F(" A  Déviation: "));
  Serial.print(deviation, 3);
  Serial.println(F(" A"));
#endif

  return (deviation > LIMITE_COURANT);
}

// ─── Lecture VCC réel via référence interne 1.1 V ────────────────────────────
// Retourne VCC en millivolts (ex. 5012).
uint16_t lireVCC_mV() {
  // Sélectionner la référence interne 1.1 V sur le canal bandgap
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);             // Laisser la tension se stabiliser
  ADCSRA |= _BV(ADSC);  // Lancer la conversion
  while (bit_is_set(ADCSRA, ADSC))
    ;  // Attendre la fin

  uint16_t result = ADCL | (ADCH << 8);
  // VCC (mV) = 1,1 V × 1023 × 1000 / résultat ADC
  return (uint16_t)(1125300UL / result);
}
