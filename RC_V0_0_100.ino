/*

-------------------------------------------------------------
>>> Comment le configurer ou calibrer la course des gaz ? <<<
-------------------------------------------------------------
Si vous avez besoin de calibrer la course des gaz ou de modifier un paramètre (comme le frein moteur)
la documentation officielle de SonicModell indique la procédure "à l'oreille" :

Sécurité : Retirez l'hélice !

Allumez votre émetteur radio et mettez le manche des gaz au maximum (100%).
Branchez la batterie LiPo sur l'ESC. Le moteur va émettre 2 bips courts (BEEP-BEEP).

Pour une simple calibration des gaz : Abaissez immédiatement le manche des gaz tout en bas (0%).
Le moteur émettra une série de bips pour compter les cellules de votre batterie, puis un bip long. C'est calibré !

Pour entrer dans les menus de configuration : Si vous laissez le manche en haut après les deux premiers bips, au bout de 5 secondes.
le moteur va chanter une petite mélodie (12321) et va faire défiler des options en boucle (1 bip pour le Frein
2 bips pour le type de batterie, etc.). Vous baissez le manche au moment de l'option voulue pour la valider.

*/


/*
---------------------------
>>> V0.0.82 du 18/07/26 <<<
---------------------------
- MIROIR de PL v0.0.76 : TelemetryData reçoit le champ failSafeCount
  (uint32_t) en VRAIE fin de struct. ATTENTION CRITIQUE : nécessite
  PL v0.0.76 ou supérieur avec la struct strictement identique.
- Ajout d'une ligne "AVION | Failsafes détectés (depuis boot avion): N"
  dans debugTransmitterData(), section "retour avion" : permet de vérifier
  si le compteur de failsafes CÔTÉ AVION progresse ou stagne, indépendamment
  du monitoring série côté RC (utile notamment pour débrancher l'USB de
  l'avion et voir si les failsafes chutent en labo).

*/

/*
---------------------------
>>> V0.0.81 du 18/07/26 <<<
---------------------------
- MIROIR de PL v0.0.74 : TelemetryData reçoit 3 nouveaux champs en VRAIE fin
  de struct (groundSpeedKmh, pressure, hdop), à la suite de "autoLanding".
  ATTENTION CRITIQUE : nécessite PL v0.0.74 ou supérieur avec la struct
  TelemetryData strictement identique, sinon désalignement du paquet LoRa
  (même risque que documenté pour les champs rudder/gimbal/autoLanding).
- Ajout, dans debugTransmitterData() (section "retour avion"), de 3 nouvelles
  lignes BARO / IMU ATT / GPS reprenant les données renvoyées par l'avion,
  sous la même forme que debugControlData() côté PL. La ligne GPS affiche
  Etat/Sats/HDOP/Lat/Lon/Vitesse/Cap ; l'Etat GPS côté RC ne distingue que
  FIX OK / RECHERCHE (2 états, contre 3 côté PL qui a accès à la trame NMEA
  brute).

*/

/*
---------------------------
>>> V0.0.80 du 15/07/26 <<<
---------------------------
- Ligne [TX DEBUG XIAO RC + BLE] : ajout du numéro de version (SOFTWARE_VERSION)
  et de la puissance LoRa courante (currentTxPower).
- NOUVEAU : gestion automatique de la puissance LoRa selon la présence USB.
  Fonction updateTxPower(), appelée en boucle dans TaskLoRaSend : puissance
  ramenée à PWR_LABO (1) quand (bool)Serial est vrai (port USB ouvert côté PC),
  remise à PWR_FLIGHT (max) sinon. ATTENTION : (bool)Serial détecte un hôte qui
  a OUVERT le port série, pas seulement la présence physique du câble USB
  (alimentation seule sans port ouvert = non détecté). currentTxPower passé en
  volatile (lu/écrit depuis TaskLoRaSend et TaskReadInputs/debug).
- Suppression complète de la ligne [STICKS] (Ail/Elv/Rud) dans debugTransmitterData().
- Duplication des lignes [RUDDER], [GIMBAL], [MODES] et [VOLETS] dans la section
  télémétrie, affichant désormais les valeurs RENVOYÉES par l'avion (telem.*)
  au lieu des valeurs envoyées par la RC (packet.*). La ligne [VOLETS] côté
  "commandes envoyées" ne montre plus que la commande TX (le retour RX a été
  déplacé dans la section télémétrie).
- AJOUT (RC uniquement, struct TelemetryData) : champs rudder, gimbalPan,
  gimbalTilt, autoLanding en fin de struct, pour porter ces valeurs de retour.
  ATTENTION CRITIQUE : contrairement à ControlData, ici la RC est RÉCEPTRICE.
  Cet ajout REQUIERT une mise à jour STRICTEMENT IDENTIQUE de la struct
  TelemetryData côté avion (PL), avec ces mêmes 4 champs remplis et envoyés en
  fin de struct, AVANT tout vol avec ce firmware RC. Sans cela, radio.receive()
  réclamera plus d'octets que ce que PL transmet réellement, ce qui risque de
  casser la réception de TOUTE la télémétrie (pas seulement ces 4 champs) —
  c'est le scénario historique déjà rencontré avec TelemetryData.
- Confirmation : dans [MODES] (section "commandes envoyées"), "Auto Landing"
  a toujours été packet.autoLanding, donc déjà la valeur ENVOYÉE par la RC,
  pas un retour avion. Le nouveau [MODES] de la section télémétrie affiche
  quant à lui telem.autoLanding (retour avion, cf. avertissement ci-dessus).

*/

/*
---------------------------
>>> V0.0.79 du 13/07/26 <<<
---------------------------
- SUPPRESSION de toute la logique "Gear" (train d'atterrissage) : le train est
  fixe sur cet avion, la broche MCP23017 n°0 et le champ ControlData.gear
  n'étaient donc plus utiles. Retiré : #define MCP_PIN_GEAR, packet.gear (champ
  struct ET lecture MCP).
- Le champ "autoLanding" (ajouté en V0.0.78 sur la broche libre n°10) est
  RAMENÉ sur la broche n°0, désormais libérée par la suppression du gear.
  La boucle d'initialisation pinMode() repasse donc de <=10 à <=9 (10 broches
  au total, 0 à 9), comme avant l'ajout du gear.
- ATTENTION CRITIQUE (désalignement binaire ControlData) : "gear" était situé
  AU MILIEU de la struct (entre autoPilot et lights), pas en fin. Sa
  suppression décale donc en binaire tous les champs suivants (lights,
  returnToHome, trimRudder, trimElevator, gimbalPan, gimbalTilt). La struct
  ControlData côté avion (PL_V0_0_71.ino) DOIT être mise à jour à l'identique
  (suppression du même champ "gear", même position) AVANT de voler avec ce
  firmware RC, sinon la RC et l'avion ne seront plus synchronisés : les trims
  et la gimbal seraient corrompus. Voir le correctif fourni pour PL_V0_0_72.ino.

---------------------------
>>> V0.0.78 du 13/07/26 <<<
---------------------------
- NOUVEAU : ajout du bouton on/off "Auto Landing" (MCP23017, broche libre n°10)
  qui donne l'ordre à l'avion de passer en mode "Atterrissage Automatique".
  Fonctionne exactement comme les boutons Autopilot (AP) et Return-to-Home (RTH)
  déjà existants : lecture via readGPIOAB(), logique inversée (bouton au repos
  = contact fermé = 1 côté MCP, donc "!" pour obtenir true = actionné).
- Ajout du champ "autoLanding" (bool) en FIN de la struct ControlData. Comme ce
  champ est ajouté à la fin, et que côté avion (PL v0.0.71) la struct ControlData
  n'a pas encore ce champ, radio.receive() côté avion continuera de fonctionner
  normalement : il lit sizeof(ControlData) octets selon SA PROPRE définition
  (plus courte), donc il ignore simplement l'octet supplémentaire envoyé par la
  RC. AUCUN désalignement binaire, contrairement au cas historique de
  TelemetryData (où le problème venait d'un champ déclaré des deux côtés mais
  rempli seulement d'un côté). L'avion ne réagira au bouton qu'à partir de la
  prochaine étape (mise à jour de PL avec la structure ControlData identique +
  logique de réception du mode "Automatic Landing").
- Ajout de la ligne "Auto Landing" dans debugTransmitterData(), aux côtés des
  lignes Autopilot/RTH existantes.
- Ajout du champ "land" dans le payload JSON BLE envoyé à la tablette Android,
  pour préparer l'affichage du statut sur l'interface web.

---------------------------
>>> V0.0.77 du 07/07/26 <<<
---------------------------
- CORRECTIF : ajout du champ "altitude" restait deja present dans TelemetryData
  mais n'etait jamais envoye par l'avion avant PL v0.0.68 (desalignement du
  paquet binaire LoRa). Necessite PL v0.0.68 ou superieur.
- Ajout des champs attRoll/attPitch/attYawRate/imuValid dans TelemetryData
  (attitude IMU renvoyee par l'avion, PL v0.0.68+).
- NOUVEAU : liaison I2C dediee (Wire, bus existant PIN_SDA/PIN_SCL) vers un
  XIAO ESP32S3 separe portant un Round Display GC9A01, monte en extension sur
  la telecommande, afin d'afficher un horizon artificiel. La telecommande
  reste maitre I2C ; l'afficheur ecoute a l'adresse 0x50 (voir HorizonLinkData).
  Envoi throttle a ~20 Hz (DISPLAY_PUSH_INTERVAL_MS), avec flag "linkOk" base
  sur la fraicheur reelle de la derniere telemetrie recue (pas seulement le
  cycle en cours), pour permettre a l'afficheur de detecter une perte de lien
  avion<->RC meme si la fonction est appelee en continu.

---------------------------
>>> V0.0.75 du 30/06/26 <<<
---------------------------
- Ajout d'une ligne dédiée [RUDDER] dans debugTransmitterData(), isolée des autres sticks
  pour faciliter le diagnostic (le rudder utilise un double-mapping et une zone morte,
  contrairement aux autres axes).
- Ajout d'une ligne de séparation visuelle avant les données reçues de l'avion (télémétrie
  et signaux radio), pour bien isoler "ce que la télécommande envoie" de "ce que l'avion renvoie".

---------------------------
>>> V0.0.74 du 30/06/26 <<<
---------------------------
- Correction throttle : suppression du seuil de coupure forcée (mappedThr > 4055 -> 0).
  Avec l'inversion du sens introduite en v0.0.73, ce seuil se déclenchait par erreur
  en HAUT de course (pleins gaz) au lieu du bas, provoquant une chute brutale à 0
  dès que la valeur dépassait 4055 (bruit ADC / jeu mécanique en fin de course).
  Le potentiomètre redescend déjà naturellement à 0 en bas de course réelle,
  donc ce garde-fou n'était plus nécessaire après l'inversion.

---------------------------
>>> V0.0.73 du 30/06/26 <<<
---------------------------
- Inversion du sens de rotation du potentiomètre Throttle (gaz augmentent en tournant
  dans le sens horaire). MIN/MAX laissés dans l'ordre croissant pour ne pas casser
  constrain(), inversion faite uniquement dans le map() de sortie (4095 -> 0).
- Correction du seuil de coupure moteur (mappedThr > 4055 au lieu de < 40), adapté
  au sens de lecture désormais inversé.
- Mise à jour des valeurs de calibration Gimbal Pan/Tilt avec les vraies valeurs
  mesurées en atelier (voir débug série [GIMBAL RAW]).

---------------------------
>>> V0.0.61-Fixed       <<<
---------------------------
- Restauration des inclusions de bibliothèques BLE oubliées.
- Intégration de la calibration avancée pour potentiomètre 10K (Rudder).
- Ajout du double-mapping (courbe brisée) et d'une zone morte au centre.
- Préservation intégrale de FreeRTOS, du BLE et du Mutex.

---------------------------
>>> V0.0.60-Calibrated <<<
---------------------------
- Intégration de la calibration avancée pour potentiomètre 10K (Rudder).
- Ajout du double-mapping (courbe brisée) et d'une zone morte au centre.
- Préservation intégrale de FreeRTOS, du BLE et du Mutex.

---------------------------
>>> V0.0.58 du 10/06/26 <<<
---------------------------
- Ajout des états Autopilot ("ap") et Return-to-Home ("rth") dans le payload JSON BLE.
- Sécurisation de la lecture de l'état des switchs via le Mutex partagé dans TaskLoRaSend.

---------------------------
>>> V0.0.56 du 08/06/26 <<<
---------------------------
- Ajout du SNR de l'avion dans la structure TelemetryData.
- Envoi du RSSI avion ("rssi_rx") et du SNR avion ("snr_rx") vers l'interface Web BLE.

---------------------------
>>> V0.0.54 du 06/06/26 <<<
---------------------------
- Désactivation totale du modem Wi-Fi au démarrage (économie d'énergie / réduction des interférences).
- Support complet FrSky M9 (Effet Hall) avec variables de calibration pour les 4 axes de l'ADS1115.
- Optimisation majeure de TaskReadInputs : Lecture globale du registre du MCP23017 (readGPIOAB) 
  en une seule transaction I2C pour libérer le bus.

*/

#include <RadioLib.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_ADS1X15.h>
#include "esp_wifi.h"  // Requis pour la désactivation totale du Wi-Fi

// --- BIBLIOTHÈQUES POUR LE BLE (RESTAURÉES) ---
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// --- CONFIGURATION VERSION ---
#define SOFTWARE_VERSION "v0.0.100"

// --- CONFIGURATION IDENTIFIANTS BLE (UUIDs) ---
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// --- CONFIGURATION MATÉRIELLE ---
#define LORA_MISO 8
#define LORA_SCK 7
#define LORA_MOSI 9
#define LORA_CS 41
#define LORA_RESET 42
#define LORA_BUSY 40
#define LORA_DIO1 39
#define LORA_ANT_SW 38

#define PIN_SDA 5
#define PIN_SCL 6

// --- SUPPRIMÉ V0.0.79 : #define MCP_PIN_GEAR 0 (train fixe, plus de logique gear) ---
#define MCP_PIN_LIGHTS 1
#define MCP_PIN_FLAP1 2
#define MCP_PIN_FLAP2 3
#define MCP_PIN_AP 4
#define MCP_PIN_RTH 5
#define MCP_PIN_TRIM_E_UP 6
#define MCP_PIN_TRIM_E_DN 7
#define MCP_PIN_TRIM_R_L 8
#define MCP_PIN_TRIM_R_R 9
// --- MODIFIÉ V0.0.79 : bouton on/off "Auto Landing" (Atterrissage Automatique) ---
// Reprend la broche 0, libérée par la suppression du gear (train fixe).
// Câblage : interrupteur entre la broche et GND, pull-up interne activée
// dans TaskReadInputs (comme pour toutes les autres broches).
#define MCP_PIN_AUTOLAND 0

// --- CONFIGURATION & CALIBRATION GIMBALS & POTENTIOMÈTRES ---
int16_t AIL_MIN = 800;
int16_t AIL_MAX = 26300;
int16_t ELV_MIN = 26300;
int16_t ELV_MAX = 800;
int16_t THR_MIN = 800;
int16_t THR_MAX = 26300;

// Variables dédiées au potentiomètre 10K du Rudder
int16_t RUD_MIN = 800;       // Valeur brute à fond d'un côté (0 dans le paquet)
int16_t RUD_MAX = 26300;     // Valeur brute à fond de l'autre côté (4095 dans le paquet)
int16_t RUD_CENTRE = 19662;  // Centre physique calculé d'après tes tests au neutre
int16_t RUD_DEADBAND = 200;  // Zone morte (tolérance) pour stabiliser le potard au centre

// --- CONFIGURATION & CALIBRATION GIMBAL (second ADS1115 à l'adresse 0x49) ---
// Pan  = rotation horizontale (gauche/droite) — branché sur AIN0 du second ADS1115
// Tilt = inclinaison verticale (haut/bas)      — branché sur AIN1 du second ADS1115
// Valeurs mesurées en atelier le 30/06/26 (voir débug série [GIMBAL RAW]).
// MIN/MAX toujours dans l'ordre croissant (valeurs brutes réelles) pour ne pas
// casser constrain() ; l'inversion éventuelle du sens se fait dans le map() de sortie.
int16_t GIMBAL_PAN_MIN = 6054;    // Manche à fond gauche
int16_t GIMBAL_PAN_MAX = 21421;   // Manche à fond droit
int16_t GIMBAL_TILT_MIN = 7081;   // Manche en haut (valeur basse, axe physique inversé)
int16_t GIMBAL_TILT_MAX = 21075;  // Manche en bas  (valeur haute, axe physique inversé)

// --- INSTANTIATION ---
SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RESET, LORA_BUSY);
Adafruit_MCP23X17 mcp;
Adafruit_ADS1115 ads;   // ADS1115 principal  — adresse I2C 0x48 — Ailerons/Elevator/Throttle/Rudder
Adafruit_ADS1115 ads2;  // ADS1115 secondaire — adresse I2C 0x49 — Gimbal Pan / Gimbal Tilt

// Variables BLE
BLECharacteristic *pCharacteristic = nullptr;
volatile bool deviceConnected = false;

// --- STRUCTURES ---
const int PWR_LABO = 1;
const int PWR_FLIGHT = 22;  // Valeur max
volatile int currentTxPower = PWR_FLIGHT;
// AJOUT V0.0.80 : mémorise le dernier état USB connu pour ne changer la
// puissance LoRa QUE lors d'une transition (évite d'appeler setOutputPower()
// à chaque itération de TaskLoRaSend).
volatile bool lastUsbConnected = false;
const uint32_t MY_AIRCRAFT_ID = 0x55AA3322;

struct __attribute__((packed)) ControlData {
  uint32_t aircraftId;
  uint32_t msgId;
  int16_t ailerons, elevator, rudder, flaps, throttle;
  // --- MODIFIÉ V0.0.79 : "gear" retiré (train fixe, plus de logique).
  // ATTENTION : ce champ était AU MILIEU de la struct -> sa suppression décale
  // tous les champs suivants en binaire. La struct ControlData côté avion
  // (PL) DOIT être modifiée à l'identique (voir PL_V0_0_72.ino). ---
  bool autoPilot, lights;
  bool returnToHome;
  int16_t trimRudder, trimElevator;
  int16_t gimbalPan;   // Axe horizontal de la gimbal (0-4095, centre = 2048)
  int16_t gimbalTilt;  // Axe vertical   de la gimbal (0-4095, centre = 2048)
  // --- AJOUT V0.0.78 : ordre "Atterrissage Automatique" ---
  // Champ ajouté en FIN de struct : côté avion (PL v0.0.71/72), ControlData ne
  // possède pas encore ce champ, radio.receive() y lit donc sizeof(ControlData)
  // selon SA définition (plus courte) et ignore simplement cet octet
  // supplémentaire. Aucun risque de désalignement pour CE champ précis (il est
  // en fin de struct). L'avion devra recevoir la struct identique (même champ,
  // même ordre) pour exploiter cette commande — prévu à l'étape suivante.
  bool autoLanding;
} packet;

struct __attribute__((packed)) TelemetryData {
  float batteryVoltage;
  float currentRSSI;
  float currentSNR;
  float freqError;
  uint32_t flightTime;
  float lat;
  float lon;
  uint16_t heading;
  uint8_t satellites;
  uint8_t gpsStatus;
  int flaps;
  bool autoPilot;
  bool returnToHome;
  int16_t ail1PWM;
  int16_t ail2PWM;
  float altitude;
  // --- AJOUT V0.0.77 : attitude IMU renvoyee par l'avion (PL v0.0.68+) ---
  // Doit rester STRICTEMENT identique (memes champs, meme ordre) a la struct
  // TelemetryData cote avion, sinon le paquet binaire LoRa sera mal decode.
  float attRoll;
  float attPitch;
  float attYawRate;
  bool imuValid;
  // --- AJOUT V0.0.80 (RC uniquement, EN ATTENTE du miroir côté PL) ---
  // Objectif : afficher dans la section télémétrie les valeurs RUDDER, GIMBAL
  // et AUTO LANDING telles que RENVOYÉES par l'avion (et non plus seulement
  // ce que la RC a envoyé).
  // ATTENTION CRITIQUE - SENS DE TRANSMISSION INVERSÉ PAR RAPPORT À ControlData :
  // ici la RC est RÉCEPTRICE (radio.receive((uint8_t*)&telem, sizeof(TelemetryData))).
  // Si PL n'envoie pas EXACTEMENT ces 4 champs, dans le même ordre, en fin de
  // SA struct TelemetryData, la RC va réclamer plus d'octets que ce que
  // l'avion transmet réellement : c'est le scénario historique qui a déjà
  // cassé la télémétrie par le passé (cf. note en tête de fichier). NE PAS
  // FLASHER sur la RC avant mise à jour identique de PL_V0_0_XX.ino.
  int16_t rudder;      // Position rudder appliquée côté avion
  int16_t gimbalPan;   // Position gimbal Pan appliquée côté avion
  int16_t gimbalTilt;  // Position gimbal Tilt appliquée côté avion
  bool autoLanding;    // Confirmation "Atterrissage Automatique" actif côté avion
  // --- AJOUT V0.0.81 : MIROIR de PL v0.0.74 (BARO/IMU/GPS détaillés) ---
  // ATTENTION CRITIQUE : ces 3 champs doivent être STRICTEMENT identiques
  // (mêmes champs, même ordre) à ceux ajoutés en fin de la struct TelemetryData
  // côté avion (PL v0.0.74), sinon le paquet binaire LoRa sera mal décodé (même
  // scénario historique que pour les 4 champs ci-dessus). NE PAS FLASHER sur la
  // RC avant mise à jour identique de PL_V0_0_74.ino ou supérieur.
  float groundSpeedKmh;  // Vitesse sol GPS (km/h) - existait déjà côté PL, absente ici jusqu'ici
  float pressure;        // hPa, pression brute BMP280
  float hdop;             // Dilution de précision horizontale GPS
  // --- AJOUT V0.0.82 : MIROIR de PL v0.0.76. Compteur de failsafes détectés
  // côté avion depuis son démarrage (transitions perte de signal LoRa).
  // Permet de vérifier à distance si le compteur progresse (signal réellement
  // instable) ou stagne (les "failsafes" affichés ailleurs venaient d'autre
  // chose, ex. ralentissement du débug série).
  uint32_t failSafeCount;
} telem;

// --- AJOUT V0.0.77 : LIAISON I2C DEDIEE VERS LE XIAO DE L'AFFICHEUR ROND ---
// La telecommande reste MAITRE I2C. L'afficheur (XIAO ESP32S3 separe, monte en
// extension sur la telecommande) ecoute en ESCLAVE sur un DEUXIEME bus I2C
// (Wire1, broches D0/D6 liberees via le micro-switch KE du Round Display),
// distinct du bus interne D4/D5 de l'afficheur (tactile CST816S + RTC).
// Cablage : RC pin5 (SDA, PIN_SDA) -> Afficheur D0 | RC pin6 (SCL, PIN_SCL) -> Afficheur D6 | GND commun.
// On reutilise le bus I2C existant de la RC (deja maitre du MCP23017 0x20 et
// des ADS1115 0x48/0x49) : l'afficheur devient simplement un 3e peripherique
// esclave sur ce meme bus, a une adresse libre.
#define DISPLAY_I2C_ADDR 0x50
#define DISPLAY_LINK_TIMEOUT_MS 1000  // Au-dela, on considere le lien avion<->RC perdu

struct __attribute__((packed)) HorizonLinkData {
  uint32_t seq;          // Incremente a chaque envoi : permet a l'afficheur de detecter un paquet neuf
  float roll;            // degres
  float pitch;           // degres
  float yawRate;         // degres/s
  float altitude;        // metres
  float batteryVoltage;  // V avion
  uint16_t heading;      // degres
  uint8_t satellites;
  uint8_t gpsStatus;
  int8_t rssi;  // dBm, qualite liaison avion->RC (arrondi)
  bool imuValid;
  bool linkOk;  // false si aucune telemetrie recue depuis DISPLAY_LINK_TIMEOUT_MS
  bool autoPilot;
  bool returnToHome;
};

uint32_t horizonSeq = 0;
unsigned long lastDisplayPush = 0;
const unsigned long DISPLAY_PUSH_INTERVAL_MS = 50;  // ~20 Hz, largement suffisant pour un horizon

SemaphoreHandle_t mutex;
uint32_t counter = 0;
uint32_t lastDebug = 0;
volatile float localRSSI = -127.0;
volatile float localSNR = 0.0;
volatile unsigned long lastPacketTime = 0;  // AJOUT V0.0.77 : fraicheur de la derniere telemetrie recue

int16_t currentTrimElevator = 0;
int16_t currentTrimRudder = 0;
unsigned long lastTrimPulse = 0;

// Prototypes
void TaskLoRaSend(void *pvParameters);
void TaskReadInputs(void *pvParameters);
void setupLoRaFastMode();

// Callbacks de connexion BLE
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    Serial.println(F("[BLE] Tablette connectée !"));
  };
  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    Serial.println(F("[BLE] Tablette déconnectée. Relance du Scan..."));
    BLEDevice::startAdvertising();
  }
};

// AJOUT V0.0.80 : bascule la puissance LoRa selon la présence USB.
// Détection basée sur (bool)Serial (USB CDC natif de l'ESP32-S3), le même
// signal déjà utilisé plus bas par debugTransmitterData() ("if (!Serial) return;").
// ATTENTION : ce booléen reflète le fait qu'un hôte a OUVERT le port série
// (typiquement un moniteur série / une appli sur le PC), pas uniquement la
// présence physique du câble USB. Si la télécommande est branchée sur une
// simple source USB (chargeur, sans port ouvert côté PC), la puissance
// restera au maximum. Si ce cas doit être couvert, il faudra détecter le
// VBUS matériellement (broche dédiée) plutôt que via Serial.
void updateTxPower() {
  bool usbNow = (bool)Serial;
  if (usbNow == lastUsbConnected) return;  // pas de changement, rien à faire

  lastUsbConnected = usbNow;
  currentTxPower = usbNow ? PWR_LABO : PWR_FLIGHT;

  if (xSemaphoreTake(mutex, pdMS_TO_TICKS(5))) {
    radio.setOutputPower(currentTxPower);
    xSemaphoreGive(mutex);
  }

  if (Serial) {
    Serial.printf("[USB] %s -> Puissance LoRa réglée à %d\n",
                  usbNow ? "PC détecté" : "Débranché", currentTxPower);
  }
}

void pushToDisplay(bool freshTelemetry) {
  if (millis() - lastDisplayPush < DISPLAY_PUSH_INTERVAL_MS) return;
  lastDisplayPush = millis();

  HorizonLinkData link;
  if (xSemaphoreTake(mutex, pdMS_TO_TICKS(5))) {
    link.roll = telem.attRoll;
    link.pitch = telem.attPitch;
    link.yawRate = telem.attYawRate;
    link.altitude = telem.altitude;
    link.batteryVoltage = telem.batteryVoltage;
    link.heading = telem.heading;
    link.satellites = telem.satellites;
    link.gpsStatus = telem.gpsStatus;
    link.rssi = (int8_t)constrain((int)localRSSI, -128, 127);
    link.imuValid = telem.imuValid;
    link.autoPilot = telem.autoPilot;
    link.returnToHome = telem.returnToHome;
    xSemaphoreGive(mutex);
  }
  // linkOk : reflete la fraicheur de la derniere telemetrie recue de l'avion,
  // pas seulement le cycle courant, pour que l'afficheur affiche "NO SIGNAL"
  // meme si le paquet LoRa retour se perd pendant plusieurs cycles.
  link.linkOk = (millis() - lastPacketTime) < DISPLAY_LINK_TIMEOUT_MS;
  link.seq = horizonSeq++;

  Wire.beginTransmission(DISPLAY_I2C_ADDR);
  Wire.write((uint8_t *)&link, sizeof(link));
  uint8_t err = Wire.endTransmission();
  if (err != 0 && (horizonSeq % 40 == 0)) {
    // Log occasionnel seulement (pas a chaque paquet) pour ne pas noyer le port serie
    Serial.printf("[I2C->AFFICHEUR] Erreur transmission (code %d)\n", err);
  }
}

void debugTransmitterData() {
  if (!Serial) return;
  if (xSemaphoreTake(mutex, 0)) {
    // MODIFIÉ V0.0.80 : ajout du numéro de version et de la puissance LoRa courante.
    Serial.printf("\n--- [ TX DEBUG XIAO RC + BLE ] --- (%s | Puissance LoRa: %d) ---\n", SOFTWARE_VERSION, currentTxPower);
    Serial.printf("ID MSG: %u | THR: %4d | BLE Connecté: %s\n", packet.msgId, packet.throttle, deviceConnected ? "OUI" : "NON");
    // --- SUPPRIMÉ V0.0.80 : ligne [STICKS] (Ail/Elv/Rud) retirée à la demande ---
    Serial.printf("[RUDDER] Valeur envoyée: %4d (Centre = 2048)\n", packet.rudder);
    Serial.printf("[GIMBAL] Pan: %4d | Tilt: %4d\n", packet.gimbalPan, packet.gimbalTilt);
    Serial.printf("[MODES]  Autopilot (AP): %s | Return-To-Home (RTH): %s | Auto Landing: %s\n",
                  packet.autoPilot ? "ACTIF" : "OFF",
                  packet.returnToHome ? "ACTIF" : "OFF",
                  packet.autoLanding ? "ACTIF" : "OFF");  // AJOUT V0.0.78 - valeur ENVOYÉE par la RC

    int flapsDegreesTX = 0;
    if (packet.flaps == 1) {
      flapsDegreesTX = 10;  // Niveau 1
    } else if (packet.flaps == 2) {
      flapsDegreesTX = 20;  // Volet niveau 2. NE PAS ALLER AU DELA DE 30 DEGRES. Au delà les volets force mécaniquement sur l'avion.
    }

    // MODIFIÉ V0.0.80 : le retour avion ("Retour Telem RX") est déplacé dans la
    // section télémétrie ci-dessous, cette ligne ne montre plus que la commande TX.
    Serial.printf("[VOLETS] Commande TX: %2d deg (Cran %d)\n", flapsDegreesTX, packet.flaps);

    // --- SÉPARATEUR VISUEL ---
    // Isole la section "commandes envoyées" (sticks, rudder, gimbal, modes, volets)
    // ci-dessus, de la section "retour avion" ci-dessous (télémétrie + qualité de signal).
    Serial.println(F("..............................................."));

    Serial.printf("[TELEM]     Avion Vbatt: %.2fV | Satellites: %d | Cap: %d deg\n", telem.batteryVoltage, telem.satellites, telem.heading);

    // --- AJOUT V0.0.80 : RUDDER/GIMBAL/MODES/VOLETS dupliqués avec les valeurs
    // RENVOYÉES par l'avion (telem.*, et non plus packet.*).
    // ATTENTION : telem.rudder / telem.gimbalPan / telem.gimbalTilt / telem.autoLanding
    // sont des champs AJOUTÉS EN FIN de TelemetryData qui n'existent pas encore côté PL.
    // Tant que PL n'a pas été mis à jour à l'identique, NE PAS COMPTER sur ces valeurs
    // (voir avertissement détaillé au niveau de la struct TelemetryData plus haut).
    Serial.printf("[RUDDER]    Retour avion: %4d\n", telem.rudder);
    Serial.printf("[GIMBAL]    Retour avion -> Pan: %4d | Tilt: %4d\n", telem.gimbalPan, telem.gimbalTilt);
    Serial.printf("[MODES]     Retour avion -> Autopilot (AP): %s | Return-To-Home (RTH): %s | Auto Landing: %s\n",
                  telem.autoPilot ? "ACTIF" : "OFF",
                  telem.returnToHome ? "ACTIF" : "OFF",
                  telem.autoLanding ? "ACTIF" : "OFF");
    Serial.printf("[VOLETS]    Retour avion: %d deg\n", telem.flaps);

    // --- AJOUT V0.0.81 : BARO / IMU ATT / GPS renvoyés par l'avion (PL v0.0.74+),
    // affichés sous la même forme que debugControlData() côté PL. ---
    Serial.printf("BARO    | Pression: %7.2f hPa | Altitude: %6.1f m\n", telem.pressure, telem.altitude);
    Serial.printf("IMU ATT | Roll: %+7.2f ° | Pitch: %+7.2f ° | YawRate: %+7.2f °/s\n", telem.attRoll, telem.attPitch, telem.attYawRate);
    // NOTE : contrairement à PL (qui distingue ABSENT/RECHERCHE/FIX OK via gps.charsProcessed()),
    // la RC ne reçoit que telem.gpsStatus (0/1) : pas de distinction "pas de trame" vs "recherche en cours".
    Serial.printf("GPS     | Etat: %-9s | Sats: %2d | HDOP: %4.1f | Lat: %9.5f | Lon: %9.5f | Vit: %5.1f km/h | Cap: %5.1f°\n",
                  telem.gpsStatus ? "FIX OK" : "RECHERCHE",
                  telem.satellites, telem.hdop, telem.lat, telem.lon, telem.groundSpeedKmh, (float)telem.heading);
    // --- AJOUT V0.0.82 : compteur de failsafes CÔTÉ AVION (indépendant du lien
    // RC->PC), pour diagnostiquer à distance sans le port série de l'avion.
    Serial.printf("AVION   | Failsafes détectés (depuis boot avion): %lu\n", (unsigned long)telem.failSafeCount);

    Serial.println(F("..............................................."));
    Serial.printf("[SIGNAL UPLINK]   Qualité reçue PAR l'avion       -> RSSI: %.1f dBm | SNR: %.1f dB\n", telem.currentRSSI, telem.currentSNR);
    Serial.printf("[SIGNAL DOWNLINK] Qualité reçue PAR la téléc.     -> RSSI: %.1f dBm | SNR: %.1f dB\n", localRSSI, localSNR);
    Serial.println(F("---------------------------------------------"));
    xSemaphoreGive(mutex);
  }
}

void setup() {
  Serial.begin(115200);

  esp_wifi_stop();
  esp_wifi_deinit();

  pinMode(LORA_ANT_SW, OUTPUT);
  digitalWrite(LORA_ANT_SW, HIGH);

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  mutex = xSemaphoreCreateMutex();

  if (!mcp.begin_I2C(0x20)) {
    Serial.println(F("Erreur: MCP23017 introuvable !"));
    while (1)
      ;
  }

  ads.setGain(GAIN_ONE);
  if (!ads.begin(0x48)) {
    Serial.println(F("Erreur: ADS1115 #1 (0x48) introuvable !"));
    while (1)
      ;
  }

  // --- INITIALISATION DU SECOND ADS1115 (GIMBAL) ---
  // Brancher la broche ADDR du module sur VDD pour obtenir l'adresse 0x49.
  // AIN0 -> potentiomètre Pan (horizontal)
  // AIN1 -> potentiomètre Tilt (vertical)
  // AIN2 et AIN3 sont libres pour de futures extensions.
  ads2.setGain(GAIN_ONE);
  if (!ads2.begin(0x49)) {
    Serial.println(F("Erreur: ADS1115 #2 (0x49 - Gimbal) introuvable !"));
    while (1)
      ;
  }

  setupLoRaFastMode();

  // --- INITIALISATION DU SERVEUR BLE ---
  BLEDevice::init("XIAO_GCS_RC");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println(F("[BLE] Prêt pour appairage Web Bluetooth !"));
  if (xSemaphoreTake(mutex, portMAX_DELAY)) {
    packet.aircraftId = MY_AIRCRAFT_ID;
    xSemaphoreGive(mutex);
  }

  xTaskCreatePinnedToCore(TaskLoRaSend, "LoRaSend", 8192, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(TaskReadInputs, "ReadInputs", 8192, NULL, 1, NULL, 1);
}

void setupLoRaFastMode() {
  Serial.print(F("[LoRa] Initialisation... "));
  int state = radio.begin(868.0);
  if (state == RADIOLIB_ERR_NONE) {
    radio.setTCXO(1.6);
    radio.setDio2AsRfSwitch(true);
    radio.setFrequency(868.000600);
    radio.setBandwidth(500.0);
    radio.setSpreadingFactor(7);
    radio.setCodingRate(5);
    radio.setPreambleLength(6);
    radio.setSyncWord(0x12);
    radio.explicitHeader();
    radio.setCRC(true);
    radio.setOutputPower(currentTxPower);
    Serial.println(F("OK !"));
  } else {
    Serial.printf("ÉCHEC (%d)\n", state);
    while (true)
      ;
  }
}

void TaskLoRaSend(void *pvParameters) {
  for (;;) {
    updateTxPower();  // AJOUT V0.0.80 : bascule 1 (USB) / max (débranché)

    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(5))) {
      packet.msgId = counter++;
      xSemaphoreGive(mutex);
    }

    radio.transmit((uint8_t *)&packet, sizeof(ControlData));
    int state = radio.receive((uint8_t *)&telem, sizeof(TelemetryData));
    if (state == RADIOLIB_ERR_NONE) {
      lastPacketTime = millis();  // AJOUT V0.0.77
      if (xSemaphoreTake(mutex, pdMS_TO_TICKS(5))) {
        localRSSI = radio.getRSSI();
        localSNR = radio.getSNR();
        xSemaphoreGive(mutex);
      }

      // --- TRANSMISSION BLE VERS LA TABLETTE ---
      if (deviceConnected && pCharacteristic != nullptr) {
        String json = "{";
        json += "\"batt\":" + String(telem.batteryVoltage, 2) + ",";
        json += "\"gps\":" + String(telem.gpsStatus) + ",";
        json += "\"sats\":" + String(telem.satellites) + ",";
        json += "\"cap\":" + String(telem.heading) + ",";
        json += "\"lat\":" + String(telem.lat, 6) + ",";
        json += "\"lon\":" + String(telem.lon, 6) + ",";
        json += "\"rssi_rx\":" + String((int)telem.currentRSSI) + ",";
        json += "\"snr_rx\":" + String((int)telem.currentSNR) + ",";
        json += "\"rssi_tx\":" + String((int)localRSSI) + ",";
        json += "\"flaps\":" + String(telem.flaps) + ",";
        json += "\"ap\":" + String(telem.autoPilot ? 1 : 0) + ",";
        json += "\"rth\":" + String(telem.returnToHome ? 1 : 0) + ",";
        // --- AJOUT V0.0.78 : statut Auto Landing tel qu'ENVOYÉ par la RC.
        // Pas encore de confirmation avion (viendra avec le champ retour
        // TelemetryData.autoLanding à l'étape suivante) ; on remonte donc
        // pour l'instant l'état de la commande locale (packet), pas telem.
        json += "\"land\":" + String(packet.autoLanding ? 1 : 0);
        json += "}";
        pCharacteristic->setValue(json.c_str());
        pCharacteristic->notify();
      }
    }
    // AJOUT V0.0.77 : pousse l'attitude + statut lien vers l'afficheur rond
    // (throttle interne a ~20 Hz, s'execute meme si aucune nouvelle telemetrie
    // n'est arrivee ce cycle, pour que l'afficheur detecte une perte de lien).
    pushToDisplay(state == RADIOLIB_ERR_NONE);

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void TaskReadInputs(void *pvParameters) {
  // MODIFIÉ V0.0.79 : retour à <=9 (10 broches, 0 à 9). L'extension à 10 de la
  // V0.0.78 n'est plus nécessaire : Auto Landing réutilise la broche 0 libérée
  // par la suppression du gear, au lieu d'une broche 10 supplémentaire.
  for (int i = 0; i <= 9; i++) {
    mcp.pinMode(i, INPUT_PULLUP);
  }

  for (;;) {
    if (xSemaphoreTake(mutex, portMAX_DELAY)) {
      // --- LECTURE DES 4 AXES PRINCIPAUX (ADS1115 #1, adresse 0x48) ---
      int16_t rawAil = ads.readADC_SingleEnded(0);
      int16_t rawElv = ads.readADC_SingleEnded(1);
      int16_t rawThr = ads.readADC_SingleEnded(2);
      int16_t rawRud = ads.readADC_SingleEnded(3);

      // --- LECTURE DES AXES GIMBAL (ADS1115 #2, adresse 0x49) ---
      // Pan  sur AIN0, Tilt sur AIN1 du second convertisseur
      int16_t rawGimbalPan = ads2.readADC_SingleEnded(0);
      int16_t rawGimbalTilt = ads2.readADC_SingleEnded(1);

      packet.ailerons = map(constrain(rawAil, AIL_MIN, AIL_MAX), AIL_MIN, AIL_MAX, 0, 4095);
      packet.elevator = map(constrain(rawElv, ELV_MIN, ELV_MAX), ELV_MIN, ELV_MAX, 0, 4095);

      // Traitement du Rudder avec Double Mapping et Zone Morte au centre
      if (rawRud >= (RUD_CENTRE - RUD_DEADBAND) && rawRud <= (RUD_CENTRE + RUD_DEADBAND)) {
        packet.rudder = 2048;  // Le manche est au repos, on force le centre parfait.
      } else if (rawRud > RUD_CENTRE) {
        // Demi-course haute (vers la gauche mécanique -> 4095)
        packet.rudder = map(constrain(rawRud, RUD_CENTRE, RUD_MAX), RUD_CENTRE, RUD_MAX, 2048, 4095);
      } else {
        // Demi-course basse (vers la droite mécanique -> 0)
        packet.rudder = map(constrain(rawRud, RUD_MIN, RUD_CENTRE), RUD_MIN, RUD_CENTRE, 0, 2048);
      }

      // Throttle : sens inversé en v0.0.73 pour que tourner dans le sens horaire
      // augmente les gaz. THR_MIN/THR_MAX restent dans l'ordre croissant (valeurs
      // brutes réelles) pour ne pas casser constrain() ; l'inversion se fait
      // uniquement dans le map() de sortie (4095, 0 au lieu de 0, 4095).
      int16_t mappedThr = map(constrain(rawThr, THR_MIN, THR_MAX), THR_MIN, THR_MAX, 4095, 0);
      // Seuil de coupure forcée supprimé en v0.0.74 : il se déclenchait par erreur
      // en haut de course (pleins gaz) après l'inversion du sens. Le potentiomètre
      // redescend déjà naturellement à 0 en bas de course réelle (gaz coupés).
      packet.throttle = mappedThr;

      // --- MAPPING DES AXES GIMBAL ---
      // Pan : mapping linéaire normal (0-4095).
      // Tilt : axe physiquement inversé sur cette gimbal (manche en haut = valeur brute basse).
      // On inverse donc uniquement la sortie du map() (4095, 0) pour que "haut" = 4095.
      // GIMBAL_TILT_MIN/MAX restent dans l'ordre croissant pour ne pas casser constrain().
      packet.gimbalPan = map(constrain(rawGimbalPan, GIMBAL_PAN_MIN, GIMBAL_PAN_MAX), GIMBAL_PAN_MIN, GIMBAL_PAN_MAX, 0, 4095);
      packet.gimbalTilt = map(constrain(rawGimbalTilt, GIMBAL_TILT_MIN, GIMBAL_TILT_MAX), GIMBAL_TILT_MIN, GIMBAL_TILT_MAX, 4095, 0);

      uint16_t mcpPins = mcp.readGPIOAB();

      // --- SUPPRIMÉ V0.0.79 : packet.gear = ... (train fixe, plus de logique) ---
      packet.lights = !((mcpPins >> MCP_PIN_LIGHTS) & 0x01);
      packet.autoPilot = !((mcpPins >> MCP_PIN_AP) & 0x01);
      packet.returnToHome = !((mcpPins >> MCP_PIN_RTH) & 0x01);
      // MODIFIÉ V0.0.79 : Auto Landing lu sur MCP_PIN_AUTOLAND, désormais la
      // broche 0 (libérée par le gear). Même logique que AP/RTH (interrupteur
      // au repos = pull-up tiré au repos = bit à 1 -> "!" pour obtenir true
      // uniquement quand l'interrupteur est actionné / fermé à GND).
      packet.autoLanding = !((mcpPins >> MCP_PIN_AUTOLAND) & 0x01);

      if (!((mcpPins >> MCP_PIN_FLAP1) & 0x01)) packet.flaps = 1;
      else if (!((mcpPins >> MCP_PIN_FLAP2) & 0x01)) packet.flaps = 2;
      else packet.flaps = 0;

      if (millis() - lastTrimPulse > 150) {
        if (!((mcpPins >> MCP_PIN_TRIM_E_UP) & 0x01)) {
          currentTrimElevator += 5;
          lastTrimPulse = millis();
        }
        if (!((mcpPins >> MCP_PIN_TRIM_E_DN) & 0x01)) {
          currentTrimElevator -= 5;
          lastTrimPulse = millis();
        }
        if (!((mcpPins >> MCP_PIN_TRIM_R_L) & 0x01)) {
          currentTrimRudder -= 5;
          lastTrimPulse = millis();
        }
        if (!((mcpPins >> MCP_PIN_TRIM_R_R) & 0x01)) {
          currentTrimRudder += 5;
          lastTrimPulse = millis();
        }
        currentTrimElevator = constrain(currentTrimElevator, -500, 500);
        currentTrimRudder = constrain(currentTrimRudder, -500, 500);
      }

      packet.trimElevator = currentTrimElevator;
      packet.trimRudder = currentTrimRudder;

      xSemaphoreGive(mutex);
    }

    if (millis() - lastDebug > 500) {
      debugTransmitterData();
      lastDebug = millis();
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void loop() {
  vTaskDelete(NULL);
}
