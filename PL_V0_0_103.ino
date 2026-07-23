/*

-----------------------------
>>> Sécurisation installation
-----------------------------
Pour sécuriser ton installation, applique l'une de ces deux astuces :
La solution simple (Le condensateur de filtrage) : Prends un condensateur électrolytique (470 ou $1000mF) certifié pour au moins 10V ou 16V)
et branche-le directement sur un canal libre de la PCA9685 (le fil long sur le V+ rouge, le fil court avec la bande blanche sur le GND noir).
Il agira comme un petit réservoir d'énergie pour absorber les pointes de courant des servos et maintenir ton microcontrôleur éveillé.

La solution royale (L'UBEC externe dédie) :
Si tu veux séparer totalement la puissance des servos de l'alimentation du microcontrôleur, tu peux acheter un petit module appelé UBEC 5V indépendant.
Tu le branches directement sur la prise de ta LiPo 3S, et sa sortie 5V servira uniquement à alimenter ton microcontrôleur.
Ainsi, même si les servos mettent la PCA9685 à genoux, ton processeur restera alimenté par une source parfaitement isolée et sécurisée.

*/


/*

-------------------------------------
>>> Version V0.0.103 du 23/07/2026 <<<
-------------------------------------
- CHANGEMENT DE LIBRAIRIE SERVOS : la PCA9685 n'est plus pilotee par
  Adafruit_PWMServoDriver.h mais par la librairie Seeed-PCA9685 (PCA9685.h,
  archive Seeed_PCA9685-master.zip). But : uniformiser sur la librairie Seeed.
  * #include <Adafruit_PWMServoDriver.h>  ->  #include <PCA9685.h>
  * Objet : Adafruit_PWMServoDriver pwm(0x40, I2C_PCA)  ->  ServoDriver pwm;
  * Init  : pwm.begin() + pwm.setPWMFreq(50)  ->  pwm.init(0x7f) + pwm.setFrequency(50)
  * ADRESSE I2C : 0x40 (Adafruit) -> 0x7f (defaut Seeed, confirme par le sketch
    de reference seeed_servo_pca9685_ok.ino pousse par l'utilisateur).
    /!\ Si un jour la carte est re-adressee, ajuster PCA9685_ADDR.
  * On garde setPwm() (comptages precis, cf. plus bas) plutot que setAngle()
    (uint16_t = ~11 us/degre, trop grossier pour des gouvernes pilotees en us).
- BUS I2C : la librairie Seeed (via I2Cdev) communique UNIQUEMENT sur l'objet
  Wire global d'Arduino (aucun moyen de lui passer un TwoWire secondaire). Le
  sketch utilisait un bus dedie "I2C_PCA = TwoWire(1)" sur GPIO5/GPIO6. Comme
  le PCA9685 et tous les autres peripheriques I2C (OLED, BMP280, BMI270, LIDAR)
  partagent physiquement CE MEME bus (GPIO5/GPIO6 = bus I2C par defaut du XIAO
  ESP32-S3), on bascule desormais tout ce bus sur l'objet Wire global :
  "#define I2C_PCA Wire" + "Wire.begin(PCA_SDA, PCA_SCL, 400000)". Aucun
  changement de cablage, un seul bus physique inchange.
- CONVERSION MICROSECONDES : Adafruit exposait writeMicroseconds(canal 0-15,
  us). Seeed-PCA9685 expose setPwm(pin 1-16, on, off) en comptages 12 bits
  (0-4095). On introduit un helper pwmWriteMicroseconds(canal 0-15, us) qui
  (1) convertit les us en comptages a 50 Hz (4096 comptages = 20 ms) et
  (2) decale le canal de +1 (Seeed numerote les canaux de 1 a 16). Les valeurs
  de commande (gouvernes, gimbal) restent exprimees en microsecondes, le
  comportement de vol est donc identique a la v0.0.102.
- ATTENTION MATERIEL : la librairie Seeed force l'usage de l'OSCILLATEUR EXTERNE
  25 MHz du module (bit EXTCLK, cf. PCA9685::restart()). Elle est prevue pour le
  "Grove 16-Channel Servo Driver" Seeed. Si ta carte PCA9685 ne dispose PAS de
  cet oscillateur externe, la sortie PWM sera inactive -> a valider sur banc
  (servos au sol) AVANT tout vol.

*/

/*

-------------------------------------
>>> Version V0.0.100 du 22/07/2026 <<<
-------------------------------------
- AJOUT : intégration du LIDAR TF-Luna (I2C 0x10, sur le bus I2C_PCA partagé
  avec le BMP280 / BMI270 / PCA9685 / OLED) pour la mesure directe de la
  HAUTEUR SOL (AGL) à basse altitude. Fonction updateLidar() calquée sur le
  sketch test_lidar_i2c.ino (lecture des 6 octets du bloc 0x00 : distance /
  signal / température ; seule la distance est conservée, la température est
  ignorée). Une mesure n'est validée que si la force du signal est suffisante
  (>= LIDAR_MIN_STRENGTH, != 0xFFFF saturation) et la distance dans la portée
  fiable (<= LIDAR_MAX_VALID_CM ~ 8 m).
- ATTERRISSAGE AUTOMATIQUE : la séquence utilise désormais la hauteur LIDAR
  comme source AGL prioritaire (getLandingAGL()) dès qu'elle est fiable
  (basse altitude), avec repli automatique sur l'altitude barométrique
  relative en approche haute (hors portée LIDAR). Les transitions de phase
  (FINALE / SOL), l'arrondi (flare) et la réduction des gaz gagnent ainsi en
  précision près du sol, là où le baro dérive le plus.
- debugControlData() : nouvelle ligne "[LIDAR]" affichant uniquement la
  hauteur sol en cm (les autres données du capteur ne sont pas affichées).
- TELEMETRIE : ajout du champ "lidarHeightCm" (uint16_t, hauteur sol en cm) en
  VRAIE fin de struct TelemetryData, renvoyé à la RC. ATTENTION CRITIQUE :
  nécessite RC v0.0.100 ou supérieur avec la struct TelemetryData strictement
  identique (même champ, en fin de struct), sinon désalignement du paquet LoRa
  (même scénario historique que pour les champs précédents). À FLASHER
  CONJOINTEMENT (PL v0.0.100 + RC v0.0.100).
- À AJUSTER ET VALIDER EN VOL PROGRESSIF (basses altitudes d'abord) : la portée
  utile de la TF-Luna (~8 m) et le montage du capteur (visée verticale vers le
  sol) doivent être vérifiés avant tout usage opérationnel.

*/

/*

-------------------------------------
>>> Version V0.0.78 du 18/07/2026 <<<
-------------------------------------
- CORRECTIF MAJEUR : passage de TaskLoRaReceive() en réception NON-BLOQUANTE
  par interruption DIO1 (radio.startReceive() + radio.setDio1Action()),
  miroir de la correction déjà appliquée côté RC en V0.0.83.
  Cause du problème corrigé : bug RadioLib documenté sur SX1262 (issue
  jgromes/RadioLib #1748, matériel identique : ESP32-S3 + SX1262) — après un
  radio.transmit(), un radio.receive() bloquant peut ne pas rebasculer
  fiablement le module en réception (retourne RX_TIMEOUT quasi immédiatement).
  Comme l'avion transmettait sa télémétrie (1 réception sur 5) via
  radio.transmit() puis rappelait un radio.receive() bloquant au tour de
  boucle suivant, ce bug pouvait perturber la réception des ControlData
  juste après un envoi de télémétrie - et donc, en cascade, décaler le
  cadencement de la télémétrie elle-même. C'est très probablement la cause
  de l'irrégularité de télémétrie observée côté RC (V0.0.83), qui persistait
  malgré l'élargissement de la fenêtre d'écoute RC jusqu'à 100 ms - le
  problème n'étant pas un manque de temps d'écoute côté RC, mais une
  réception perturbée côté PL en amont.
  Nouveau fonctionnement : écoute continue armée une fois dans
  setupLoRaFastMode() (radio.startReceive()), réarmée systématiquement après
  CHAQUE paquet traité (succès ou échec CRC) ET après chaque transmission de
  télémétrie, par prudence vis-à-vis du comportement IRQ du SX126x qui peut
  nécessiter un réarmement explicite même en mode d'écoute "continu" (cf.
  issue jgromes/RadioLib #703). La boucle de détection RC au démarrage
  (avant bascule OTA/vol) est également passée en non-bloquant, par
  cohérence - elle utilisait aussi un radio.receive() bloquant.
  AUCUN changement de struct, AUCUN changement côté RC requis : cette
  version peut être flashée seule sur l'avion.
  À VALIDER SUR BANC PUIS EN VOL PROGRESSIF avant tout usage opérationnel,
  comme pour tout changement touchant la réception des commandes de vol.

*/

/*

-------------------------------------
>>> Version V0.0.77 du 18/07/2026 <<<
-------------------------------------
- CORRECTIF : "glitchCount" était déclaré et affiché (ligne [RX DEBUG]
  "GLITCHS: %u") mais jamais incrémenté nulle part (code mort, resté
  bloqué à 0 depuis toujours). Il compte désormais réellement les paquets
  LoRa reçus mais rejetés au contrôle CRC (RADIOLIB_ERR_CRC_MISMATCH) dans
  TaskLoRaReceive() : un "glitch" est un paquet dont le préambule/en-tête a
  été détecté par le module radio mais dont le contenu est corrompu — signe
  d'un lien marginal (parasite, léger fading, limite de distance...),
  différent d'un simple silence radio normal entre deux transmissions
  (RADIOLIB_ERR_RX_TIMEOUT, non compté). Complémentaire de failSafeCount
  (V0.0.75), qui lui ne compte que les coupures franches (> TIMEOUT_MS sans
  aucun paquet). Utile comme indicateur précoce de dégradation du lien,
  avant qu'elle ne devienne un vrai failsafe.

*/

/*

-------------------------------------
>>> Version V0.0.76 du 18/07/2026 <<<
-------------------------------------
- AJOUT : champ "failSafeCount" (uint32_t) en fin de TelemetryData, renvoyant
  à la RC le compteur de transitions "signal LoRa perdu" détectées côté
  avion depuis le boot (voir failSafeCount++ dans TaskControlFlight()).
  Permet de diagnostiquer à distance (ex : USB débranché côté avion pour
  éliminer l'hypothèse "monitoring série ralentit la boucle") sans avoir
  besoin du port série de l'avion : il suffit d'observer si ce compteur
  cesse d'augmenter côté RC. MIROIR OBLIGATOIRE de RC v0.0.82 : même champ,
  en VRAI dernier champ de la struct des deux côtés.
- NOTE DIAGNOSTIC (pas un correctif de code, juste une observation à
  garder en tête pour la suite) : la boucle TaskLoRaSend() de la RC appelle
  un radio.receive() BLOQUANT après CHAQUE transmission, alors que l'avion
  ne renvoie de la télémétrie qu'une fois sur 5 réceptions
  (packetCount % 5 == 0). Sur ~80% des cycles RC, ce receive() attend donc
  dans le vide jusqu'à expiration du timeout RadioLib (calculé
  automatiquement, aucun timeout explicite n'est passé), ce qui retarde
  d'autant le prochain envoi de ControlData. C'est probablement la cause
  principale des délais de 900-2200 ms observés en labo (RSSI excellent
  dans les deux sens, donc pas un problème RF). Pas encore corrigé dans
  cette version : nécessite une décision sur l'approche (timeout explicite
  court sur ce receive(), ou passage en réception non-bloquante côté RC).

*/

/*

-------------------------------------
>>> Version V0.0.75 du 18/07/2026 <<<
-------------------------------------
- CORRECTIF DIAGNOSTIC : le message "FAILSAFE CRITIQUE (Pas de GPS)" prêtait
  à confusion — il ne signifiait pas "pas de fix GPS" (lat/lon peuvent être
  valides) mais "HOME non fixé" (gps.satellites.value() jamais passé au-dessus
  du seuil) COMBINÉ à une perte de signal LoRa RC de plus de TIMEOUT_MS.
  * Seuil satellites pour fixer le HOME sorti en constante nommée
    HOME_MIN_SATELLITES, et ABAISSÉ de >6 à >3 (donc dès 4 satellites, un fix
    TinyGPS++ valide déjà exploitable). Le HOME (et donc RTH + Auto Landing)
    est ainsi disponible plus tôt après l'allumage.
  * Message de debugControlData() clarifié : distingue maintenant nommément
    "Signal LoRa perdu depuis X ms" de "HOME non fixé", et affiche le nombre
    de satellites courant vs le seuil requis, pour un diagnostic terrain
    immédiat sans avoir à recouper avec la ligne [GPS].

*/

/*

-------------------------------------
>>> Version V0.0.74 du 18/07/2026 <<<
-------------------------------------
- AJOUT : logique de vol complète pour l'Atterrissage Automatique (bouton
  "Auto Landing" de la RC), dans TaskControlFlight(). Machine à états à 3
  phases (APPROCHE / FINALE / SOL) :
    * APPROCHE : navigation vers le point HOME (relèvement GPS, comme le RTH)
      + descente contrôlée vers une altitude d'approche (20 m sol par défaut),
      ailes tenues à plat par un PID roll dédié, assiette pilotée par un PID
      altitude dédié (intégrateurs séparés de ceux de l'AutoPilot).
    * FINALE (sous 15 m sol) : cap figé (on arrête de recalculer le
      relèvement vers HOME, imprécis à courte distance), volets sortis en
      grand, gaz réduits progressivement avec l'altitude, arrondi (cabré
      progressif) sous 3 m sol.
    * SOL (sous 0.5 m sol, ou timeout de sécurité 20s en finale) : gaz
      coupés, gouvernes au neutre, volets sortis (freinage aérodynamique).
  Priorité : le RTH (switch ou perte de signal) reste prioritaire sur l'Auto
  Landing. Nécessite un HOME GPS fixé, sinon la commande reçue reste sans
  effet (comme avant, cf. V0.0.73). L'altitude utilisée est relative
  (bmpAltitude au moment de l'activation), pas un QNH calibré.
  Voir le bloc de constantes/variables "ATTERRISSAGE AUTOMATIQUE" pour tous
  les seuils (altitudes de transition, gaz, gains). À AJUSTER ET VALIDER EN
  VOL PROGRESSIF (basses altitudes d'abord) avant tout usage opérationnel.
- debugControlData() : la ligne "AUTO LANDING" affiche désormais la phase en
  cours (APPROCHE/FINALE/SOL), l'altitude sol estimée et la distance au HOME
  quand le mode est actif.
- MIROIR de RC v0.0.81 (télémétrie détaillée BARO/IMU/GPS) :
  * TelemetryData : ajout des champs "pressure" (hPa, BMP280) et "hdop"
    (GPS), en VRAIE fin de struct désormais (après groundSpeedKmh, qui
    n'était le dernier champ que jusqu'à cette version). RC v0.0.81 ajoute
    ces 3 champs (groundSpeedKmh + pressure + hdop) dans le même ordre à la
    suite de son "autoLanding". PRÉREQUIS AVANT VOL : flasher PL v0.0.74 et
    RC v0.0.81 ensemble (paquet télémétrie désaligné sinon, cf. avertissement
    historique en tête de fichier).
  * TaskLoRaReceive() : telem.pressure et telem.hdop désormais remplis.
- Ligne [RX DEBUG] de debugControlData() : ajout du numéro de version
  (FIRMWARE_VERSION) et de la puissance LoRa courante (currentTxPower), en
  miroir du format déjà utilisé côté RC (ligne [TX DEBUG] depuis v0.0.80).

*/

/*

-------------------------------------
>>> Version V0.0.73 du 15/07/2026 <<<
-------------------------------------
- MIROIR de RC v0.0.80 (télémétrie détaillée RUDDER/GIMBAL/MODES) :
  * ControlData : ajout du champ "autoLanding" (bool) en FIN de struct, en
    miroir de RC v0.0.78/79. Sans risque de désalignement (PL est récepteur
    de ControlData, direction déjà sûre pour les ajouts en fin de struct).
    La commande est désormais reçue et confirmée en télémétrie, mais AUCUNE
    logique de vol "Atterrissage Automatique" n'est encore implémentée dans
    TaskControlFlight() (comportement à définir dans une prochaine étape).
  * TelemetryData : ajout des champs "rudder", "gimbalPan", "gimbalTilt",
    "autoLanding" (échos de commande confirmant à la RC ce que l'avion a
    reçu/applique). ATTENTION CRITIQUE : ces champs sont insérés AVANT
    "groundSpeedKmh" (et non à la fin) car la RC v0.0.80 ne connaît pas ce
    dernier champ. Les ajouter après "groundSpeedKmh" aurait décalé la
    lecture côté RC (elle aurait interprété "groundSpeedKmh" comme "rudder").
    Voir commentaire détaillé au niveau de la struct.
  * debugControlData() : nouvelle ligne "AUTO LANDING" affichant l'état de
    la commande reçue.
- PRÉREQUIS AVANT VOL : nécessite RC v0.0.80 ou supérieur (structs ControlData
  et TelemetryData strictement identiques aux 4 champs près listés ci-dessus).

*/

/*

-------------------------------------
>>> Version V0.0.72 du 13/07/2026 <
-------------------------------------
- MODIFIÉ : "gear" retiré de la struct ControlData, en miroir de RC v0.0.79
  (train fixe sur cet avion). Correctif obligatoire d'alignement binaire.
  
-------------------------------------
>>> Version V0.0.71 du 11/07/2026 <<<
-------------------------------------
- Ajout d'un signal sonore sur le buzzer passif de la carte d'extension XIAO
  (broche D3 / GPIO4, câblé en dur sur la carte, cf. schéma Seeed
  "A3_D3_BUZZER") : un petit carillon (2 notes montantes) se déclenche une
  seule fois, au moment précis où le GPS passe de "recherche" à "FIX OK".
  Permet d'être averti sans avoir à regarder l'écran OLED.
- Nouvelle fonction checkGpsFixBeep(), appelée à chaque itération de
  TaskLoRaReceive (donc plus réactive que le rafraîchissement OLED cadencé
  à 1 Hz) : détecte la transition d'état via prevGpsState et ne joue le son
  qu'une seule fois par acquisition de fix (pas de bip répété en boucle).
- Le son est également rejoué si le fix est perdu puis réacquis (utile en
  vol pour confirmer une reprise GPS après un passage sous couvert).

-------------------------------------
>>> Version V0.0.70 du 11/07/2026 <<<
-------------------------------------
- Ajout d'un indicateur d'état de réception GPS sur l'écran OLED, rafraîchi
  périodiquement (toutes les GPS_OLED_REFRESH_MS) depuis TaskLoRaReceive.
  Remplace l'écran statique "Démarrage vol" une fois les tâches de vol
  lancées. 3 états distingués via getGpsState() :
    * ABSENT     : aucune trame NMEA reçue (gps.charsProcessed() < 10) ->
                   probable souci de câblage/alimentation du module L76K.
    * RECHERCHE  : le module transmet mais aucun fix valide (satellites
                   en cours d'acquisition).
    * FIX OK     : position valide, affiche satellites/lat/lon.
- Ajout d'une ligne "GPS" dans debugControlData() (Serial) : état de
  réception, nombre de satellites, HDOP, lat/lon, vitesse sol et cap.
- Module L76K câblé en UART sur le connecteur Grove de la carte
  d'extension (TXD module -> RX carte D7, RXD module -> RX carte D6,
  cf. broches GPS_RX/GPS_TX déjà définies plus bas, inchangées).

-------------------------------------
>>> Version V0.0.69 du 07/07/2026 <<<
-------------------------------------
- Ajout du champ "groundSpeedKmh" (vitesse sol GPS) dans TelemetryData, pour
  alimenter l'instrument "badin" cote telecommande. ATTENTION : vitesse SOL,
  pas vitesse AIR (pas de sonde Pitot sur cet avion) - faussee par le vent.

-------------------------------------
>>> Version V0.0.68 du 07/07/2026 <<<
-------------------------------------
- CORRECTIF : ajout du champ "altitude" (float) dans TelemetryData, desormais
  rempli avec la valeur BMP280. Ce champ existait deja cote telecommande
  (RC v0.0.76) mais n'etait jamais envoye par l'avion : la struct "packed"
  etant lue par radio.receive() sur sizeof(TelemetryData), la RC attendait
  4 octets de plus que ce qui etait reellement transmis (desalignement du
  paquet binaire, risque d'echec de reception ou de donnees aleatoires).
- Ajout de l'attitude (Roll, Pitch, YawRate) + flag imuValid dans la struct
  TelemetryData, envoyes a la telecommande via LoRa (champs ajoutes en fin de
  struct pour ne pas decaler les offsets existants).
- Objectif : alimenter un horizon artificiel sur l'ecran rond (GC9A01) de
  l'extension XIAO montee sur la telecommande.
- IMPORTANT : la struct TelemetryData cote telecommande (code de reception)
  doit etre mise a jour a l'identique (memes champs, meme ordre) pour que
  le decodage du paquet binaire reste correct.

-------------------------------------
>>> Version V0.0.67 du 04/07/2026 <<<
-------------------------------------
- Correctif détection BMI270 : ajout d'une vérification directe du registre
  CHIP_ID (0x00, doit valoir 0x24) avant d'appeler imu.begin(). Corrige le
  faux positif où imu.begin() retournait "true" alors qu'aucun BMI270 n'était
  physiquement connecté (limitation connue de la bibliothèque
  Arduino_BMI270_BMM150, conçue à l'origine pour un IMU soudé en dur).

-------------------------------------
>>> Version V0.0.66 du 04/07/2026 <<<
-------------------------------------
- Vérification au boot des capteurs BMP280/BMI270 : en cas d'échec, message
  d'erreur détaillé sur le port Série ET sur l'écran OLED (le boot n'est pas
  bloqué, le pilotage manuel reste possible sans baro/IMU).
- Affichage sur l'OLED du statut télécommande/OTA au démarrage :
    * Aucune RC détectée après 5s -> écran "MODE OTA" avec SSID + IP.
    * RC détectée -> écran de confirmation avant démarrage des tâches de vol.
- Nouvelle fonction générique oledShowLines() pour factoriser l'affichage
  de messages multi-lignes (erreurs, statut boot), réutilisée par les
  écrans capteurs et OTA.

-------------------------------------
>>> Version V0.0.62 du 30/06/2026 <<<
-------------------------------------
- Ajout de l'affichage du Rudder (data.rudder) dans debugControlData(), aux côtés des autres commandes reçues.
- Ajout d'une ligne de séparation visuelle dédiée pour mieux isoler la section télémétrie
  (signal, baro, IMU) de la section commandes reçues (sticks, gimbal, moteurs).

-------------------------------------
>>> Version V0.0.61 du 29/06/2026 <<<
-------------------------------------
- Intégration de la commande gimbal reçue depuis la télécommande (RC v0.0.71).
- Ajout des champs gimbalPan et gimbalTilt dans la structure ControlData (doit être
  identique à celle de la télécommande pour que le paquet binaire soit cohérent).
- Ajout des variables de calibration et de mapping dédiées à la gimbal :
    * GIMBAL_PAN_CHANNEL  : canal PCA9685 affecté à l'axe Pan  (rotation horizontale)
    * GIMBAL_TILT_CHANNEL : canal PCA9685 affecté à l'axe Tilt (inclinaison verticale)
    * Valeurs PWM min/centre/max configurables indépendamment pour chaque axe.
- Application des commandes gimbal dans TaskControlFlight() (tous modes : Manuel, AP, RTH, FS).
- Affichage des valeurs gimbal dans debugControlData().

-------------------------------------
>>> Version V0.0.58 du 22/06/2026 <<<
-------------------------------------
- Implémentation complète de la logique AutoPilot (AP) dans TaskControlFlight()
- Architecture fly-by-wire : les sticks pilote s'ajoutent par-dessus l'AP
- 4 boucles PID indépendantes :
    * PID Roll  → correction ailerons  (cible 0°)
    * PID Pitch → correction profondeur (cible AP_TARGET_PITCH_DEG)
    * PID Altitude → sortie pitch cible (cascade sur PID Pitch, via BMP280)
    * PID Cap   → correction différentielle moteurs (cible cap au moment activation)
- Throttle figé à l'activation AP (AP_THROTTLE_PWM)
- Reset automatique des intégrateurs PID à chaque entrée/sortie du mode AP
- Tous les gains PID regroupés en constantes facilement ajustables

-------------------------------------
>>> Version V0.0.58-OTA du 24/06/2026 <<<
-------------------------------------
- Intégration de la couche OTA passive via Wi-Fi (Approche 1 : Timeout LoRa au démarrage)
- Si aucune télécommande détectée après 5 secondes au boot -> Activation Point d'Accès "Avion-OTA"
- Si télécommande détectée -> Wi-Fi éteint, démarrage immédiat des tâches de vol (0% impact CPU)

Comment l'utiliser sur le terrain/atelier :
Pour voler : Allume ta télécommande en premier, puis branche la batterie de l'avion.
L'avion va capter la télécommande en moins de quelques millisecondes, bypasser complètement le Wi-Fi et lancer tes tâches d'AutoPilot et de contrôle.
Pour mettre à jour : Laisse ta télécommande éteinte.
Branche la batterie de ton avion (ou connecte-le simplement en USB à l'atelier).
Attends 5 secondes. Prends ton téléphone ou ton PC, connecte-toi au réseau Wi-Fi Avion-OTA.
Ouvre ton navigateur à l'adresse http://192.168.4.1/update, charge ton .bin généré par l'IDE Arduino.
L'avion se met à jour, redémarre, et c'est prêt !

-------------------------------------
>>> Version V0.0.55 du 17/06/2026 <<<
-------------------------------------
- Ajout d'un capteur 6 axes BMI270 (accéléromètre + gyroscope)
- Calcul des angles d'attitude : Roll, Pitch via filtre complémentaire
- Affichage IMU dans debugControlData

-------------------------------------
>>> Version V0.0.54 du 16/06/2026 <<<
-------------------------------------
- Ajout d'un capteur barométrique BMP280

-------------------------------------
>>> Version V0.0.48 du 12/05/2026 <<<
-------------------------------------
- Correctif pour gérer le 2èeme volet de courbure (sur le canal 6 - le premier étant sur le canal 3)

*/

#include <RadioLib.h>
#include <SPI.h>
#include <Wire.h>
#include <PCA9685.h>
#include <TinyGPS++.h>
#include <Adafruit_BMP280.h>
#include <Arduino_BMI270_BMM150.h>

// --- AJOUT POUR L'ÉCRAN OLED (Grove OLED de la carte d'extension XIAO) ---
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- AJOUTS POUR LA COUCHE OTA ---
#include <WiFi.h>
#include <WebServer.h>
#include <ElegantOTA.h>

WebServer server(80);  // Serveur web sur le port 80 pour ElegantOTA

// --- CONFIGURATION VERSION ---
const char *FIRMWARE_VERSION = "v0.0.103";

// --- CONFIGURATION ÉCRAN OLED (Grove OLED, carte d'extension XIAO) ---
// L'écran est câblé sur le bus I2C par défaut de la carte d'extension, qui correspond
// physiquement aux mêmes broches (GPIO5/GPIO6) que le bus I2C_PCA déjà utilisé pour
// la PCA9685, le BMP280 et le BMI270. On réutilise donc I2C_PCA plutôt que Wire,
// pour éviter tout conflit de broches.
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1          // Pas de broche reset dédiée
#define OLED_I2C_ADDRESS 0x3C  // Adresse I2C standard du Grove OLED 0.96" (SSD1306/SSD1315) \
                               // -> Si l'écran reste noir, essayer 0x3D.
bool oledReady = false;

// --- CONSTANTES DE PUISSANCE ---
const int PWR_LABO = 1;  // Puissance réduite pour tests USB
const int PWR_FLIGHT = 2;
const int DIFF_RANGE = 200;  // Poussée différentielle

int currentTxPower = PWR_FLIGHT;

// --- CONFIGURATION DES BROCHES ---
#define LORA_MISO 8
#define LORA_SCK 7
#define LORA_MOSI 9
#define LORA_CS 41
#define LORA_RESET 42
#define LORA_BUSY 40
#define LORA_DIO1 39
#define LORA_ANT_SW 38  // RF Switch

#define PCA_SDA 5  // Pin D4 (GPIO5)
#define PCA_SCL 6  // Pin D5 (GPIO6)
#define GPS_RX 44  // Pin D7 (GPIO44)
#define GPS_TX 43  // Pin D6 (GPIO43)
// --- AJOUT V0.0.71 : buzzer passif intégré à la carte d'extension XIAO ---
// Câblé en dur sur la carte (voir schéma Seeed "A3_D3_BUZZER") : pas de fil à
// ajouter, il suffit de piloter cette broche avec tone()/noTone().
#define BUZZER_PIN 4   // Pin D3 (GPIO4) - buzzer passif
#define BAT_ADC_PIN 3  // GPIO3 (GPIO1 et 2 NE DOIVENT JAMAIS ETRE UTILISES)
#define BATTERY_CALIBRATION 3.930f
//#define ESC_RX_PIN 3   // Pin D2 (GPIO3) -> Entrée télémétrie unique ESC

#define GPS_BAUD 9600
#define ESC_BAUDRATE 115200
#define BMI270_I2C_ADDR 0x68
#define BMI270_CHIP_ID 0x24  // Valeur attendue au registre 0x00 (voir datasheet Bosch)

// // --- STRUCT TÉLÉMÉTRIE ESC ---
// struct ESCTelemetry {
//   uint8_t temperature;
//   float voltage;
//   float current;
//   uint16_t consumption_mah;
//   uint16_t raw_rpm;
// };

// --- OBJETS ---
TinyGPSPlus gps;
HardwareSerial SerialGPS(1);
//HardwareSerial SerialESC(2);
SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RESET, LORA_BUSY);

// =========================================================================
// AJOUT V0.0.78 : RÉCEPTION NON-BLOQUANTE (interruption DIO1), miroir de la
// correction déjà appliquée côté RC (V0.0.83).
// =========================================================================
// Remplace l'ancien radio.receive() bloquant dans TaskLoRaReceive(), qui ne
// rebasculait pas toujours fiablement le SX1262 en réception juste après un
// radio.transmit() (bug RadioLib documenté sur SX1262 : après un transmit(),
// un receive() bloquant peut renvoyer RX_TIMEOUT presque immédiatement, le
// module ne réentrant pas correctement en mode RX). Cela pouvait perturber
// la réception des ControlData juste après l'envoi d'une télémétrie (1
// réception sur 5), et donc décaler le cadencement d'envoi de la télémétrie
// elle-même - symptôme observé côté RC : télémétrie irrégulière malgré une
// fenêtre d'écoute élargie.
// Principe : écoute continue armée une fois dans setup() (radio.startReceive()),
// réarmée systématiquement après chaque paquet traité ET après chaque
// transmission de télémétrie (double sécurité, cf. comportement IRQ SX126x
// qui nécessite parfois un réarmement explicite même en mode "continu").
// IMPORTANT : plRxFlag est modifié en contexte d'interruption -> volatile obligatoire.
volatile bool plRxFlag = false;

void IRAM_ATTR onLoRaDio1PL() {
  plRxFlag = true;
}
// --- MODIFIE V0.0.103 : bus I2C unifie sur l'objet Wire global ---
// La librairie Seeed-PCA9685 (I2Cdev) communique EXCLUSIVEMENT via l'objet Wire
// global d'Arduino : impossible de lui passer un TwoWire secondaire. Le PCA9685
// et les autres peripheriques I2C (OLED, BMP280, BMI270, LIDAR) partagent le
// meme bus physique (GPIO5/GPIO6). On alias donc I2C_PCA sur Wire pour ne pas
// toucher au reste du code capteurs ; Wire est initialise sur PCA_SDA/PCA_SCL
// dans setup().
#define I2C_PCA Wire

// --- OBJET ÉCRAN OLED (utilise le bus I2C_PCA, voir explication plus haut) ---
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C_PCA, OLED_RESET);

// --- BMP280 ---
Adafruit_BMP280 bmp(&I2C_PCA);
bool bmpReady = false;
float bmpPressure = 0.0;
float bmpAltitude = 0.0;
float bmpTemperature = 0.0;
const float SEA_LEVEL_HPA = 1013.25;

// --- AJOUT V0.0.100 : LIDAR TF-Luna (mesure de hauteur sol / AGL) ---
// Le capteur est câblé sur le MÊME bus I2C que les autres périphériques
// (I2C_PCA, broches PCA_SDA/PCA_SCL), à l'adresse 0x10, distincte du BMP280
// (0x76), du BMI270 (0x68), de la PCA9685 (0x7f) et de l'OLED (0x3C).
// Protocole de lecture identique au sketch test_lidar_i2c.ino.
#define LIDAR_I2C_ADDR 0x10
const uint16_t LIDAR_MIN_STRENGTH = 100;  // En-dessous : mesure peu fiable (datasheet TF-Luna)
const uint16_t LIDAR_MAX_VALID_CM = 800;  // Portée fiable max ~8 m
bool lidarReady = false;
uint16_t lidarHeightCm = 0;  // Hauteur sol mesurée (cm) ; 0 si mesure invalide
uint16_t lidarStrength = 0;  // Force du signal (sert uniquement à valider la mesure)
bool lidarValid = false;     // Dernière mesure jugée fiable (signal + portée OK)

// --- BMI270 ---
BoschSensorClass imu(I2C_PCA);
bool imuReady = false;

float imuAccX = 0.0, imuAccY = 0.0, imuAccZ = 0.0;
float imuGyrX = 0.0, imuGyrY = 0.0, imuGyrZ = 0.0;
float imuRoll = 0.0;
float imuPitch = 0.0;
float imuYawRate = 0.0;

const float COMP_FILTER_ALPHA = 0.98f;
unsigned long lastImuTime = 0;

// --- MODIFIE V0.0.103 : pilote PCA9685 via la librairie Seeed-PCA9685 ---
// Adresse I2C 0x7f : valeur par defaut de la carte Seeed et confirmee par le
// sketch de reference fourni (seeed_servo_pca9685_ok.ino, ServoDriver.init(0x7f)).
#define PCA9685_ADDR 0x7f  // Adresse I2C du module PCA9685 (Seeed)
ServoDriver pwm;

// Conversion microsecondes -> comptage 12 bits (0-4095) de la PCA9685 a 50 Hz.
// A 50 Hz la periode vaut 20000 us et correspond a 4096 comptages.
static inline uint16_t pwmMicrosecondsToTicks(int microseconds) {
  if (microseconds < 0) microseconds = 0;
  long ticks = ((long)microseconds * 4096L) / 20000L;
  if (ticks > 4095) ticks = 4095;
  return (uint16_t)ticks;
}

// Equivalent de l'ancien pwm.writeMicroseconds(canal, us) d'Adafruit.
// NB : Seeed-PCA9685 numerote les canaux de 1 a 16 -> on ajoute 1 au canal
// (qui reste exprime de 0 a 15 dans le reste du sketch).
static inline void pwmWriteMicroseconds(uint8_t channel, int microseconds) {
  pwm.setPwm(channel + 1, 0, pwmMicrosecondsToTicks(microseconds));
}
const uint32_t MY_AIRCRAFT_ID = 0x55AA3322;

struct __attribute__((packed)) ControlData {
  uint32_t aircraftId;
  uint32_t msgId;
  int16_t ailerons, elevator, rudder, flaps, throttle;


  // --- MODIFIÉ V0.0.72 : "gear" retiré de ControlData, en miroir de RC v0.0.79
  // (train fixe, plus de logique gear). Ce champ étant AU MILIEU de la struct,
  // sa suppression devait être répercutée ici pour garder l'alignement binaire
  // du paquet LoRa (sinon lights/returnToHome/trims/gimbal auraient été décodés
  // avec un décalage d'un octet). ---
  //  bool autoPilot, gear, lights;
  bool autoPilot, lights;

  bool returnToHome;
  int16_t trimRudder, trimElevator;
  // --- GIMBAL (ajouté en RC v0.0.71 / PL v0.0.61) ---
  // Ces deux champs doivent être présents et dans le même ordre que dans la
  // télécommande, la struct étant sérialisée bit à bit dans le paquet LoRa.
  int16_t gimbalPan;   // Axe horizontal (0-4095, centre = 2048)
  int16_t gimbalTilt;  // Axe vertical   (0-4095, centre = 2048)
  // --- AJOUT V0.0.73 : ordre "Atterrissage Automatique", en miroir de RC v0.0.78/79.
  // Champ ajouté en FIN de struct (comme côté RC) : direction RC -> PL, donc sans
  // risque de désalignement (PL réclame simplement sizeof(ControlData) octets,
  // qui inclut désormais ce champ, à la même position que côté RC).
  // NOTE : la RÉCEPTION de ce champ est câblée ici et l'état est renvoyé en
  // télémétrie (voir TelemetryData.autoLanding), mais AUCUNE logique de vol
  // "Atterrissage Automatique" n'est implémentée dans TaskControlFlight() à ce
  // stade (comportement à définir : approche, réduction des gaz, taux de
  // descente, etc.). Pour l'instant l'appui sur le bouton est reçu et confirmé
  // à la RC, sans effet sur le pilotage.
  bool autoLanding;
} rxData;

// =========================================================================
// CONFIGURATION GIMBAL
// Adapter les numéros de canaux PCA9685 et les valeurs PWM selon le servo
// ou contrôleur de gimbal utilisé.
// =========================================================================

// --- Canaux PCA9685 affectés à la gimbal ---
// Choisir deux canaux libres (ici 8 et 9, canaux 0-7 étant déjà utilisés).
#define GIMBAL_PAN_CHANNEL 8   // Canal PCA9685 → servo Pan  (rotation horizontale)
#define GIMBAL_TILT_CHANNEL 9  // Canal PCA9685 → servo Tilt (inclinaison verticale)

// --- Limites PWM du servo Pan (µs) ---
// À ajuster selon les butées mécaniques de votre gimbal pour ne pas forcer.
const int GIMBAL_PAN_MIN = 1000;     // Butée gauche
const int GIMBAL_PAN_CENTER = 1500;  // Position neutre (repos / face avant)
const int GIMBAL_PAN_MAX = 2000;     // Butée droite

// --- Limites PWM du servo Tilt (µs) ---
const int GIMBAL_TILT_MIN = 1000;     // Butée basse
const int GIMBAL_TILT_CENTER = 1500;  // Position neutre (horizontal)
const int GIMBAL_TILT_MAX = 2000;     // Butée haute

// --- Position de sécurité en cas de perte de signal ou failsafe ---
// La gimbal revient face avant / horizontale pour éviter de filmer vers le bas ou l'arrière.
const int GIMBAL_PAN_FAILSAFE = 1500;   // Centre (neutre)
const int GIMBAL_TILT_FAILSAFE = 1500;  // Horizontal (neutre)

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
  // --- CORRECTIF V0.0.68 : ce champ existait deja cote telecommande (RC v0.0.76)
  // mais n'etait jamais rempli/envoye cote avion. Comme la struct est "packed"
  // et que radio.receive() lit sizeof(TelemetryData) octets, la RC attendait
  // 4 octets de plus que ce qui etait reellement transmis -> desalignement du
  // paquet binaire. Corrige en l'ajoutant ici, alimente par le BMP280.
  float altitude;
  // --- AJOUT V0.0.68 : attitude IMU pour horizon artificiel telecommande ---
  // Champs ajoutes en FIN de struct pour ne rien decaler cote offsets existants.
  // ATTENTION : cette structure doit etre STRICTEMENT identique (memes champs,
  // meme ordre) cote telecommande, sinon le paquet binaire sera mal interprete.
  float attRoll;     // degres, + = aile droite basse
  float attPitch;    // degres, + = nez haut
  float attYawRate;  // degres/s
  bool imuValid;     // false si BMI270 absent/en echec -> ne pas afficher l'horizon
  // --- AJOUT V0.0.73 : échos de commande renvoyés à la RC (miroir RC v0.0.80) ---
  // ATTENTION CRITIQUE À L'ORDRE : ces 4 champs sont insérés ICI, AVANT
  // groundSpeedKmh, et non à la toute fin de la struct. Raison : côté RC
  // (v0.0.80), TelemetryData s'arrête à "imuValid" + ces 4 nouveaux champs -
  // elle n'a PAS de champ groundSpeedKmh. radio.receive() côté RC ne lit donc
  // que sizeof(TelemetryData RC) octets depuis le début du paquet ; si ces 4
  // champs étaient ajoutés APRÈS groundSpeedKmh, ils tomberaient hors de la
  // zone lue par la RC (ignorés), et surtout groundSpeedKmh se retrouverait
  // décodé par la RC à la place de "rudder" -> valeurs aberrantes. En les
  // plaçant ici, juste après imuValid (qui est le dernier champ commun aux
  // deux structs), l'alignement reste correct pour la RC, et groundSpeedKmh
  // (que la RC ignore de toute façon, elle ne l'a jamais eu) reste simplement
  // le tout dernier champ, tronqué sans risque côté réception RC.
  int16_t rudder;      // Position rudder reçue de la RC, telle qu'appliquée par l'avion
  int16_t gimbalPan;   // Position gimbal Pan reçue de la RC, telle qu'appliquée par l'avion
  int16_t gimbalTilt;  // Position gimbal Tilt reçue de la RC, telle qu'appliquée par l'avion
  bool autoLanding;    // Confirmation de réception de l'ordre "Atterrissage Automatique"
  // --- AJOUT V0.0.69 : vitesse sol GPS, utilisee en substitut d'anemometre
  // (pas de sonde Pitot sur cet avion) pour l'instrument "badin" telecommande.
  // ATTENTION : c'est une vitesse SOL, pas une vitesse AIR (faussee par le vent).
  float groundSpeedKmh;
  // --- AJOUT V0.0.74 : pression barométrique brute + HDOP GPS, en MIROIR de
  // RC v0.0.81. ATTENTION CRITIQUE : ces 2 champs sont désormais les VRAIS
  // derniers champs de la struct (groundSpeedKmh n'est plus le dernier).
  // RC v0.0.81 ajoute EXACTEMENT ces 3 champs (groundSpeedKmh, pressure, hdop),
  // dans le même ordre, à la suite de son "autoLanding". PRÉREQUIS AVANT VOL :
  // flasher PL et RC ensemble (RC v0.0.81 + PL v0.0.74 minimum), sinon paquet
  // télémétrie mal aligné (voir avertissement historique en tête de fichier).
  float pressure;  // hPa, valeur brute BMP280 (bmpPressure)
  float hdop;      // Dilution de précision horizontale GPS (99.9 si non valide)
  // --- AJOUT V0.0.76 : compteur de failsafes (transitions perte de signal LoRa),
  // pour diagnostic à distance depuis la RC sans avoir besoin du port série de
  // l'avion. MIROIR RC v0.0.82 : même champ, en VRAI dernier champ de la struct.
  uint32_t failSafeCount;
  // --- AJOUT V0.0.100 : hauteur sol (AGL) mesurée par le LIDAR TF-Luna, en cm.
  // MIROIR RC v0.0.100 : même champ (uint16_t), en VRAI dernier champ de la
  // struct. Renvoyé à la RC pour affichage. 0 si mesure LIDAR invalide.
  uint16_t lidarHeightCm;
} telem;

// --- AJOUT V0.0.70 : état de réception GPS (déclaré ici, avant les variables
// globales qui l'utilisent ; voir getGpsState() plus bas pour le calcul) ---
enum GpsState { GPS_NO_DATA,
                GPS_SEARCHING,
                GPS_FIX };  // Absent / en recherche / fix valide

// --- VARIABLES GLOBALES ---
volatile float lastRSSI = 0;
volatile float lastSNR = 0;
volatile uint32_t packetCount = 0;
volatile uint32_t lastPacketTime = 0;
const uint32_t TIMEOUT_MS = 500;
// AJOUT V0.0.75 : seuil satellites pour fixer le HOME GPS, sorti en constante
// nommée (au lieu du "6" en dur). Abaissé de >6 à >3 (donc dès 4 satellites) :
// un fix TinyGPS++ isValid() est déjà exploitable dès 4 sats, et le seuil
// précédent retardait inutilement le HOME (donc la disponibilité du RTH et de
// l'Auto Landing) dans les cas où l'avion reste un moment avec 4-6 satellites
// après l'allumage (sous couvert, proximité de bâtiments). Cf. discussion sur
// le message FAILSAFE CRITIQUE déclenché par un HOME non fixé.
const int HOME_MIN_SATELLITES = 3;  // HOME fixé dès satellites > 3 (donc >= 4)
unsigned long lastDebugTime = 0;
SemaphoreHandle_t mutex;
int prevFlaps = 1000;

// --- AJOUT V0.0.70 : rafraîchissement périodique de l'indicateur GPS sur l'OLED ---
unsigned long lastGpsOledUpdate = 0;
const uint32_t GPS_OLED_REFRESH_MS = 1000;  // 1 rafraîchissement OLED par seconde

// --- AJOUT V0.0.71 : mémorise l'état GPS précédent pour ne jouer le bip de
// fix qu'une seule fois, au moment exact de la transition ---
GpsState prevGpsState = GPS_NO_DATA;

//ESCTelemetry liveEscData = { 0, 0.0, 0.0, 0, 0 };
float homeLat = 0, homeLon = 0;
bool homeSet = false;
float targetHeading = 0;
float currentHeading = 0;

int prevL = 1000, prevR = 1000;
int prevElev = 1500, prevRud = 1500;
int prevAil1 = 1500, prevAil2 = 1500;
uint32_t glitchCount = 0;
uint32_t failSafeCount = 0;
bool fsDetected = false;
const int MAX_SERVO_JUMP = 800;

// =========================================================================
// AUTOPILOT (AP) — CONSTANTES ET ÉTAT
// =========================================================================

// --- Cibles AP ---
const float AP_TARGET_ROLL_DEG = 0.0f;
const float AP_TARGET_PITCH_DEG = 3.0f;
const float AP_ALT_HOLD_DEADBAND = 1.5f;
const int AP_THROTTLE_PWM = 1550;

// --- Déflexion maxi autorisée par l'AP sur chaque axe (µs autour du neutre) ---
const int AP_MAX_AIL_CORR = 300;
const int AP_MAX_ELEV_CORR = 250;
const int AP_MAX_DIFF_CORR = 200;

// --- Autorité stick pilote en mode AP (µs) — permet le fly-by-wire ---
// Le stick est mappé sur ±AP_STICK_AUTHORITY µs et s'ajoute à la correction AP.
const int AP_STICK_AUTHORITY_AIL = 300;
const int AP_STICK_AUTHORITY_ELEV = 300;

// ---------------------------------------------------------------------------
// Gains PID Roll  (IMU → ailerons)
// Kp : effort proportionnel à l'erreur d'angle
// Ki : compensation de la dérive lente (vent, déséquilibre)
// Kd : amortissement des oscillations
// ---------------------------------------------------------------------------
const float AP_ROLL_KP = 4.5f;
const float AP_ROLL_KI = 0.05f;
const float AP_ROLL_KD = 1.2f;

// ---------------------------------------------------------------------------
// Gains PID Pitch (IMU → profondeur)
// ---------------------------------------------------------------------------
const float AP_PITCH_KP = 5.0f;
const float AP_PITCH_KI = 0.08f;
const float AP_PITCH_KD = 1.5f;

// ---------------------------------------------------------------------------
// Gains PID Altitude (BMP280 → pitch cible, boucle externe de la cascade)
// La sortie de ce PID est une correction d'angle en degrés ajoutée à AP_TARGET_PITCH_DEG.
// ---------------------------------------------------------------------------
const float AP_ALT_KP = 0.6f;
const float AP_ALT_KI = 0.02f;
const float AP_ALT_KD = 0.8f;
const float AP_ALT_MAX_PITCH_CORR = 8.0f;

// ---------------------------------------------------------------------------
// Gains PID Cap (GPS/IMU → différentielle moteurs)
// L'erreur est la différence angulaire entre le cap courant et le cap figé (°).
// ---------------------------------------------------------------------------
const float AP_HDG_KP = 3.0f;
const float AP_HDG_KI = 0.02f;
const float AP_HDG_KD = 0.5f;

// --- État interne AP ---
bool apWasActive = false;
float apLockedAltitude = 0.0f;
float apLockedHeading = 0.0f;
int apLockedThrottle = AP_THROTTLE_PWM;

// Accumulateurs et précédents pour les 4 PID
float apRollInteg = 0, apRollPrev = 0;
float apPitchInteg = 0, apPitchPrev = 0;
float apAltInteg = 0, apAltPrev = 0;
float apHdgInteg = 0, apHdgPrev = 0;

// Anti-windup : limite absolue de l'intégrateur (en µs ou degrés selon la boucle)
const float AP_INTEG_LIMIT_AIL = 100.0f;
const float AP_INTEG_LIMIT_ELEV = 100.0f;
const float AP_INTEG_LIMIT_ALT = 3.0f;   // ° de correction pitch maxi par intégrateur
const float AP_INTEG_LIMIT_HDG = 80.0f;  // µs

// Utilitaire : différence angulaire normalisée dans [-180, +180]
inline float angleDiff(float target, float current) {
  float d = target - current;
  while (d > 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

// =========================================================================
// AJOUT V0.0.74 : ATTERRISSAGE AUTOMATIQUE (Auto Landing)
// =========================================================================
// Déclenché par rxData.autoLanding (bouton RC, cf. RC v0.0.78+). Nécessite le
// HOME GPS fixé (homeSet), sinon la commande est reçue mais sans effet (reste
// en Manuel/AP), comme documenté au niveau de debugControlData().
//
// Principe (3 phases) :
//  1) APPROCHE : l'avion navigue vers le point HOME (cap GPS, comme le RTH)
//     tout en descendant vers une altitude d'approche (AUTOLAND_APPROACH_ALT_AGL
//     mètres au-dessus du sol), ailes tenues à plat par un PID roll dédié,
//     profondeur pilotée par un PID altitude dédié (indépendant de celui de
//     l'AP, pour ne pas mélanger les intégrateurs des deux modes).
//  2) FINALE : dès que l'altitude sol (AGL) passe sous AUTOLAND_FINAL_ALT_AGL,
//     on fige le cap courant (on ne cherche plus à recalculer le relèvement
//     vers HOME, qui devient erratique à faible distance), on sort les volets
//     en grand braquage, on réduit les gaz progressivement avec l'altitude,
//     et on ajoute un arrondi (cabré progressif) sous AUTOLAND_FLARE_ALT_AGL.
//  3) SOL : gaz coupés, gouvernes au neutre, volets sortis (freinage aéro).
//     On y reste tant que le pilote n'a pas désactivé le switch "Auto Landing".
//
// Sécurité : le RTH (switch ou perte de signal) reste PRIORITAIRE sur l'Auto
// Landing (cf. ordre des tests dans TaskControlFlight). Pour reprendre la
// main à tout moment, il suffit de désactiver le switch "Auto Landing" (ou
// d'activer RTH) : la machine à états est réinitialisée proprement.
// L'altitude utilisée est TOUJOURS relative (bmpAltitude - homeAltitude), le
// BMP280 n'étant pas calibré sur une pression QNH locale : homeAltitude
// mémorise l'altitude barométrique au moment précis où l'Auto Landing démarre
// réellement (et non au moment du fix HOME, pour rester cohérent même si le
// terrain a une pente ou si le HOME a été fixé loin du point de décollage).
// =========================================================================

enum LandingPhase { LANDING_APPROACH,
                    LANDING_FINAL,
                    LANDING_GROUND };
LandingPhase landingPhase = LANDING_APPROACH;
bool landWasActive = false;
float landHomeAltitude = 0.0f;            // Référence "sol" (bmpAltitude au démarrage de la séquence)
float landFinalHeading = 0.0f;            // Cap GPS figé au passage en phase FINALE
unsigned long landingFinalStartTime = 0;  // Horodatage d'entrée en phase FINALE (garde-fou timeout)

// --- Seuils (à ajuster sur le terrain selon l'avion et le calage QNH) ---
const float AUTOLAND_APPROACH_ALT_AGL = 20.0f;          // m : altitude cible en phase d'approche
const float AUTOLAND_FINAL_ALT_AGL = 15.0f;             // m : bascule APPROCHE -> FINALE
const float AUTOLAND_FLARE_ALT_AGL = 3.0f;              // m : début de l'arrondi (cabré progressif)
const float AUTOLAND_GROUND_ALT_AGL = 0.5f;             // m : bascule FINALE -> SOL (posé)
const unsigned long AUTOLAND_FINAL_TIMEOUT_MS = 20000;  // Garde-fou : coupe les gaz après 20s de finale max

const int AUTOLAND_APPROACH_THROTTLE_PWM = 1350;  // Gaz réduits, descente contrôlée en approche
const int AUTOLAND_IDLE_THROTTLE_PWM = 1100;      // Gaz coupés (finale basse / sol)
const int AUTOLAND_FLARE_MAX_CORR = 150;          // µs de cabré max ajoutés pendant l'arrondi

// --- PID Altitude dédié à la descente (séparé de celui de l'AP) ---
const float LAND_ALT_KP = 0.5f;
const float LAND_ALT_KI = 0.01f;
const float LAND_ALT_KD = 0.6f;
const float LAND_ALT_MAX_PITCH_CORR = 10.0f;
const float LAND_INTEG_LIMIT_ALT = 3.0f;
const float LAND_TARGET_PITCH_DEG = 2.0f;        // Pitch de base en approche stabilisée (légèrement cabré)
const float LAND_PITCH_GAIN_US_PER_DEG = 40.0f;  // Conversion simple assiette -> µs (P pur, pas de PID complet)
float landAltInteg = 0, landAltPrev = 0;

// --- PID Roll dédié (ailes à plat), mêmes gains que l'AP mais intégrateur séparé ---
float landRollInteg = 0, landRollPrev = 0;

// --- Cap : gain de correction simple (pas un PID complet, suffisant pour tenir un axe) ---
const float LAND_HDG_GAIN = 8.0f;   // µs de rudder par degré d'erreur de cap
const int LAND_HDG_MAX_CORR = 300;  // µs

// --- PROTOTYPES ---
//void updateESCTelemetry();
uint8_t update_crc8(uint8_t crc, uint8_t crc_seed);
void debugControlData(const ControlData &data);
void updateGPS();
void setupLoRaFastMode();
void updateBarometer();
void updateLidar();     // AJOUT V0.0.100 : lecture de la hauteur sol via LIDAR TF-Luna (I2C 0x10)
float getLandingAGL();  // AJOUT V0.0.100 : hauteur sol (m) pour l'atterrissage (LIDAR prioritaire, repli baro)
void updateIMU();
void applyGimbal(int panPWM, int tiltPWM);                                                          // Applique les commandes gimbal sur la PCA9685
void showVersionOnOLED();                                                                           // Affiche le numéro de version du firmware sur l'écran OLED
void oledShowLines(const char *l1, const char *l2 = "", const char *l3 = "", const char *l4 = "");  // Affiche jusqu'à 4 lignes sur l'OLED
bool checkBMI270Present();                                                                          // Vérifie la présence réelle du BMI270 via son registre CHIP_ID
// --- AJOUT V0.0.70 : indicateur d'état de réception GPS (enum déclaré plus
// haut, avant les variables globales qui en dépendent) ---
GpsState getGpsState();      // Calcule l'état courant à partir de l'objet TinyGPS++
void updateGpsStatusOLED();  // Affiche l'état GPS sur l'écran OLED (appelé périodiquement)
// --- AJOUT V0.0.71 : signal sonore de confirmation de fix GPS ---
void playGpsFixChime();  // Joue un carillon 2 notes sur le buzzer passif (D3)
void checkGpsFixBeep();  // Détecte la transition vers GPS_FIX et déclenche le bip
void TaskControlFlight(void *pvParameters);
void TaskLoRaReceive(void *pvParameters);

// =========================================================================
// AFFICHAGE DU NUMÉRO DE VERSION SUR L'ÉCRAN OLED
// =========================================================================
void showVersionOnOLED() {
  if (!oledReady) return;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("PLANE FIRMWARE"));
  display.println(F("--------------"));

  display.setTextSize(2);
  display.setCursor(0, 24);
  display.println(FIRMWARE_VERSION);

  display.setTextSize(1);
  display.setCursor(0, 54);
  display.println(F("Demarrage..."));

  display.display();
}

// =========================================================================
// AFFICHAGE GÉNÉRIQUE DE 4 LIGNES SUR L'OLED
// Utilisé pour les messages d'erreur capteurs et le statut OTA.
// Appel ponctuel uniquement (boot / OTA) : jamais dans les tâches temps réel.
// =========================================================================
void oledShowLines(const char *l1, const char *l2, const char *l3, const char *l4) {
  if (!oledReady) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(l1);
  display.setCursor(0, 16);
  display.println(l2);
  display.setCursor(0, 32);
  display.println(l3);
  display.setCursor(0, 48);
  display.println(l4);
  display.display();
}

// =========================================================================
// VÉRIFICATION DIRECTE DE LA PRÉSENCE DU BMI270 (registre CHIP_ID)
// La bibliothèque Arduino_BMI270_BMM150 (conçue à l'origine pour l'IMU soudé
// en dur de la Nano 33 BLE Sense Rev2) ne vérifie pas toujours correctement
// l'ACK I2C : son begin() peut renvoyer "true" même sans capteur connecté.
// On effectue donc ici une lecture bas niveau du registre CHIP_ID (0x00),
// qui doit impérativement valoir 0x24 pour un vrai BMI270.
// =========================================================================
bool checkBMI270Present() {
  I2C_PCA.beginTransmission(BMI270_I2C_ADDR);
  I2C_PCA.write(0x00);                                    // Registre CHIP_ID
  if (I2C_PCA.endTransmission(false) != 0) return false;  // Pas d'ACK -> absent

  I2C_PCA.requestFrom((int)BMI270_I2C_ADDR, 1);
  if (I2C_PCA.available() < 1) return false;

  uint8_t chipId = I2C_PCA.read();
  return (chipId == BMI270_CHIP_ID);
}

// =========================================================================
// AJOUT V0.0.70 : ÉTAT DE RÉCEPTION GPS (module L76K, UART Grove)
// Distingue 3 cas à partir de l'objet TinyGPS++ :
//  - GPS_NO_DATA   : aucune trame NMEA reçue depuis le boot
//                     (gps.charsProcessed() reste < 10) -> quasi toujours un
//                     souci de câblage (TXD/RXD croisés ?) ou d'alimentation.
//  - GPS_SEARCHING : des trames NMEA arrivent bien, mais aucun fix valide
//                     pour l'instant (acquisition satellites en cours).
//  - GPS_FIX       : position valide obtenue (gps.location.isValid()).
// =========================================================================
GpsState getGpsState() {
  if (gps.charsProcessed() < 10) return GPS_NO_DATA;
  if (!gps.location.isValid()) return GPS_SEARCHING;
  return GPS_FIX;
}

// =========================================================================
// AJOUT V0.0.70 : AFFICHAGE DE L'ÉTAT GPS SUR L'ÉCRAN OLED
// Appelée périodiquement (voir GPS_OLED_REFRESH_MS) depuis TaskLoRaReceive.
// Remplace à l'écran le message statique de boot "Demarrage vol..." une fois
// que les tâches de vol tournent, sans bloquer ni ralentir la boucle.
// =========================================================================
void updateGpsStatusOLED() {
  if (!oledReady) return;

  GpsState state = getGpsState();
  char l2[22], l3[22], l4[22];

  switch (state) {
    case GPS_NO_DATA:
      // Aucune donnée reçue : très probablement un problème de câblage
      // (TXD/RXD non croisés, VCC/GND inversés, ou module non alimenté).
      snprintf(l2, sizeof(l2), "Aucune donnee recue");
      snprintf(l3, sizeof(l3), "Verifier cablage");
      snprintf(l4, sizeof(l4), "TXD/RXD/3V3/GND");
      oledShowLines("GPS : ABSENT", l2, l3, l4);
      break;

    case GPS_SEARCHING:
      // Le module dialogue mais n'a pas encore de position valide.
      snprintf(l2, sizeof(l2), "Recherche fix...");
      snprintf(l3, sizeof(l3), "Satellites: %d",
               gps.satellites.isValid() ? (int)gps.satellites.value() : 0);
      snprintf(l4, sizeof(l4), "Ciel degage requis");
      oledShowLines("GPS : RECHERCHE", l2, l3, l4);
      break;

    case GPS_FIX:
      // Position valide : on affiche le nombre de satellites et la position.
      snprintf(l2, sizeof(l2), "Satellites: %d", (int)gps.satellites.value());
      snprintf(l3, sizeof(l3), "Lat: %.5f", gps.location.lat());
      snprintf(l4, sizeof(l4), "Lon: %.5f", gps.location.lng());
      oledShowLines("GPS : FIX OK", l2, l3, l4);
      break;
  }
}

// =========================================================================
// AJOUT V0.0.71 : SIGNAL SONORE DE CONFIRMATION DE FIX GPS
// Joue un petit carillon 2 notes montantes sur le buzzer passif de la carte
// d'extension (D3/GPIO4, câblé en dur, pas de fil à ajouter).
// Bloquant ~250ms : acceptable ici car appelé une seule fois par transition,
// jamais en boucle - impact négligeable sur la réception/transmission LoRa.
// =========================================================================
void playGpsFixChime() {
  tone(BUZZER_PIN, 1046, 80);  // Note 1 (Do) - courte
  delay(90);
  tone(BUZZER_PIN, 1568, 150);  // Note 2 (Sol, plus aiguë) - signal "positif"
  delay(160);
  noTone(BUZZER_PIN);
}

// =========================================================================
// AJOUT V0.0.71 : DÉTECTION DE LA TRANSITION VERS UN FIX GPS
// Appelée à chaque itération de TaskLoRaReceive (donc plus réactive que le
// rafraîchissement OLED cadencé à 1 Hz). Compare l'état courant à l'état
// précédent (prevGpsState) et ne déclenche le bip qu'au moment exact où l'on
// passe à GPS_FIX - pas de bip répété tant que le fix reste acquis. Si le fix
// est perdu puis réacquis, le bip est rejoué (utile pour confirmer une
// reprise GPS après un passage sous couvert en vol).
// =========================================================================
void checkGpsFixBeep() {
  GpsState state = getGpsState();
  if (state == GPS_FIX && prevGpsState != GPS_FIX) {
    playGpsFixChime();
  }
  prevGpsState = state;
}

// --- INITIALISATION RADIO ---
void setupLoRaFastMode() {
  Serial.print(F("[LoRa] Initialisation Radio... "));
  int state = radio.begin(868.0);
  if (state == RADIOLIB_ERR_NONE) {
    state = radio.setTCXO(1.6);
    if (state != RADIOLIB_ERR_NONE) {
      Serial.printf("Erreur configuration TCXO (%d)\n", state);
    }
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
    // AJOUT V0.0.78 : callback DIO1 + armement initial de l'écoute non-bloquante
    // (voir TaskLoRaReceive()). Doit être fait ICI, radio.begin() venant de réussir.
    radio.setDio1Action(onLoRaDio1PL);
    radio.startReceive();
  } else {
    Serial.printf("ÉCHEC (Code erreur : %d)\n", state);
    while (true)
      ;
  }
}

// --- CALCUL DU CRC8 POUR TELEMETRIE BLHELI ---
uint8_t update_crc8(uint8_t crc, uint8_t crc_seed) {
  uint8_t i;
  crc ^= crc_seed;
  for (i = 0; i < 8; i++) {
    if (crc & 0x80) {
      crc = (crc << 1) ^ 0x07;
    } else {
      crc <<= 1;
    }
  }
  return crc;
}

// // --- LECTURE ET DECODAGE DU FLUX UART ESC ---
// void updateESCTelemetry() {
//   static uint8_t buffer[10];
//   static uint8_t bufIndex = 0;
//   while (SerialESC.available() > 0) {
//     uint8_t c = SerialESC.read();
//     buffer[bufIndex++] = c;
//     if (bufIndex >= 10) {
//       uint8_t crc = 0;
//       for (int i = 0; i < 9; i++) {
//         crc = update_crc8(crc, buffer[i]);
//       }
//       if (crc == buffer[9]) {
//         if (xSemaphoreTake(mutex, 0)) {
//           liveEscData.temperature = buffer[0];
//           liveEscData.voltage = ((buffer[1] << 8) | buffer[2]) * 0.01;
//           liveEscData.current = ((buffer[3] << 8) | buffer[4]) * 0.01;
//           liveEscData.consumption_mah = (buffer[5] << 8) | buffer[6];
//           liveEscData.raw_rpm = (buffer[7] << 8) | buffer[8];
//           xSemaphoreGive(mutex);
//         }
//       }
//       bufIndex = 0;
//     }
//   }
// }

// =========================================================================
// LECTURE ET FUSION IMU BMI270
// Filtre complémentaire : angle = α × (angle + gyro×dt) + (1-α) × angle_accel
// =========================================================================
void updateIMU() {
  if (!imuReady) return;
  if (!imu.accelerationAvailable() && !imu.gyroscopeAvailable()) return;

  unsigned long now = micros();
  float dt = (lastImuTime == 0) ? 0.02f : (float)(now - lastImuTime) / 1000000.0f;
  lastImuTime = now;

  // Clamp dt pour éviter une explosion au 1er cycle ou après un freeze
  if (dt > 0.1f) dt = 0.02f;

  float ax = 0, ay = 0, az = 0;
  float gx = 0, gy = 0, gz = 0;

  // Lecture accéléromètre (en g)
  if (imu.accelerationAvailable()) imu.readAcceleration(ax, ay, az);

  // Lecture gyroscope (en °/s)
  if (imu.gyroscopeAvailable()) imu.readGyroscope(gx, gy, gz);

  // Angles d'attitude estimés par l'accéléromètre seul (référence statique)
  float rollAcc = atan2f(ay, az) * RAD_TO_DEG;
  float pitchAcc = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

  float rollFused = COMP_FILTER_ALPHA * (imuRoll + gx * dt) + (1.0f - COMP_FILTER_ALPHA) * rollAcc;
  float pitchFused = COMP_FILTER_ALPHA * (imuPitch + gy * dt) + (1.0f - COMP_FILTER_ALPHA) * pitchAcc;

  // Mise à jour des variables globales protégées
  if (xSemaphoreTake(mutex, 0)) {
    imuAccX = ax;
    imuAccY = ay;
    imuAccZ = az;
    imuGyrX = gx;
    imuGyrY = gy;
    imuGyrZ = gz;
    imuRoll = rollFused;
    imuPitch = pitchFused;
    imuYawRate = gz;
    xSemaphoreGive(mutex);
  }
}


// =========================================================================
// AUTOPILOT — CALCUL DES CORRECTIONS PID
// Appelé uniquement depuis TaskControlFlight(), déjà sous mutex.
// dt en secondes. Retourne la correction en µs (ailerons/profondeur/diff)
// ou en degrés (altitude→pitch).
// =========================================================================

// PID générique : erreur → sortie, avec anti-windup sur l'intégrateur
static float pidCompute(float error, float dt, float kp, float ki, float kd, float &integ, float &prevError, float integLimit) {
  integ += error * dt;
  integ = constrain(integ, -integLimit, integLimit);
  float deriv = (dt > 0.0f) ? (error - prevError) / dt : 0.0f;
  prevError = error;
  return kp * error + ki * integ + kd * deriv;
}

// =========================================================================
// COMMANDE GIMBAL
// Envoie les valeurs PWM calculées sur les deux canaux PCA9685 de la gimbal.
// Appelée depuis TaskControlFlight() dans tous les modes de vol.
// panPWM et tiltPWM sont déjà contraints avant l'appel.
// =========================================================================
void applyGimbal(int panPWM, int tiltPWM) {
  pwmWriteMicroseconds(GIMBAL_PAN_CHANNEL, panPWM);
  pwmWriteMicroseconds(GIMBAL_TILT_CHANNEL, tiltPWM);
}

void TaskControlFlight(void *pvParameters) {
  static float currentFlaps = 1000.0;
  const float flapSpeed = 10.0;
  static unsigned long lastLoopTime = 0;

  for (;;) {
    uint32_t now = millis();
    float dt = (lastLoopTime == 0) ? 0.02f : (float)(now - lastLoopTime) / 1000.0f;
    if (dt > 0.1f) dt = 0.02f;
    lastLoopTime = now;

    int targetPower = (Serial) ? PWR_LABO : PWR_FLIGHT;
    if (currentTxPower != targetPower) {
      currentTxPower = targetPower;
      radio.setOutputPower(currentTxPower);
    }

    if (xSemaphoreTake(mutex, portMAX_DELAY)) {
      bool lossOfSignal = (now - lastPacketTime > TIMEOUT_MS);
      if (lossOfSignal && !fsDetected) {
        failSafeCount++;
        fsDetected = true;
      } else if (!lossOfSignal) {
        fsDetected = false;
      }

      bool rthRequested = rxData.returnToHome || lossOfSignal;
      bool apActive = rxData.autoPilot && !rthRequested;
      // AJOUT V0.0.74 : Auto Landing actif seulement si demandé, non prioritaire
      // devant le RTH/failsafe (rthRequested), et seulement si un HOME GPS existe
      // (sans HOME, impossible de naviguer vers un point de poser).
      bool landActive = rxData.autoLanding && !rthRequested && homeSet;
      if (!landActive && landWasActive) {
        // On vient de sortir du mode (switch coupé, RTH demandé, ou HOME perdu) :
        // on réinitialise proprement la machine à états pour repartir de zéro
        // à la prochaine activation.
        landingPhase = LANDING_APPROACH;
        landAltInteg = landAltPrev = 0;
        landRollInteg = landRollPrev = 0;
        landWasActive = false;
      }

      int targetL, targetR, targetElev, targetRud, targetAil1, targetAil2;
      int targetFlaps = 1000;
      // Valeurs gimbal calculées dans chaque branche de mode ci-dessous.
      // Par défaut : position neutre (failsafe), écrasée en mode Manuel et AP.
      int targetGimbalPan = GIMBAL_PAN_FAILSAFE;
      int targetGimbalTilt = GIMBAL_TILT_FAILSAFE;

      // -----------------------------------------------------------------------
      // MODE RTH (prioritaire sur AP)
      // -----------------------------------------------------------------------
      if (rthRequested && homeSet) {
        if (apWasActive) {
          apRollInteg = apRollPrev = 0;
          apPitchInteg = apPitchPrev = 0;
          apAltInteg = apAltPrev = 0;
          apHdgInteg = apHdgPrev = 0;
          apWasActive = false;
        }
        targetL = 1650;
        targetR = 1650;
        targetElev = 1550;
        targetRud = map(constrain((int)TinyGPSPlus::courseTo(telem.lat, telem.lon, homeLat, homeLon) - (int)gps.course.deg(), -40, 40), -40, 40, 1200, 1800);
        targetAil1 = 1500;
        targetAil2 = 1500;
        targetFlaps = 1000;
        // Gimbal : position neutre pendant le RTH pour préserver la stabilité et la vue
        targetGimbalPan = GIMBAL_PAN_FAILSAFE;
        targetGimbalTilt = GIMBAL_TILT_FAILSAFE;

        // -----------------------------------------------------------------------
        // FAILSAFE CRITIQUE : perte signal SANS home GPS
        // -----------------------------------------------------------------------
      } else if (lossOfSignal && !homeSet) {
        // Si on sort de l'AP, on reset les intégrateurs pour éviter le windup
        if (apWasActive) {
          apRollInteg = apRollPrev = 0;
          apPitchInteg = apPitchPrev = 0;
          apAltInteg = apAltPrev = 0;
          apHdgInteg = apHdgPrev = 0;
          apWasActive = false;
        }
        targetL = 1000;
        targetR = 1000;
        targetElev = 1500;
        targetRud = 1500;
        targetAil1 = 1500;
        targetAil2 = 1500;
        targetFlaps = 1000;
        // Gimbal : position neutre pendant le failsafe critique
        targetGimbalPan = GIMBAL_PAN_FAILSAFE;
        targetGimbalTilt = GIMBAL_TILT_FAILSAFE;

        // -----------------------------------------------------------------------
        // AJOUT V0.0.74 : MODE ATTERRISSAGE AUTOMATIQUE
        // -----------------------------------------------------------------------
      } else if (landActive) {
        if (apWasActive) {
          apRollInteg = apRollPrev = 0;
          apPitchInteg = apPitchPrev = 0;
          apAltInteg = apAltPrev = 0;
          apHdgInteg = apHdgPrev = 0;
          apWasActive = false;
        }
        if (!landWasActive) {
          // Première itération de la séquence : on mémorise l'altitude de
          // référence "sol" à cet instant précis (voir note au niveau de la
          // déclaration de landHomeAltitude, plus haut dans le fichier).
          landHomeAltitude = bmpAltitude;
          landingPhase = LANDING_APPROACH;
          landAltInteg = landAltPrev = 0;
          landRollInteg = landRollPrev = 0;
          landWasActive = true;
        }

        // MODIFIÉ V0.0.100 : hauteur sol issue en priorité du LIDAR (mesure directe
        // précise à basse altitude), avec repli baro au-dessus de sa portée.
        float agl = getLandingAGL();  // Hauteur au-dessus du sol (m)
        float hdgCurrent = gps.course.isValid() ? (float)gps.course.deg() : landFinalHeading;

        // ---- Transitions de phase ----
        if (landingPhase == LANDING_APPROACH && agl <= AUTOLAND_FINAL_ALT_AGL) {
          landingPhase = LANDING_FINAL;
          landFinalHeading = hdgCurrent;  // Cap figé : on ne recalcule plus le relèvement vers HOME
          landingFinalStartTime = now;
        }
        if (landingPhase == LANDING_FINAL && (agl <= AUTOLAND_GROUND_ALT_AGL || (now - landingFinalStartTime) > AUTOLAND_FINAL_TIMEOUT_MS)) {
          landingPhase = LANDING_GROUND;
        }

        // ---- PID ROLL (ailes à plat), intégrateur dédié ----
        float rollError = AP_TARGET_ROLL_DEG - imuRoll;
        float rollCorr_us = pidCompute(rollError, dt, AP_ROLL_KP, AP_ROLL_KI, AP_ROLL_KD, landRollInteg, landRollPrev, AP_INTEG_LIMIT_AIL);
        rollCorr_us = constrain(rollCorr_us, -AP_MAX_AIL_CORR, AP_MAX_AIL_CORR);
        targetAil1 = constrain(1500 + (int)rollCorr_us, 1000, 2000);
        targetAil2 = constrain(1500 + (int)rollCorr_us, 1000, 2000);

        if (landingPhase == LANDING_GROUND) {
          // ---- SOL : gaz coupés, gouvernes au neutre (ailerons déjà gérés ci-dessus), volets sortis ----
          targetL = AUTOLAND_IDLE_THROTTLE_PWM;
          targetR = AUTOLAND_IDLE_THROTTLE_PWM;
          targetElev = 1500;
          targetRud = 1500;
          targetFlaps = 2000;

        } else if (landingPhase == LANDING_FINAL) {
          // ---- FINALE : descente terminale, réduction gaz + arrondi progressif ----
          // Plus de PID d'altitude ici (on ne "tient" plus une altitude cible : on
          // descend en tenant une assiette fixe légèrement cabrée), seulement une
          // conversion proportionnelle simple assiette -> µs, comme en approche.
          float pitchError = LAND_TARGET_PITCH_DEG - imuPitch;
          int pitchCorr_us = constrain((int)(pitchError * LAND_PITCH_GAIN_US_PER_DEG), -AP_MAX_ELEV_CORR, AP_MAX_ELEV_CORR);

          // Arrondi : cabré additionnel qui croît linéairement en dessous de AUTOLAND_FLARE_ALT_AGL
          float aglClamped = constrain(agl, 0.0f, AUTOLAND_FLARE_ALT_AGL);
          int flareCorr = (int)map((long)(aglClamped * 100), 0, (long)(AUTOLAND_FLARE_ALT_AGL * 100), AUTOLAND_FLARE_MAX_CORR, 0);

          targetElev = constrain(1500 + pitchCorr_us + flareCorr, 800, 2200);

          // Gaz réduits progressivement avec l'altitude (idle au sol, approche au seuil de finale)
          int aglForThrottle = constrain((int)agl, 0, (int)AUTOLAND_FINAL_ALT_AGL);
          int throttlePWM = map(aglForThrottle, 0, (int)AUTOLAND_FINAL_ALT_AGL, AUTOLAND_IDLE_THROTTLE_PWM, AUTOLAND_APPROACH_THROTTLE_PWM);
          targetL = throttlePWM;
          targetR = throttlePWM;

          // Cap tenu sur l'axe figé à l'entrée en finale (correction proportionnelle simple)
          float hdgError = angleDiff(landFinalHeading, hdgCurrent);
          int hdgCorr = constrain((int)(hdgError * LAND_HDG_GAIN), -LAND_HDG_MAX_CORR, LAND_HDG_MAX_CORR);
          targetRud = constrain(1500 + hdgCorr, 1200, 1800);

          targetFlaps = 2000;  // Volets pleins : portance/traînée max pour ralentir avant le poser

        } else {
          // ---- APPROCHE : navigation vers HOME + descente contrôlée vers l'altitude d'approche ----
          float altError = (landHomeAltitude + AUTOLAND_APPROACH_ALT_AGL) - bmpAltitude;
          float pitchAltCorr = pidCompute(altError, dt, LAND_ALT_KP, LAND_ALT_KI, LAND_ALT_KD, landAltInteg, landAltPrev, LAND_INTEG_LIMIT_ALT);
          pitchAltCorr = constrain(pitchAltCorr, -LAND_ALT_MAX_PITCH_CORR, LAND_ALT_MAX_PITCH_CORR);
          float effectivePitchTarget = LAND_TARGET_PITCH_DEG + pitchAltCorr;
          float pitchError = effectivePitchTarget - imuPitch;
          // Conversion assiette -> µs par un gain proportionnel simple (pas de second
          // étage PID ici : suffisant pour une approche stabilisée à faible vitesse).
          int pitchCorr_us = constrain((int)(pitchError * LAND_PITCH_GAIN_US_PER_DEG), -AP_MAX_ELEV_CORR, AP_MAX_ELEV_CORR);
          targetElev = constrain(1500 + pitchCorr_us, 800, 2200);

          targetL = AUTOLAND_APPROACH_THROTTLE_PWM;
          targetR = AUTOLAND_APPROACH_THROTTLE_PWM;

          // Navigation vers HOME (cap à suivre = relèvement GPS vers le point HOME)
          float bearingToHome = (float)TinyGPSPlus::courseTo(telem.lat, telem.lon, homeLat, homeLon);
          float hdgError = angleDiff(bearingToHome, hdgCurrent);
          int hdgCorr = constrain((int)(hdgError * LAND_HDG_GAIN), -LAND_HDG_MAX_CORR, LAND_HDG_MAX_CORR);
          targetRud = constrain(1500 + hdgCorr, 1200, 1800);

          targetFlaps = 1500;  // Volets mi-course : vol plus lent et stable en approche

          landFinalHeading = hdgCurrent;  // Tenu à jour tant qu'on n'est pas en finale (voir transition ci-dessus)
        }

        targetGimbalPan = GIMBAL_PAN_FAILSAFE;
        targetGimbalTilt = GIMBAL_TILT_FAILSAFE;

        // ---- VOLETS : même lissage progressif que les modes AP/Manuel ----
        if (abs(targetFlaps - (int)currentFlaps) > 2) {
          if (currentFlaps < targetFlaps) currentFlaps += flapSpeed;
          else if (currentFlaps > targetFlaps) currentFlaps -= flapSpeed;
        } else {
          currentFlaps = targetFlaps;
        }

        // -----------------------------------------------------------------------
        // MODE AUTOPILOT
        // -----------------------------------------------------------------------
      } else if (apActive) {
        if (!apWasActive) {
          apLockedAltitude = bmpAltitude;
          apLockedHeading = gps.course.isValid() ? (float)gps.course.deg() : 0.0f;
          apLockedThrottle = map(rxData.throttle, 0, 4095, 1100, 1940);
          apLockedThrottle = constrain(apLockedThrottle, 1200, 1800);
          apRollInteg = apRollPrev = 0;
          apPitchInteg = apPitchPrev = 0;
          apAltInteg = apAltPrev = 0;
          apHdgInteg = apHdgPrev = 0;
          apWasActive = true;
        }

        // ---- PID ALTITUDE (boucle externe de la cascade) ----
        // Sortie : correction en degrés de pitch cible
        float altError = apLockedAltitude - bmpAltitude;
        if (fabsf(altError) < AP_ALT_HOLD_DEADBAND) altError = 0.0f;
        float pitchAltCorr = pidCompute(altError, dt, AP_ALT_KP, AP_ALT_KI, AP_ALT_KD, apAltInteg, apAltPrev, AP_INTEG_LIMIT_ALT);
        pitchAltCorr = constrain(pitchAltCorr, -AP_ALT_MAX_PITCH_CORR, AP_ALT_MAX_PITCH_CORR);

        // Cible de pitch effective = cible de base + correction altitude
        float effectivePitchTarget = AP_TARGET_PITCH_DEG + pitchAltCorr;

        // ---- PID ROLL (IMU → ailerons) ----
        float rollError = AP_TARGET_ROLL_DEG - imuRoll;
        float rollCorr_us = pidCompute(rollError, dt, AP_ROLL_KP, AP_ROLL_KI, AP_ROLL_KD, apRollInteg, apRollPrev, AP_INTEG_LIMIT_AIL);
        rollCorr_us = constrain(rollCorr_us, -AP_MAX_AIL_CORR, AP_MAX_AIL_CORR);

        // ---- PID PITCH (IMU → profondeur) ----
        float pitchError = effectivePitchTarget - imuPitch;
        float pitchCorr_us = pidCompute(pitchError, dt, AP_PITCH_KP, AP_PITCH_KI, AP_PITCH_KD, apPitchInteg, apPitchPrev, AP_INTEG_LIMIT_ELEV);
        pitchCorr_us = constrain(pitchCorr_us, -AP_MAX_ELEV_CORR, AP_MAX_ELEV_CORR);

        // ---- PID CAP (GPS → différentielle moteurs) ----
        float hdgCurrent = gps.course.isValid() ? (float)gps.course.deg() : apLockedHeading;
        float hdgError = angleDiff(apLockedHeading, hdgCurrent);
        float hdgCorr_us = pidCompute(hdgError, dt, AP_HDG_KP, AP_HDG_KI, AP_HDG_KD, apHdgInteg, apHdgPrev, AP_INTEG_LIMIT_HDG);
        hdgCorr_us = constrain(hdgCorr_us, -AP_MAX_DIFF_CORR, AP_MAX_DIFF_CORR);

        // ---- CONTRIBUTION STICK PILOTE (fly-by-wire) ----
        // Le stick est centré autour de 2047 (ADC 12 bits).
        // On mappe [-2047, +2047] → [-authority, +authority] µs.
        int stickAil = (int)(((float)rxData.ailerons - 2047.5f) / 2047.5f * AP_STICK_AUTHORITY_AIL);
        int stickElev = (int)(((float)rxData.elevator - 2047.5f) / 2047.5f * AP_STICK_AUTHORITY_ELEV);

        // ---- COMMANDE AILERONS (AP + stick pilote) ----
        // Canal 4 et 5 : neutre 1500 µs.
        // rollCorr_us positif → aile gauche descend → virage droite
        targetAil1 = constrain(1500 + (int)rollCorr_us + stickAil, 1000, 2000);
        targetAil2 = constrain(1500 + (int)rollCorr_us + stickAil, 1000, 2000);

        // ---- COMMANDE PROFONDEUR (AP + stick pilote) ----
        // Canal 2 : neutre 1500 µs. pitchCorr_us positif → cabre
        targetElev = constrain(1500 + (int)pitchCorr_us + stickElev, 800, 2200);

        // ---- COMMANDE MOTEURS (throttle figé ± diff cap) ----
        // hdgCorr_us positif → erreur vers la droite → accélérer gauche, ralentir droite
        targetL = constrain(apLockedThrottle + (int)hdgCorr_us, 1100, 1940);
        targetR = constrain(apLockedThrottle - (int)hdgCorr_us, 1100, 1940);

        // ---- GOUVERNE DE DIRECTION : neutre en AP ----
        // (la direction est gérée par la différentielle moteurs)
        targetRud = 1500;

        if (rxData.flaps == 1) targetFlaps = 1500;
        else if (rxData.flaps == 2) targetFlaps = 2000;
        else targetFlaps = 1000;

        // ---- VOLETS : inchangés, position commandée par le pilote ----
        if (abs(targetFlaps - (int)currentFlaps) > 2) {
          if (currentFlaps < targetFlaps) currentFlaps += flapSpeed;
          else if (currentFlaps > targetFlaps) currentFlaps -= flapSpeed;
        } else {
          currentFlaps = targetFlaps;
        }

        // ---- GIMBAL EN MODE AP : le pilote garde le contrôle total de la gimbal ----
        // Même en autopilot, l'opérateur oriente la caméra librement via les joysticks dédiés.
        targetGimbalPan = map(rxData.gimbalPan, 0, 4095, GIMBAL_PAN_MIN, GIMBAL_PAN_MAX);
        targetGimbalTilt = map(rxData.gimbalTilt, 0, 4095, GIMBAL_TILT_MIN, GIMBAL_TILT_MAX);
        targetGimbalPan = constrain(targetGimbalPan, GIMBAL_PAN_MIN, GIMBAL_PAN_MAX);
        targetGimbalTilt = constrain(targetGimbalTilt, GIMBAL_TILT_MIN, GIMBAL_TILT_MAX);

        // -----------------------------------------------------------------------
        // MODE MANUEL
        // -----------------------------------------------------------------------
      } else {
        if (apWasActive) {
          apRollInteg = apRollPrev = 0;
          apPitchInteg = apPitchPrev = 0;
          apAltInteg = apAltPrev = 0;
          apHdgInteg = apHdgPrev = 0;
          apWasActive = false;
        }

        // --- CALCUL POUSSÉE DIFFÉRENTIELLE ---
        const int THROTTLE_MIN = 1100;
        const int THROTTLE_MAX = 1940;
        const int RANGE = THROTTLE_MAX - THROTTLE_MIN;

        float throttleRatio = (float)rxData.throttle / 4095.0;
        float rudderRatio = ((float)rxData.rudder - 2047.5) / 2047.5;
        float diffAdjustment = rudderRatio * 0.20 * throttleRatio;
        int throttlePWM = map(rxData.throttle, 0, 4095, THROTTLE_MIN, THROTTLE_MAX);

        targetL = constrain(throttlePWM + (int)(diffAdjustment * RANGE), THROTTLE_MIN, THROTTLE_MAX);
        targetR = constrain(throttlePWM - (int)(diffAdjustment * RANGE), THROTTLE_MIN, THROTTLE_MAX);

        // --- CALCUL PROFONDEUR / VOLETS ---
        int baseElev = map(rxData.elevator + rxData.trimElevator, 0, 4095, 800, 2200);
        if (rxData.flaps == 1) targetFlaps = 1500;
        else if (rxData.flaps == 2) targetFlaps = 2000;
        else targetFlaps = 1000;

        if (abs(targetFlaps - (int)currentFlaps) > 2) {
          if (currentFlaps < targetFlaps) currentFlaps += flapSpeed;
          else if (currentFlaps > targetFlaps) currentFlaps -= flapSpeed;
        } else {
          currentFlaps = targetFlaps;
        }

        // --- COMPENSATION PROFONDEUR ---
        int elevCompensation = 0;
        const int HALF_ELEV_COURSE = 700;
        if (currentFlaps <= 1500.0) {
          float pct = (currentFlaps - 1000.0) / 500.0;
          elevCompensation = (int)(pct * (-0.04 * HALF_ELEV_COURSE));
        } else {
          float pct = (currentFlaps - 1500.0) / 500.0;
          int comp20 = (int)(-0.04 * HALF_ELEV_COURSE);
          int comp40 = (int)(-0.09 * HALF_ELEV_COURSE);
          elevCompensation = comp20 + (int)(pct * (comp40 - comp20));
        }
        targetElev = constrain(baseElev + elevCompensation, 800, 2200);

        // --- AILERONS ---
        targetRud = map(rxData.rudder + rxData.trimRudder, 0, 4095, 1000, 2000);
        targetAil1 = map(rxData.ailerons, 0, 4095, 1000, 2000);
        targetAil2 = map(rxData.ailerons, 0, 4095, 1000, 2000);

        // --- GIMBAL EN MODE MANUEL : contrôle direct par les joysticks dédiés ---
        targetGimbalPan = map(rxData.gimbalPan, 0, 4095, GIMBAL_PAN_MIN, GIMBAL_PAN_MAX);
        targetGimbalTilt = map(rxData.gimbalTilt, 0, 4095, GIMBAL_TILT_MIN, GIMBAL_TILT_MAX);
        targetGimbalPan = constrain(targetGimbalPan, GIMBAL_PAN_MIN, GIMBAL_PAN_MAX);
        targetGimbalTilt = constrain(targetGimbalTilt, GIMBAL_TILT_MIN, GIMBAL_TILT_MAX);
      }


      // -----------------------------------------------------------------------
      // APPLICATION DES COMMANDES SUR LA PCA9685
      // -----------------------------------------------------------------------

      pwmWriteMicroseconds(0, targetL);
      pwmWriteMicroseconds(1, targetR);
      pwmWriteMicroseconds(2, targetElev);
      pwmWriteMicroseconds(3, (int)currentFlaps);
      pwmWriteMicroseconds(6, (int)currentFlaps);
      pwmWriteMicroseconds(7, targetRud);
      pwmWriteMicroseconds(4, targetAil1);
      pwmWriteMicroseconds(5, targetAil2);

      // --- APPLICATION DES COMMANDES GIMBAL (canaux 8 et 9 de la PCA9685) ---
      // La fonction applyGimbal() centralise l'écriture sur les deux canaux servo.
      applyGimbal(targetGimbalPan, targetGimbalTilt);

      telem.ail1PWM = targetAil1;
      telem.ail2PWM = targetAil2;
      telem.flaps = map((int)currentFlaps, 1000, 2000, 0, 20);

      if (Serial && (now - lastDebugTime > 500)) {
        debugControlData(rxData);
        lastDebugTime = now;
      }
      xSemaphoreGive(mutex);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// --- LECTURE BAROMÉTRIQUE BMP280 ---
void updateBarometer() {
  if (!bmpReady) return;
  float p = bmp.readPressure() / 100.0F;
  float t = bmp.readTemperature();
  float a = bmp.readAltitude(SEA_LEVEL_HPA);
  // Filtre basique : on rejette les valeurs aberrantes
  if (p > 800.0 && p < 1100.0) {
    if (xSemaphoreTake(mutex, 0)) {
      bmpPressure = p;
      bmpTemperature = t;
      bmpAltitude = a;
      xSemaphoreGive(mutex);
    }
  }
}

// --- AJOUT V0.0.100 : LECTURE LIDAR TF-Luna (hauteur sol) ---
// Protocole identique au sketch test_lidar_i2c.ino, mais sur le bus I2C_PCA
// (celui des autres capteurs) : on pointe sur le registre 0x00 puis on lit 6
// octets (distance / signal / température). Seule la distance (= hauteur sol)
// est conservée ; la température est ignorée (inutile ici). Une lecture n'est
// considérée fiable (lidarValid) que si le signal est suffisant et la distance
// dans la portée exploitable du capteur (~8 m).
void updateLidar() {
  if (!lidarReady) return;

  I2C_PCA.beginTransmission(LIDAR_I2C_ADDR);
  I2C_PCA.write(0x00);
  if (I2C_PCA.endTransmission(false) != 0) return;

  I2C_PCA.requestFrom((int)LIDAR_I2C_ADDR, 6);
  if (I2C_PCA.available() < 6) return;

  uint16_t distance = I2C_PCA.read() | (I2C_PCA.read() << 8);  // cm
  uint16_t strength = I2C_PCA.read() | (I2C_PCA.read() << 8);  // force du signal
  I2C_PCA.read();                                              // température (octet bas) - ignorée
  I2C_PCA.read();                                              // température (octet haut) - ignorée

  bool valid = (strength >= LIDAR_MIN_STRENGTH) && (strength != 0xFFFF) && (distance > 0) && (distance <= LIDAR_MAX_VALID_CM);

  if (xSemaphoreTake(mutex, 0)) {
    lidarHeightCm = distance;
    lidarStrength = strength;
    lidarValid = valid;
    xSemaphoreGive(mutex);
  }
}

// --- AJOUT V0.0.100 : HAUTEUR SOL (AGL) POUR L'ATTERRISSAGE AUTOMATIQUE ---
// Le LIDAR fournit une mesure directe et précise de la hauteur sol, mais sur
// une portée limitée (~8 m). On le privilégie donc dès que sa lecture est
// fiable (phases FINALE / arrondi / SOL, où la précision compte le plus), et
// on retombe automatiquement sur l'altitude barométrique relative
// (bmpAltitude - landHomeAltitude) au-dessus de sa portée (approche haute).
// Appelée sous mutex depuis TaskControlFlight().
float getLandingAGL() {
  if (lidarValid) return lidarHeightCm / 100.0f;
  return bmpAltitude - landHomeAltitude;
}

void debugControlData(const ControlData &data) {
  if (Serial.availableForWrite() < 10) return;
  uint32_t now = millis();
  bool lossOfSignal = (now - lastPacketTime > TIMEOUT_MS);

  int throttlePWM = map(data.throttle, 0, 4095, 1100, 1940);
  int rudderOffset = map(data.rudder, 0, 4095, -DIFF_RANGE, DIFF_RANGE);
  int motorLeft = constrain(throttlePWM - rudderOffset, 1000, 1940);
  int motorRight = constrain(throttlePWM + rudderOffset, 1000, 1940);

  const char *flapPosStr = (data.flaps == 1) ? "MID" : ((data.flaps == 2) ? "FULL" : "NEUTRE");
  int flapPWM = (data.flaps == 1) ? 1500 : ((data.flaps == 2) ? 2000 : 1000);

  // MODIFIÉ V0.0.74 : ajout du numéro de version et de la puissance LoRa courante (miroir RC v0.0.80).
  Serial.printf("\n--- [ RX DEBUG ] --- (%s | Puissance LoRa: %d) ---\n", FIRMWARE_VERSION, currentTxPower);
  if (lossOfSignal) {
    Serial.printf("AIRCRAFT ID: 0x%08X | STATUS: [!!! FAILSAFE !!!] | TOTAL FS: %u\n", data.aircraftId, failSafeCount);
  } else {
    Serial.printf("AIRCRAFT ID: 0x%08X | STATUS: LINK OK | GLITCHS: %u | FAILSAFES: %u\n", data.aircraftId, glitchCount, failSafeCount);
  }

  Serial.print(F("MODE    | "));
  if (lossOfSignal) {
    if (homeSet) {
      Serial.printf("AUTO-RTH (Signal LoRa perdu depuis %lu ms) : ACTIF\n", now - lastPacketTime);
    } else {
      // MODIFIÉ V0.0.75 : message clarifié. La vraie cause de cette coupure
      // moteurs N'EST PAS "pas de GPS" au sens d'une absence de fix (lat/lon
      // peuvent très bien être valides) : c'est l'absence de HOME fixé
      // (gps.satellites.value() > HOME_MIN_SATELLITES au moment du premier
      // fix, cf. updateGPS()) COMBINÉE à une perte de signal LoRa RC.
      // On affiche donc explicitement le nombre de satellites courant pour
      // rendre la vraie cause immédiatement visible en debug.
      int satsNow = gps.satellites.isValid() ? (int)gps.satellites.value() : 0;
      Serial.printf("FAILSAFE CRITIQUE (Signal LoRa perdu depuis %lu ms | HOME non fixé - Sats: %d/%d requis) : COUPURE MOTEURS\n",
                    now - lastPacketTime, satsNow, HOME_MIN_SATELLITES + 1);
    }
  } else if (data.returnToHome) {
    Serial.println(homeSet ? F("RTH MANUEL (Switch) : ACTIF") : F("RTH DEMANDÉ | /!\\ ERREUR : HOME NON FIXÉ"));
  } else if (data.autoPilot) {
    Serial.println(F("AUTOPILOT : ACTIF"));
  } else {
    Serial.println(F("MANUEL"));
  }
  // MODIFIÉ V0.0.74 : affiche désormais la phase de la séquence quand le mode est actif.
  // NOTE : landActive est recalculé ici (même logique que TaskControlFlight) car cette
  // variable est locale à TaskControlFlight et n'est pas partagée directement.
  bool landActiveDbg = data.autoLanding && !(data.returnToHome || lossOfSignal) && homeSet;
  if (landActiveDbg) {
    const char *phaseStr = (landingPhase == LANDING_APPROACH) ? "APPROCHE" : (landingPhase == LANDING_FINAL) ? "FINALE"
                                                                                                             : "SOL";
    Serial.printf("AUTO LANDING | ACTIF - Phase: %-8s | AGL: %6.1f m | Dist Home: %6.1f m\n",
                  phaseStr, getLandingAGL(),
                  (float)TinyGPSPlus::distanceBetween(telem.lat, telem.lon, homeLat, homeLon));
  } else {
    Serial.printf("AUTO LANDING | Commande reçue: %s%s\n", data.autoLanding ? "ACTIF" : "OFF",
                  (data.autoLanding && !homeSet) ? " (en attente du FIX HOME GPS)" : "");
  }

  if (data.autoPilot && !data.returnToHome && !lossOfSignal) {
    Serial.printf("AP CIBLES| AltFigée: %6.1f m | CapFigé: %5.1f ° | ThroFigé: %4d µs\n", apLockedAltitude, apLockedHeading, apLockedThrottle);
    Serial.printf("AP ERRORS| Roll: %+6.2f ° | Pitch: %+6.2f ° | Alt: %+6.2f m | Cap: %+6.2f °\n", AP_TARGET_ROLL_DEG - imuRoll, AP_TARGET_PITCH_DEG - imuPitch, apLockedAltitude - bmpAltitude, angleDiff(apLockedHeading, gps.course.isValid() ? (float)gps.course.deg() : apLockedHeading));
  }

  Serial.printf("MOTORS  | L: %4d | R: %4d (Diff: %d)\n", motorLeft, motorRight, rudderOffset);
  Serial.printf("RUDDER  | Brut: %4d | Offset Diff: %+5d\n", data.rudder, rudderOffset);
  Serial.printf("GIMBAL  | Pan=%d Tilt=%d\n", data.gimbalPan, data.gimbalTilt);

  // --- SÉPARATEUR VISUEL ---
  // Isole la section "commandes reçues" (sticks, moteurs, gimbal) de la section
  // "télémétrie avion" ci-dessous (signal radio, batterie ESC, baro, IMU).
  Serial.println(F("..............................................."));

  //Serial.printf("ESC TLM | Tension: %5.2f V | Courant: %5.2f A | Conso: %4d mAh | Temp: %3d °C\n", liveEscData.voltage, liveEscData.current, liveEscData.consumption_mah, liveEscData.temperature);
  Serial.printf("[SIGNAL]  | RSSI Avion: %.1f dBm | SNR Avion: %.1f dB\n", lastRSSI, lastSNR);
  if (bmpReady) Serial.printf("[BARO]    | Pression: %7.2f hPa | Altitude: %6.1f m\n", bmpPressure, bmpAltitude);
  if (imuReady) Serial.printf("[IMU ATT] | Roll: %+7.2f ° | Pitch: %+7.2f ° | YawRate: %+7.2f °/s\n", imuRoll, imuPitch, imuYawRate);
  // --- AJOUT V0.0.100 : hauteur sol LIDAR (seule donnée utile affichée) ---
  Serial.printf("[LIDAR]   | Hauteur: %u cm\n", lidarHeightCm);

  // --- AJOUT V0.0.70 : état de réception GPS (module L76K) ---
  {
    GpsState gState = getGpsState();
    const char *gpsStateStr = (gState == GPS_NO_DATA) ? "ABSENT" : (gState == GPS_SEARCHING) ? "RECHERCHE"
                                                                                             : "FIX OK";
    Serial.printf("[GPS]   | Etat: %-9s | Sats: %2d | HDOP: %4.1f | Lat: %9.5f | Lon: %9.5f | Vit: %5.1f km/h | Cap: %5.1f°\n",
                  gpsStateStr,
                  gps.satellites.isValid() ? (int)gps.satellites.value() : 0,
                  gps.hdop.isValid() ? gps.hdop.hdop() : 99.9f,
                  gps.location.isValid() ? gps.location.lat() : 0.0f,
                  gps.location.isValid() ? gps.location.lng() : 0.0f,
                  gps.speed.isValid() ? gps.speed.kmph() : 0.0f,
                  gps.course.isValid() ? gps.course.deg() : 0.0f);
  }

  Serial.println(F("--------------------"));
}

void TaskLoRaReceive(void *pvParameters) {
  for (;;) {
    updateGPS();
    // --- AJOUT V0.0.71 : détection immédiate de l'acquisition d'un fix GPS ---
    checkGpsFixBeep();
    //updateESCTelemetry();
    updateBarometer();
    updateLidar();  // AJOUT V0.0.100 : rafraîchit la hauteur sol (LIDAR TF-Luna)
    updateIMU();

    // --- AJOUT V0.0.70 : rafraîchissement périodique de l'indicateur GPS OLED ---
    // Non bloquant : simple comparaison de millis(), le rafraîchissement de
    // l'écran (I2C) ne se déclenche qu'une fois par seconde.
    uint32_t nowOled = millis();
    if (nowOled - lastGpsOledUpdate >= GPS_OLED_REFRESH_MS) {
      updateGpsStatusOLED();
      lastGpsOledUpdate = nowOled;
    }

    // MODIFIÉ V0.0.78 : réception non-bloquante par interruption DIO1, au lieu
    // de l'ancien radio.receive() bloquant (voir bloc de commentaires détaillé
    // au niveau de la déclaration de plRxFlag, plus haut dans le fichier).
    // L'écoute continue est armée dans setupLoRaFastMode() et systématiquement
    // réarmée ci-dessous après chaque paquet traité (et après chaque
    // transmission de télémétrie), par prudence vis-à-vis du comportement IRQ
    // du SX126x qui peut nécessiter un réarmement explicite même en "continu".
    if (plRxFlag) {
      plRxFlag = false;
      int state = radio.readData((uint8_t *)&rxData, sizeof(ControlData));
      radio.startReceive();  // Réarmement immédiat : la fenêtre d'écoute doit rester ouverte en continu

      if (state == RADIOLIB_ERR_NONE) {
        if (rxData.aircraftId == MY_AIRCRAFT_ID) {
          float fErr = radio.getFrequencyError();
          if (xSemaphoreTake(mutex, portMAX_DELAY)) {
            lastPacketTime = millis();
            lastRSSI = radio.getRSSI();
            lastSNR = radio.getSNR();
            packetCount++;
            telem.freqError = fErr;
            xSemaphoreGive(mutex);
          }

          if (packetCount % 5 == 0) {
            if (xSemaphoreTake(mutex, portMAX_DELAY)) {
              telem.heading = gps.course.isValid() ? (uint16_t)gps.course.deg() : 0;
              
              // Tension Batterie LiPo
              uint16_t rawAdc = analogRead(BAT_ADC_PIN); // GPIO3 sur XIAO ESPS3
              telem.batteryVoltage = (rawAdc / 4095.0f) * 3.3f * BATTERY_CALIBRATION;

              // Serial.println(rawAdc);
              // Serial.println(telem.batteryVoltage);
              // Serial.println(analogReadMilliVolts(BAT_ADC_PIN));

              telem.currentRSSI = lastRSSI;
              telem.currentSNR = lastSNR;
              telem.autoPilot = rxData.autoPilot;
              telem.returnToHome = rxData.returnToHome;
              // --- AJOUT V0.0.73 : échos de commande vers la RC (voir avertissement struct) ---
              telem.rudder = rxData.rudder;
              telem.gimbalPan = rxData.gimbalPan;
              telem.gimbalTilt = rxData.gimbalTilt;
              telem.autoLanding = rxData.autoLanding;
              telem.flightTime = millis();
              // --- CORRECTIF V0.0.68 : champ altitude desormais rempli (voir struct) ---
              telem.altitude = bmpReady ? bmpAltitude : 0.0f;
              // --- AJOUT V0.0.68 : attitude IMU pour horizon artificiel ---
              telem.attRoll = imuRoll;
              telem.attPitch = imuPitch;
              telem.attYawRate = imuYawRate;
              telem.imuValid = imuReady;
              // --- AJOUT V0.0.69 : vitesse sol pour instrument "badin" ---
              telem.groundSpeedKmh = gps.speed.isValid() ? (float)gps.speed.kmph() : 0.0f;
              // --- AJOUT V0.0.74 : pression baro + HDOP GPS, renvoyés à la RC ---
              telem.pressure = bmpReady ? bmpPressure : 0.0f;
              telem.hdop = gps.hdop.isValid() ? (float)gps.hdop.hdop() : 99.9f;
              // --- AJOUT V0.0.76 : compteur de failsafes, lu directement (même
              // mutex déjà pris ; failSafeCount est écrit sous mutex dans TaskControlFlight).
              telem.failSafeCount = failSafeCount;
              // --- AJOUT V0.0.100 : hauteur sol LIDAR (cm) renvoyée à la RC ---
              telem.lidarHeightCm = lidarHeightCm;
              xSemaphoreGive(mutex);
            }
            radio.transmit((uint8_t *)&telem, sizeof(TelemetryData));
            // MODIFIÉ V0.0.78 : réarmement OBLIGATOIRE après la transmission -
            // c'est précisément ce réarmement qui échouait avec l'ancien
            // radio.receive() bloquant (bug SX1262 documenté : le module ne
            // rebascule pas toujours fiablement en RX après un transmit()).
            // Avec startReceive() explicite ici, plus de dépendance à un
            // comportement implicite du receive() bloquant.
            radio.startReceive();
          }
        }
      } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
        // AJOUT V0.0.77 : "glitch" = un paquet a bien été détecté et démodulé
        // (préambule + en-tête valides) mais son contenu a échoué au contrôle
        // CRC - signe d'un lien marginal (parasite, léger fading, distance
        // limite...), à distinguer d'un simple silence radio (plus de notion
        // de RX_TIMEOUT à gérer ici : en non-bloquant, l'absence de paquet ne
        // déclenche simplement pas plRxFlag). Indicateur de dégradation
        // progressive du lien, complémentaire de failSafeCount qui ne compte
        // lui que les coupures franches (> TIMEOUT_MS sans paquet).
        if (xSemaphoreTake(mutex, pdMS_TO_TICKS(5))) {
          glitchCount++;
          xSemaphoreGive(mutex);
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void updateGPS() {
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }
  if (xSemaphoreTake(mutex, portMAX_DELAY)) {
    if (gps.location.isValid()) {
      telem.lat = gps.location.lat();
      telem.lon = gps.location.lng();
      telem.gpsStatus = 1;
      if (!homeSet && gps.satellites.value() > HOME_MIN_SATELLITES) {
        homeLat = telem.lat;
        homeLon = telem.lon;
        homeSet = true;
        Serial.println(F(">>> HOME POSITION SET !"));
      }
    } else {
      telem.gpsStatus = 0;
    }
    telem.satellites = gps.satellites.value();
    xSemaphoreGive(mutex);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LORA_ANT_SW, OUTPUT);
  digitalWrite(LORA_ANT_SW, HIGH);

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  I2C_PCA.begin(PCA_SDA, PCA_SCL, 400000);

  // --- INITIALISATION ÉCRAN OLED ---
  if (display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS)) {
    oledReady = true;
    showVersionOnOLED();
    Serial.printf("[OLED] Écran initialisé - Firmware %s\n", FIRMWARE_VERSION);
  } else {
    Serial.println(F("[OLED] Écran non détecté !"));
  }

  SerialGPS.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  //SerialESC.begin(ESC_BAUDRATE, SERIAL_8N1, ESC_RX_PIN, -1);

  // --- AJOUT V0.0.71 : initialisation du buzzer passif (bip de confirmation fix GPS) ---
  pinMode(BUZZER_PIN, OUTPUT);

  // Lecture tension de la batterie LiPo 3S
  pinMode(BAT_ADC_PIN, INPUT);
  analogSetAttenuation(ADC_11db);  // Plage complète mesurée jusqu'à ~3,3V
  analogReadResolution(12);

  // Initialize sensors basic state
  if (bmp.begin(0x76)) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2, Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16, Adafruit_BMP280::STANDBY_MS_500);
    bmpReady = true;
  }
  if (checkBMI270Present() && imu.begin()) {
    imuReady = true;
    lastImuTime = micros();
  }

  // --- AJOUT V0.0.100 : détection du LIDAR TF-Luna (I2C 0x10 sur I2C_PCA) ---
  // Simple ACK I2C : si le capteur répond, on active sa lecture périodique.
  I2C_PCA.beginTransmission(LIDAR_I2C_ADDR);
  if (I2C_PCA.endTransmission() == 0) {
    lidarReady = true;
    Serial.println(F("[LIDAR] TF-Luna détecté (0x10)."));
  } else {
    Serial.println(F("[ERREUR] LIDAR TF-Luna non détecté ! Vérifier le câblage I2C."));
  }

  // --- VÉRIFICATION DES CAPTEURS AU BOOT ---
  // On n'interrompt pas le démarrage (le pilotage manuel reste possible sans
  // baro/IMU), mais on prévient clairement le pilote au sol : Serial + OLED.
  if (!bmpReady) {
    Serial.println(F("[ERREUR] BMP280 non détecté ! Vérifier le câblage I2C."));
  }
  if (!imuReady) {
    Serial.println(F("[ERREUR] BMI270 non détecté ! Vérifier le câblage I2C."));
  }
  if (!bmpReady || !imuReady) {
    oledShowLines(
      "!! CAPTEUR KO !!",
      bmpReady ? "BMP280 : OK" : "BMP280 : ABSENT",
      imuReady ? "BMI270 : OK" : "BMI270 : ABSENT",
      "Verif. cablage I2C");
    delay(2500);  // Laisse le temps de lire le message avant la suite du boot
  }

  mutex = xSemaphoreCreateMutex();
  if (xSemaphoreTake(mutex, portMAX_DELAY)) {
    telem.autoPilot = false;
    telem.returnToHome = false;
    // AJOUT V0.0.73
    telem.rudder = 2048;
    telem.gimbalPan = 2048;
    telem.gimbalTilt = 2048;
    telem.autoLanding = false;
    telem.lidarHeightCm = 0;  // AJOUT V0.0.100
    xSemaphoreGive(mutex);
  }

  // --- LOGIQUE DE VÉRIFICATION POUR ENTRÉE EN MODE OTA ---
  setupLoRaFastMode();
  Serial.println(F("[System] Attente du signal de la télécommande (5s)..."));

  unsigned long startWait = millis();
  bool transmitterFound = false;

  // On écoute pendant 5 secondes si un paquet LoRa arrive
  // MODIFIÉ V0.0.78 : passage en non-bloquant (plRxFlag), cohérent avec le
  // reste (l'écoute continue a déjà été armée dans setupLoRaFastMode()).
  while (millis() - startWait < 5000) {
    if (plRxFlag) {
      plRxFlag = false;
      int state = radio.readData((uint8_t *)&rxData, sizeof(ControlData));
      radio.startReceive();  // Réarmement systématique
      if (state == RADIOLIB_ERR_NONE && rxData.aircraftId == MY_AIRCRAFT_ID) {
        transmitterFound = true;
        lastPacketTime = millis();
        break;  // Émetteur trouvé, on sort immédiatement !
      }
    }
    delay(10);
  }

  if (!transmitterFound) {
    // AUCUNE TÉLÉCOMMANDE TROUVÉE -> PORTAIL OTA MAINTENANCE ACTIVÉ
    Serial.println(F("[OTA] Aucune télécommande détectée. Lancement du mode OTA..."));

    // Fermeture propre du module Radio pour libérer la bande et l'énergie au sol
    radio.sleep();

    // Initialisation du Point d'Accès Wi-Fi
    WiFi.softAP("Avion-OTA", "");
    IPAddress IP = WiFi.softAPIP();
    Serial.print(F("[OTA] Point d'accès démarré. SSID: Avion-OTA\n"));
    Serial.print(F("[OTA] Connectez-vous et allez sur http://"));
    Serial.print(IP);
    Serial.println(F("/update"));

    // --- AFFICHAGE OLED DU MODE OTA ---
    String ipLine = "IP: " + IP.toString();
    oledShowLines(
      "*** MODE OTA ***",
      "Pas de RC detectee",
      "WiFi: Avion-OTA",
      ipLine.c_str());

    // Configuration d'ElegantOTA
    server.on("/", HTTP_GET, []() {
      server.send(200, "text/plain", "Portail Maintenance Avion. Allez sur /update pour flasher.");
    });

    ElegantOTA.begin(&server);
    server.begin();

    // Boucle infinie de maintenance : l'ESP32 ne fera QUE gérer l'OTA
    while (true) {
      server.handleClient();
      ElegantOTA.loop();
      delay(2);
    }
  }

  // SI ÉMETTEUR TROUVÉ : ON CONTINUE LE BOOT NORMAL ET LE WI-FI NE S'ALLUME JAMAIS
  Serial.println(F("[System] Télécommande détectée. Démarrage des tâches de Vol..."));
  oledShowLines(
    "RC detectee",
    "Demarrage vol...",
    bmpReady ? "Baro: OK" : "Baro: ABSENT",
    imuReady ? "IMU : OK" : "IMU : ABSENT");

  // MODIFIE V0.0.103 : initialisation via la librairie Seeed-PCA9685.
  // pwm.init() effectue le restart du PCA9685 puis regle une frequence par
  // defaut ; on impose ensuite 50 Hz (20 ms), standard pour les servos.
  pwm.init(PCA9685_ADDR);
  pwm.setFrequency(50);

  // Démarrage de l'architecture temps réel FreeRTOS
  xTaskCreatePinnedToCore(TaskLoRaReceive, "LoRaRecv", 8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskControlFlight, "FlightCtrl", 8192, NULL, 2, NULL, 1);
}

void loop() {
  vTaskDelete(NULL);  // Détruit la loop Arduino standard au profit des tâches FreeRTOS
}