#include <Adafruit_GFX.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h>
#include "bitmap.h"
#include <EEPROM.h>
#include <ESP32Servo.h>

// Registres du capteur
#define BU27006MUC_ADDRESS 0x38
#define REG_MODE_CONTROL1 0x41
#define REG_MODE_CONTROL2 0x42
#define REG_MODE_CONTROL3 0x43
#define REG_RED_DATA_LSB 0x50
#define REG_RED_DATA_MSB 0x51
#define REG_GREEN_DATA_LSB 0x52
#define REG_GREEN_DATA_MSB 0x53
#define REG_BLUE_DATA_LSB 0x54
#define REG_BLUE_DATA_MSB 0x55
#define gainx1 0x03 // gain x1 sans temps attente
#define gainx4 0x13 // gain x4 sans temps attente
#define gainx1_100ms 0x02   //gain x1 avec temps de mesure de 100ms
#define gainx1_55ms 0x01    //gain x1 avec temps de mesure de 55ms
#define gainx4_100ms 0x12   //gain x4 avec temps de mesure de 100ms
#define gainx4_55ms 0x11    //gain x4 avec temps de mesure de 55ms
#define gainx32_100ms 0x22
#define gainx32_55ms 0x21
#define gainx32_notime 0x20
#define gainx128_100ms 0x32
#define gainx128_55ms 0x31
#define gainx128_notime 0x30
#define LED 17
#define SWITCH 27
#define TFT_DC 4
#define TFT_CS 2
#define TFT_RESET 13
#define longueur_tableau 5

#define position_rouge 0
#define position_orange 40
#define position_vert 85
#define position_mauve 145
#define position_jaune 250
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RESET);

#define CS_PIN  5
XPT2046_Touchscreen ts(CS_PIN); 

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240
#define BUTTON_WIDTH 100
#define BUTTON_HEIGHT 100


enum AppState { LOADING, MENU, START, CONFIG ,THEME_CONFIG, CALIBRATION_CONFIG,CONTROLE_CONFIG};
AppState appState = LOADING;

// Coordonnées des Skittles
int skittleX[5] = {60, 100, 140, 180, 220}; // Seront mis à jour pour être centrés
int skittleY = 120; // Milieu de l'écran (240/2)

int16_t skittle_rouge_mesure_calibration[3] = {0,0,0};
int16_t skittle_orange_mesure_calibration[3] = {0,0,0};
int16_t skittle_vert_mesure_calibration[3] = {0,0,0};
int16_t skittle_mauve_mesure_calibration[3] = {0,0,0};
int16_t skittle_jaune_mesure_calibration[3] = {0,0,0};
int16_t aucun_skittle_mesure_calibration[3] = {0,0,0};

int compteur_rouge = 0;
int compteur_orange = 0;
int compteur_vert = 0;
int compteur_jaune = 0;
int compteur_mauve = 0;
int compteur_aucun = 0;
int compteur[5];
int compteur_total = 0;
uint8_t calibration_compteur = 0;
uint16_t skittleColors[5] = {ILI9341_RED, 0xFD20, ILI9341_GREEN, 0x780F, ILI9341_YELLOW}; // Rouge, orange, vert, mauve, jaune

uint16_t theme_color = ILI9341_BLACK;
uint8_t highByte;
uint8_t lowByte;
int interrupt_flag_100ms = 0;
int interrupt_flag = 0;
int Seconde = 0;
int Minute = 0;
hw_timer_t *My_timer = NULL;
hw_timer_t *My_timer_100ms = NULL;

uint16_t echantillon_rouge[longueur_tableau];
uint16_t echantillon_bleu[longueur_tableau];
uint16_t echantillon_vert[longueur_tableau];
uint16_t red_data;            //-------------------variable capteur couleur-------------------
uint16_t blue_data;
uint16_t green_data;
uint8_t data_flag;
uint8_t read_index = 0;
uint16_t total_rouge = 0;
uint16_t total_bleu = 0;
uint16_t total_vert = 0;  //---------------------------------------------------------------------
uint32_t score_rouge = 0;
uint32_t score_orange = 0;
uint32_t score_vert = 0;
uint32_t score_mauve = 0;
uint32_t score_jaune = 0;
uint32_t score_aucun = 0;
char couleur_detecter[5] = {'A','A','A','A','A'};
uint8_t index_liste_attente_detection_couleur = 0;
uint8_t index_a_lire;

int servoPin = 12;
int servoPin2 = 14;
uint8_t flag_skittle_position = 1;
uint8_t flag_fini = 0;
uint8_t compteur_skittle_position = 0;
Servo servo_redirection;  // create servo object to control a servo
Servo servo_360;

bool boutton_spin_state = false;

uint16_t readColorData(uint8_t msb_reg, uint8_t lsb_reg)  //---------------------fonction capteur de couleur-------------------
{
  Wire.beginTransmission(BU27006MUC_ADDRESS);
  Wire.write(lsb_reg);
  Wire.endTransmission();
  Wire.requestFrom(BU27006MUC_ADDRESS, 1);
  uint8_t lsb = Wire.read();

  Wire.beginTransmission(BU27006MUC_ADDRESS);
  Wire.write(msb_reg);
  Wire.endTransmission();
  Wire.requestFrom(BU27006MUC_ADDRESS, 1);
  uint8_t msb = Wire.read();

  return (msb << 8) | lsb; // Combinaison de l'octet de poids fort et faible
}

uint8_t reset_mesurement_flag(void){
  Wire.beginTransmission(BU27006MUC_ADDRESS);
  Wire.write(REG_MODE_CONTROL3);
  Wire.endTransmission();
  Wire.requestFrom(BU27006MUC_ADDRESS, 1);
  uint8_t flag = Wire.read();
  return flag;
}

// Fonction pour configurer le capteur
void configureSensor()
{
  Wire.beginTransmission(BU27006MUC_ADDRESS);   //config controle 1
  Wire.write(REG_MODE_CONTROL1);
  Wire.write(gainx32_55ms); // Gain RGB x4, temps de mesure 100ms
  Wire.endTransmission();

  Wire.beginTransmission(BU27006MUC_ADDRESS);   //config controle 2
  Wire.write(REG_MODE_CONTROL2);
  Wire.write(0x38); // desactive gain flc
  Wire.endTransmission();

  Wire.beginTransmission(BU27006MUC_ADDRESS);   //config controle 3
  Wire.write(REG_MODE_CONTROL3);
  Wire.write(0x06); // Activer la mesure RGB/IR
  Wire.endTransmission();
}                                                            //------------------------------------------------------------------------------

void updateLoadingScreen(void) {
  // Afficher l'écran de chargement
  tft.fillScreen(theme_color);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setCursor(60, 90);
  tft.println("Loading...");

  // Afficher la barre de chargement
  tft.drawRect(60, 120, 200, 20, ILI9341_WHITE);
  for (int i = 0; i <= 200; i += 20) {
    tft.fillRect(60, 120, i, 20, ILI9341_GREEN);
    delay(100); // Simule le chargement
  }
}

void drawSkittle(int x, int y, uint16_t color) {
  // Dessiner un cercle représentant le Skittle
  tft.fillCircle(x, y, 15, color); // Augmenté de 5 pixels (original + 5)
  tft.drawCircle(x, y, 15, ILI9341_WHITE); // Dessiner une bordure blanche autour
  // Ajouter le "S" blanc au centre du Skittle
  tft.setCursor(x - 5, y - 7);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.print("S");
}

void drawCup(int x, int y) {
  // Dessiner une petite coupe sous le Skittle
  int cupWidth = 25; // Augmenter la taille de la coupe
  int cupHeight = 15; // Augmenter la taille de la coupe
  tft.fillRect(x - cupWidth / 2, y, cupWidth, cupHeight, ILI9341_WHITE); //haut de la coupe
  tft.fillRect(x - (cupWidth / 2) + 2, y + cupHeight, cupWidth - 4, 5, ILI9341_WHITE); // Base de la coupe
}

void drawReturnButton(void){
  tft.fillRect(10, 10, 100, 40, ILI9341_BLUE);
  tft.drawRect(10, 10, 100, 40, ILI9341_WHITE);
  int16_t arrowX = 10 + 5;
  int16_t arrowY = 10 + (40 / 2) - 5;
  tft.fillTriangle(arrowX, arrowY, arrowX + 10, arrowY + 5, arrowX, arrowY + 10, ILI9341_WHITE);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  int16_t textX = arrowX + 15;
  int16_t textY = 10 + (40 - (8 * 2)) / 2; 
  tft.setCursor(textX, textY);
  tft.print("Retour");
}

void singleTouch(void){
  while (ts.touched()) {
    delay(150);
  }
}


void draw_MenuButton(int x, int y, int w, int h, const char* label,int x_text, int y_text, int text_size, uint16_t rectColor) {
  tft.fillRect(x,y, w, h, rectColor);
  tft.drawRect(x, y, w, h, ILI9341_WHITE);
  tft.setCursor(x_text, y + y_text);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(text_size);
  tft.println(label);
}

void drawBitmapFromProgmem(int16_t x, int16_t y, const uint16_t* bitmap, int16_t w, int16_t h) {
  for (int16_t j = 0; j < h; j++) {
    for (int16_t i = 0; i < w; i++) {
      // Lire la couleur du bitmap depuis la mémoire flash
      uint16_t color = pgm_read_word(&bitmap[j * w + i]);
      // Dessiner le pixel à l'écran
      tft.drawPixel(x + i, y + j, color);
    }
  }
}

void drawHomeIcon(int x, int y, int w, int h, uint16_t color) {
  int centerX = x + w / 2;
  int centerY = y + h / 2;

  int roofHeight = h / 3;
  int houseHeight = h / 2;
  int houseWidth = w / 2;

  // Dessiner le toit (triangle)
  tft.fillTriangle(
    centerX, centerY - roofHeight,            // pointe du toit
    centerX - houseWidth / 2, centerY,        // coin gauche
    centerX + houseWidth / 2, centerY,        // coin droit
    color
  );

  // Dessiner le corps de la maison (rectangle)
  tft.fillRect(
    centerX - houseWidth / 2,
    centerY,
    houseWidth,
    houseHeight,
    color
  );

  // (Optionnel) porte
  tft.fillRect(
    centerX - houseWidth / 6,
    centerY + houseHeight / 2,
    houseWidth / 3,
    houseHeight / 2,
    ILI9341_BLACK // petite porte noire
  );
}

void write_eeprom(uint16_t data, uint8_t mem_slot_1, uint8_t mem_slot_2){
  highByte = (data >> 8) & 0xFF; // Octet de poids fort
  lowByte = data & 0xFF;        
  EEPROM.write(mem_slot_1, highByte);
  EEPROM.write(mem_slot_2, lowByte);
  EEPROM.commit();
}

uint16_t read_eeprom(uint8_t mem_slot_1,uint8_t mem_slot_2){
  highByte = EEPROM.read(mem_slot_1);
  lowByte = EEPROM.read(mem_slot_2);
  return (highByte << 8) | lowByte;
}

void IRAM_ATTR chronometre (void){ 
  Seconde++;
  interrupt_flag = 1;
  if (Seconde > 59) {
    Seconde = 0;
    Minute++;
  }
  
}

void IRAM_ATTR capteur_couleur_go (void){
  interrupt_flag_100ms = 1;
}

void Read_Skittle_color(uint8_t nombre_ecahantillon){
  uint8_t compteur_echantillon = 0;
  uint8_t trash_data = 0;
  while(compteur_echantillon != nombre_ecahantillon){
          data_flag = reset_mesurement_flag();              //lecture de l update de la data couleur    
          if(data_flag == 0x86){
                              //si data updated lit les data RGB
            red_data = readColorData(REG_RED_DATA_MSB, REG_RED_DATA_LSB);             //lecture rouge
            green_data = readColorData(REG_GREEN_DATA_MSB, REG_GREEN_DATA_LSB);       //lecture vert
            blue_data = readColorData(REG_BLUE_DATA_MSB, REG_BLUE_DATA_LSB);          //lecture bleue

            trash_data ++;

            if(trash_data >= 8){ 
              total_rouge = total_rouge - echantillon_rouge[read_index];
              total_bleu = total_bleu - echantillon_bleu[read_index];
              total_vert = total_vert - echantillon_vert[read_index];
          
              echantillon_rouge[read_index] = red_data;
              echantillon_bleu[read_index] = blue_data;
              echantillon_vert[read_index] = green_data;

              total_rouge = total_rouge + echantillon_rouge[read_index];
              total_bleu = total_bleu + echantillon_bleu[read_index];
              total_vert = total_vert + echantillon_vert[read_index];
  

              read_index++;

              if (read_index >= nombre_ecahantillon) {
                read_index = 0;
              }
              red_data = total_rouge / nombre_ecahantillon;
              blue_data = total_bleu / nombre_ecahantillon;
              green_data = total_vert / nombre_ecahantillon;  
              compteur_echantillon ++;
            }
          }   
  }
  compteur_echantillon = 0;
  trash_data = 0;
}


void setup() {
  My_timer = timerBegin(0, 80, true);       //interruption 1 seconde
  timerAttachInterrupt(My_timer, &chronometre, true);
  timerAlarmWrite(My_timer, 1000000, true);
  timerAlarmEnable(My_timer);

  My_timer_100ms = timerBegin(1, 80, true);       //interruption 100ms 
  timerAttachInterrupt(My_timer_100ms, &capteur_couleur_go, true);
  timerAlarmWrite(My_timer_100ms, 100000, true);
  timerAlarmEnable(My_timer_100ms);

  Serial.begin(9600);
  tft.begin();
  EEPROM.begin(1024);
  
  tft.setRotation(3); // Ajuster la rotation de l'écran pour le graphique

  // Initialiser l'écran tactile
  ts.begin();
  ts.setRotation(1); // Ajuster la rotation de l'écran pour le touch
  Serial.println("ouverture du terminal");
  Serial.println("");
  theme_color = read_eeprom(0,1);                                     //espace mem de 0 a 1 -----------------------------------lecture de memoire---------------------//
  
  for(int i=0;i<3;i++){
    skittle_rouge_mesure_calibration[i] = read_eeprom((2*i)+2,(2*i)+3);            //espace mem de 2 a 7
    skittle_orange_mesure_calibration[i] = read_eeprom((2*i)+8,(2*i)+9);         //espace mem de 8 a 13
    skittle_vert_mesure_calibration[i] = read_eeprom((2*i)+14,(2*i)+15);           //espace mem de 14 a 19
    skittle_mauve_mesure_calibration[i] = read_eeprom((2*i)+20,(2*i)+21);          //espace mem de 20 a 25
    skittle_jaune_mesure_calibration[i] = read_eeprom((2*i)+26,(2*i)+27);          //espace mem de 26 a 31 -----------------------------------------------------------------------//
    aucun_skittle_mesure_calibration[i] = read_eeprom((2*i)+32,(2*i)+33);         //espace memoire 31 a 37
  }
  // Effacer l'écran et définir la couleur de fond
  tft.fillScreen(theme_color);

  Wire.begin();
  configureSensor();
  servo_redirection.setPeriodHertz(50);    // standard 50 hz servo
	servo_360.setPeriodHertz(50);    // standard 50 hz servo
	servo_redirection.attach(servoPin, 1000, 2500); // attaches the servo on pin 18 to the servo object
	servo_360.attach(servoPin2, 1000, 2000);
  pinMode(LED,OUTPUT);
  Serial.println(xPortGetCoreID());
  servo_redirection.write(position_rouge);
}

void loop() {
  switch(appState){
    case LOADING:                                                             //loading state
      updateLoadingScreen();                                                  //loading screen
      tft.fillScreen(theme_color);                                            //couleur de fond selon le theme
      drawBitmapFromProgmem(40, 70, start_button, 100, 100);                  //dessin du bouton de start
      draw_MenuButton(0,0,320,40,"Main menu",10,20,2,0xC618);
      drawBitmapFromProgmem(170, 70, setting_gear, 100, 100);                 //dessin du bouton de setting
      tft.setCursor(180, 180);
      tft.setTextColor(ILI9341_WHITE);
      tft.setTextSize(2);
      tft.println("Setting");
      tft.setCursor(63, 180);
      tft.setTextColor(ILI9341_WHITE);
      tft.setTextSize(2);
      tft.println("Start");
      appState = MENU;
      break;
    
    case MENU:                                                              //menu state
      servo_360.write(90);
      if (ts.touched()) {
        TS_Point p = ts.getPoint();
      // Conversion des coordonnées selon l'orientation de l'écran
        p.x = map(p.x, 240, 3800, 0, 320);
        p.y = map(p.y, 240, 3800, 0, 240);
     //si boutton start appuyer   
        if (abs(p.x < 140) && (abs(p.x > 40)) && (abs(p.y < 170)) && (abs(p.y > 70))) {     //p.x < x+w  px > x   p.y < y+h  py > y 
          tft.fillScreen(theme_color);
          drawReturnButton();

  // Mettre à jour les coordonnées des Skittles pour les centrer
          int spacing = (320 - (5 * 30)) / 6;
          for (int i = 0; i < 5; i++) {
            skittleX[i] = spacing + (i * (30 + spacing)) + 30 / 2;
          }
          compteur[0] = compteur_rouge;
          compteur[1] = compteur_orange;
          compteur[2] = compteur_vert;
          compteur[3] = compteur_mauve;
          compteur[4] = compteur_jaune;
          for (int i = 0; i < 5; i++) {                 // Dessiner les cups + compteur skittle
            drawCup(skittleX[i], skittleY + 30);      
            tft.setCursor(skittleX[i] -5,skittleY + 60);
            tft.print(compteur[i]);
          }

          for (int i = 0; i < 5; i++) {                               //dessine skittle + compteur total
            drawSkittle(skittleX[i], skittleY, skittleColors[i]);
          }
          tft.setCursor(120,skittleY + 90);
          tft.print("total:");

          tft.print(compteur_total);
          Seconde = 0;                        //reset chronometre
          Minute = 0;                         
          timerWrite(My_timer, 0);
          timerWrite(My_timer_100ms, 0);

          tft.fillRect(180,30,100,30,theme_color);  //affichage chronometre
          tft.setTextSize(3);
          tft.setCursor(180, 30);
          tft.print(Minute < 10 ? "0" : "");
          tft.print(Minute);
          tft.print(":");
          tft.print(Seconde < 10 ? "0" : "");
          tft.print(Seconde);
          digitalWrite(LED,HIGH);
           while(digitalRead(SWITCH) == 1){
	
		        servo_360.write(70);
	        }
	        servo_360.write(90);
          appState = START;                     //switch appstate
        }
      //si bouton config appuyer
        if (abs(p.x < 270) && (abs(p.x > 170)) && (abs(p.y < 170)) && (abs(p.y > 70))) {
          tft.fillScreen(theme_color);
          drawReturnButton();
          draw_MenuButton(50,60,220,50,"Theme",110,10,3,0x7BEF);
          draw_MenuButton(50,115,220,50,"Calibration",60,10,3,0x7BEF);
          draw_MenuButton(50,170,220,50,"Controle",90,10,3,0x7BEF);
          appState = CONFIG;
        }
        singleTouch();
      }
      break;

    case START:                                                         //start state
      if((interrupt_flag == 1) && (flag_fini == 0)){
        tft.fillRect(180,30,100,30,theme_color);
        tft.setTextSize(3);
        tft.setCursor(180, 30);
        tft.print(Minute < 10 ? "0" : "");
        tft.print(Minute);
        tft.print(":");
        tft.print(Seconde < 10 ? "0" : "");
        tft.print(Seconde);
        interrupt_flag = 0;
      }                                                      
    
        

        if(((digitalRead(SWITCH) == 0) && (flag_skittle_position == 0)) && flag_fini == 0){ // skittle vis a vis du capteur de couleur
          compteur_skittle_position++;
          index_liste_attente_detection_couleur ++;
          
          if(index_liste_attente_detection_couleur >= 5){
            index_liste_attente_detection_couleur = 0;
          }

          servo_360.write(90); // arrete le moteur

          Read_Skittle_color(5);

          servo_360.write(90);
          score_aucun = pow((red_data - aucun_skittle_mesure_calibration[0]),2) + pow((green_data - aucun_skittle_mesure_calibration[1]),2) + pow((blue_data - aucun_skittle_mesure_calibration[2]),2);
          //printf("le score de aucun est %d\r\n",score_aucun);

          score_rouge = pow((red_data - skittle_rouge_mesure_calibration[0]),2) + pow((green_data - skittle_rouge_mesure_calibration[1]),2) + pow((blue_data - skittle_rouge_mesure_calibration[2]),2);
          //printf("le score de rouge est %d\r\n",score_rouge);

          score_orange = pow((red_data - skittle_orange_mesure_calibration[0]),2) + pow((green_data - skittle_orange_mesure_calibration[1]),2) + pow((blue_data - skittle_orange_mesure_calibration[2]),2);
          //printf("le score de orange est %d\r\n",score_orange);

          score_vert = pow((red_data - skittle_vert_mesure_calibration[0]),2) + pow((green_data - skittle_vert_mesure_calibration[1]),2) + pow((blue_data - skittle_vert_mesure_calibration[2]),2);
          //printf("le score de vert est %d\r\n",score_vert);
 
          score_mauve = pow((red_data - skittle_mauve_mesure_calibration[0]),2) + pow((green_data - skittle_mauve_mesure_calibration[1]),2) + pow((blue_data - skittle_mauve_mesure_calibration[2]),2);
          //printf("le score de mauve est %d\r\n",score_mauve);

          score_jaune = pow((red_data - skittle_jaune_mesure_calibration[0]),2) + pow((green_data - skittle_jaune_mesure_calibration[1]),2) + pow((blue_data - skittle_jaune_mesure_calibration[2]),2);
          //printf("le score de jaune est %d\r\n",score_jaune);
          
          

          if((score_aucun < score_rouge) && (score_aucun < score_orange) && (score_aucun < score_vert) && (score_aucun < score_mauve) && (score_aucun < score_jaune)){
             couleur_detecter[index_liste_attente_detection_couleur] = 'A';
             compteur_aucun++;
             if(compteur_aucun >= 8){
              flag_fini = 1;
              digitalWrite(LED,LOW);
              servo_360.write(90);
              compteur_skittle_position = 0;
              compteur_aucun = 0;
              compteur_rouge = 0;
              compteur_orange = 0;
              compteur_vert = 0;
              compteur_mauve = 0;
              compteur_jaune = 0;
              compteur_total = 0;
              flag_skittle_position = 1;
              

             }
          }
          else if((score_rouge < score_jaune) && (score_rouge < score_mauve) && (score_rouge < score_orange) && (score_rouge < score_vert) && (score_rouge < score_aucun)){
            couleur_detecter[index_liste_attente_detection_couleur] = 'R';

          }else if((score_orange < score_jaune) && (score_orange < score_mauve) && (score_orange < score_rouge) && (score_orange < score_vert) && (score_orange < score_aucun)){
            couleur_detecter[index_liste_attente_detection_couleur] = 'O';

          }else if((score_vert < score_jaune) && (score_vert < score_mauve) && (score_vert < score_rouge) && (score_vert < score_orange)  && (score_vert < score_aucun)){
            couleur_detecter[index_liste_attente_detection_couleur] = 'V';

          }else if((score_mauve < score_jaune) && (score_mauve < score_vert) && (score_mauve < score_rouge) && (score_mauve < score_orange)  && (score_mauve < score_aucun)){
            couleur_detecter[index_liste_attente_detection_couleur] = 'M';

          }else if((score_jaune < score_mauve) && (score_jaune < score_vert) && (score_jaune < score_rouge) && (score_jaune < score_orange) && (score_jaune < score_aucun)){ 
            couleur_detecter[index_liste_attente_detection_couleur] = 'J';
          }

          data_flag = reset_mesurement_flag();
          flag_skittle_position = 1;
          
          }
          else if((digitalRead(SWITCH) == 1 && flag_skittle_position == 1 && flag_fini == 0)){
            servo_360.write(75);
            printf("erreur\r\n");
          }
        else{
          if((flag_skittle_position == 1) && (flag_fini == 0)){
            while(digitalRead(SWITCH) == 0){
              if (ts.touched()) {

                TS_Point p = ts.getPoint();
        
                // Conversion des coordonnées selon l'orientation de l'écran
                p.x = map(p.x, 240, 3800, 0, 320);
                p.y = map(p.y, 240, 3800, 0, 240);
                
                if (abs(p.x - 10) < 100 && abs(p.y - 10) < 40){           //appui sur bouton retour
                  tft.fillScreen(theme_color);
                  drawBitmapFromProgmem(40, 70, start_button, 100, 100);
                  draw_MenuButton(0,0,320,40,"Main menu",10,20,2,0xC618);
                  drawBitmapFromProgmem(170, 70, setting_gear, 100, 100);
                  tft.setCursor(180, 180);
                  tft.setTextColor(ILI9341_WHITE);
                  tft.setTextSize(2);
                  tft.println("Setting");
                  tft.setCursor(63, 180);
                  tft.setTextColor(ILI9341_WHITE);
                  tft.setTextSize(2);
                  tft.println("Start");
                  digitalWrite(LED,LOW);
                  compteur_skittle_position = 0;
                  compteur_aucun = 0;
                  compteur_rouge = 0;
                  compteur_orange = 0;
                  compteur_vert = 0;
                  compteur_mauve = 0;
                  compteur_jaune = 0;
                  compteur_total = 0;
                  flag_skittle_position = 1;
                  flag_fini = 0;
                  servo_360.write(90);
                  appState = MENU;
                  singleTouch();
                }     
              }
              index_a_lire = (index_liste_attente_detection_couleur + 4) % 5;

              if(couleur_detecter[index_a_lire] == 'R'){
                servo_redirection.write(position_rouge);
                compteur_rouge++;
                compteur_total++;
                compteur_aucun = 0;
                tft.setTextSize(2);
                tft.fillRect(skittleX[0] - 20,skittleY + 60,60,30,theme_color);                    
                tft.setCursor(skittleX[0] -5,skittleY + 60);
                tft.print(compteur_rouge);

                tft.fillRect(120,skittleY + 90,320,30,theme_color);
                tft.setCursor(120,skittleY + 90);
                tft.print("total:");
                tft.print(compteur_total);
                couleur_detecter[index_a_lire] = 'A';
              }else if (couleur_detecter[index_a_lire] == 'O'){
                servo_redirection.write(position_orange);
                compteur_orange++;
                compteur_total++;
                compteur_aucun = 0;
                tft.setTextSize(2);
                tft.fillRect(skittleX[1] - 20,skittleY + 60,60,30,theme_color);                    
                tft.setCursor(skittleX[1] -5,skittleY + 60);
                tft.print(compteur_orange);

                tft.fillRect(120,skittleY + 90,320,30,theme_color);
                tft.setCursor(120,skittleY + 90);
                tft.print("total:");
                tft.print(compteur_total);
                couleur_detecter[index_a_lire] = 'A';
              }else if (couleur_detecter[index_a_lire] == 'V'){
                servo_redirection.write(position_vert);
                compteur_vert++;
                compteur_total++;
                compteur_aucun = 0;
                tft.setTextSize(2);
                tft.fillRect(skittleX[2] - 20,skittleY + 60,60,30,theme_color);                    
                tft.setCursor(skittleX[2] -5,skittleY + 60);
                tft.print(compteur_vert);

                tft.fillRect(120,skittleY + 90,320,30,theme_color);
                tft.setCursor(120,skittleY + 90);
                tft.print("total:");
                tft.print(compteur_total);
                couleur_detecter[index_a_lire] = 'A';
              }else if (couleur_detecter[index_a_lire] == 'M'){
                servo_redirection.write(position_mauve);
                compteur_mauve++;
                compteur_total++;
                compteur_aucun = 0;
                tft.setTextSize(2);
                tft.fillRect(skittleX[3] - 20,skittleY + 60,60,30,theme_color);                    
                tft.setCursor(skittleX[3] -5,skittleY + 60);
                tft.print(compteur_mauve);

                tft.fillRect(120,skittleY + 90,320,30,theme_color);
                tft.setCursor(120,skittleY + 90);
                tft.print("total:");
                tft.print(compteur_total);
                couleur_detecter[index_a_lire] = 'A';
              }else if (couleur_detecter[index_a_lire] == 'J'){
                servo_redirection.write(position_jaune);
                compteur_jaune++;
                compteur_total++;
                compteur_aucun = 0;
                tft.setTextSize(2);
                tft.fillRect(skittleX[4] - 20,skittleY + 60,60,30,theme_color);                    
                tft.setCursor(skittleX[4] -5,skittleY + 60);
                tft.print(compteur_jaune);

                tft.fillRect(120,skittleY + 90,320,30,theme_color);
                tft.setCursor(120,skittleY + 90);
                tft.print("total:");
                tft.print(compteur_total);
                couleur_detecter[index_a_lire] = 'A';
              }
              delay(120);
              servo_360.write(65);
            }
            flag_skittle_position = 0;
          }
        }   
      
      
      if (ts.touched()) {

        TS_Point p = ts.getPoint();

        // Conversion des coordonnées selon l'orientation de l'écran
        p.x = map(p.x, 240, 3800, 0, 320);
        p.y = map(p.y, 240, 3800, 0, 240);
        
        if (abs(p.x - 10) < 100 && abs(p.y - 10) < 40){           //appui sur bouton retour
          tft.fillScreen(theme_color);
          drawBitmapFromProgmem(40, 70, start_button, 100, 100);
          draw_MenuButton(0,0,320,40,"Main menu",10,20,2,0xC618);
          drawBitmapFromProgmem(170, 70, setting_gear, 100, 100);
          tft.setCursor(180, 180);
          tft.setTextColor(ILI9341_WHITE);
          tft.setTextSize(2);
          tft.println("Setting");
          tft.setCursor(63, 180);
          tft.setTextColor(ILI9341_WHITE);
          tft.setTextSize(2);
          tft.println("Start");
          digitalWrite(LED,LOW);
          compteur_skittle_position = 0;
          compteur_aucun = 0;
          compteur_rouge = 0;
          compteur_orange = 0;
          compteur_vert = 0;
          compteur_mauve = 0;
          compteur_jaune = 0;
          compteur_total = 0;
          flag_skittle_position = 1;
          flag_fini = 0;
          servo_360.write(90);
          appState = MENU;
          singleTouch();  
        }
     
      }
      break;

    case CONFIG:                                                                //config state
      if (ts.touched()) {

        TS_Point p = ts.getPoint();

        // Conversion des coordonnées selon l'orientation de l'écran
        p.x = map(p.x, 240, 3800, 0, 320);
        p.y = map(p.y, 240, 3800, 0, 240);
        
        
        
        if (abs(p.x - 10) < 100 && abs(p.y - 10) < 40){                               //appui sur bouton retour
          tft.fillScreen(theme_color);
          drawBitmapFromProgmem(40, 70, start_button, 100, 100);
          draw_MenuButton(0,0,320,40,"Main menu",10,20,2,0xC618);
          drawBitmapFromProgmem(170, 70, setting_gear, 100, 100);
          tft.setCursor(180, 180);
          tft.setTextColor(ILI9341_WHITE);
          tft.setTextSize(2);
          tft.println("Setting");
          tft.setCursor(63, 180);
          tft.setTextColor(ILI9341_WHITE);
          tft.setTextSize(2);
          tft.println("Start");
          appState = MENU;
        }

        if (abs(p.x < 270) && (abs(p.x > 50)) && (abs(p.y < 110)) && (abs(p.y > 60))){             //bouton theme appuyer
          tft.fillScreen(theme_color);
          drawReturnButton();
          draw_MenuButton(50,60,220,30,"rose",135,8,2,0xfc1c);
          draw_MenuButton(50,95,220,30,"vert",135,8,2,0x87B3);
          draw_MenuButton(50,130,220,30,"mauve",130,8,2,0xDC9E);
          draw_MenuButton(50,165,220,30,"noir",135,8,2,ILI9341_BLACK);
          draw_MenuButton(50,200,220,30,"jaune",130,8,2,0xFF69);
          
          appState = THEME_CONFIG;
        }

        if (abs(p.x < 270) && (abs(p.x > 50)) && (abs(p.y < 165)) && (abs(p.y > 115))){             //bouton calibration appuyer
          tft.fillScreen(theme_color);
          drawReturnButton();
          tft.setTextSize(2);
          tft.setCursor(20, 60);
          tft.print("placez un skittle rouge");
          draw_MenuButton(50,115,220,50,"OK",145,10,3,0x7BEF);
          drawSkittle(165,95,ILI9341_RED);
          digitalWrite(LED,HIGH);
          while(digitalRead(SWITCH) == 1){
	
		        servo_360.write(70);
	        }
	        servo_360.write(90);
          
          Read_Skittle_color(5);
          aucun_skittle_mesure_calibration[0] = red_data;                                                                           //calibration jaune           
          aucun_skittle_mesure_calibration[1] = green_data;
          aucun_skittle_mesure_calibration[2] = blue_data;

          for(int i=0;i<3;i++){       
            write_eeprom(aucun_skittle_mesure_calibration[i],(2*i)+32,(2*i)+33);            //espace mem de 32 a 37
          }
          Serial.println("data aucun: ");
          Serial.print("rouge :");
          Serial.println(red_data);
          Serial.print("vert :");
          Serial.println(green_data);
          Serial.print("bleu :");
          Serial.println(blue_data);
          Serial.println("");
          Serial.println("");
          Serial.println("");
          appState = CALIBRATION_CONFIG;
        }

        if(abs(p.x < 270) && (abs(p.x > 50)) && (abs(p.y < 220)) && (abs(p.y > 170))){     //bouton controle appuyer
          tft.fillScreen(theme_color);
          drawReturnButton();

          for(int i=0;i<5;i++){
            tft.fillRect(5+i*64,150,50,50,skittleColors[i]);
          }
          draw_MenuButton(80,70,150,50,"Spin",120,15,3,0x7BEF);
          draw_MenuButton(10,70,60,50,"",10,10,3,0x7BEF);
          drawHomeIcon(10, 70, 60, 50, ILI9341_WHITE);  // maison blanche au centre du bouton
          appState = CONTROLE_CONFIG;
        }

        
        singleTouch();      
      }
      break;
    
    case THEME_CONFIG:                                                //theme config state
      if (ts.touched()) {
        TS_Point p = ts.getPoint();

        // Conversion des coordonnées selon l'orientation de l'écran
        p.x = map(p.x, 240, 3800, 0, 320);
        p.y = map(p.y, 240, 3800, 0, 240);
        
        if (abs(p.x < 270) && (abs(p.x > 50)) && (abs(p.y < 90)) && (abs(p.y > 60))){        //theme rose
          theme_color = 0xfc1c;
          tft.fillScreen(theme_color);
          drawReturnButton();
          draw_MenuButton(50,60,220,30,"rose",135,8,2,0xfc1c);
          draw_MenuButton(50,95,220,30,"vert",135,8,2,0x87B3);
          draw_MenuButton(50,130,220,30,"mauve",130,8,2,0xDC9E);
          draw_MenuButton(50,165,220,30,"noir",135,8,2,ILI9341_BLACK);
          draw_MenuButton(50,200,220,30,"jaune",130,8,2,0xFF69);  
          write_eeprom(theme_color,0,1); 
        }
        
        if (abs(p.x < 270) && (abs(p.x > 50)) && (abs(p.y < 125)) && (abs(p.y > 95))){        //theme vert
          theme_color = 0x87B3;
          tft.fillScreen(theme_color);
          drawReturnButton();
          draw_MenuButton(50,60,220,30,"rose",135,8,2,0xfc1c);
          draw_MenuButton(50,95,220,30,"vert",135,8,2,0x87B3);
          draw_MenuButton(50,130,220,30,"mauve",130,8,2,0xDC9E);
          draw_MenuButton(50,165,220,30,"noir",135,8,2,ILI9341_BLACK);
          draw_MenuButton(50,200,220,30,"jaune",130,8,2,0xFF69); 
          write_eeprom(theme_color,0,1);   
        }

        if (abs(p.x < 270) && (abs(p.x > 50)) && (abs(p.y < 160)) && (abs(p.y > 130))){        //theme mauve
          theme_color = 0xDC9E;
          tft.fillScreen(theme_color);
          drawReturnButton();
          draw_MenuButton(50,60,220,30,"rose",135,8,2,0xfc1c);
          draw_MenuButton(50,95,220,30,"vert",135,8,2,0x87B3);
          draw_MenuButton(50,130,220,30,"mauve",130,8,2,0xDC9E);
          draw_MenuButton(50,165,220,30,"noir",135,8,2,ILI9341_BLACK);
          draw_MenuButton(50,200,220,30,"jaune",130,8,2,0xFF69);  
          write_eeprom(theme_color,0,1); 
        }

        if (abs(p.x < 270) && (abs(p.x > 50)) && (abs(p.y < 195)) && (abs(p.y > 165))){        //theme noir
          theme_color = ILI9341_BLACK;
          tft.fillScreen(theme_color);
          drawReturnButton();
          draw_MenuButton(50,60,220,30,"rose",135,8,2,0xfc1c);
          draw_MenuButton(50,95,220,30,"vert",135,8,2,0x87B3);
          draw_MenuButton(50,130,220,30,"mauve",130,8,2,0xDC9E);
          draw_MenuButton(50,165,220,30,"noir",135,8,2,ILI9341_BLACK);
          draw_MenuButton(50,200,220,30,"jaune",130,8,2,0xFF69);
          write_eeprom(theme_color,0,1); 
        }

        if (abs(p.x < 270) && (abs(p.x > 50)) && (abs(p.y < 230)) && (abs(p.y > 200))){        //theme jaune
          theme_color = 0xFF69;
          tft.fillScreen(theme_color);
          drawReturnButton();
          draw_MenuButton(50,60,220,30,"rose",135,8,2,0xfc1c);
          draw_MenuButton(50,95,220,30,"vert",135,8,2,0x87B3);
          draw_MenuButton(50,130,220,30,"mauve",130,8,2,0xDC9E);
          draw_MenuButton(50,165,220,30,"noir",135,8,2,ILI9341_BLACK);
          draw_MenuButton(50,200,220,30,"jaune",130,8,2,0xFF69);  
          write_eeprom(theme_color,0,1);  
        }

        if (abs(p.x - 10) < 100 && abs(p.y - 10) < 40){               //bouton retour appuyer
          tft.fillScreen(theme_color);
          drawReturnButton();
          draw_MenuButton(50,60,220,50,"Theme",110,10,3,0x7BEF);
          draw_MenuButton(50,115,220,50,"Calibration",60,10,3,0x7BEF);
          draw_MenuButton(50,170,220,50,"Controle",90,10,3,0x7BEF);
          appState = CONFIG;
        }

        singleTouch();      
      }
      break;
    case CALIBRATION_CONFIG:
      if (ts.touched()) {  
        TS_Point p = ts.getPoint();
        // Conversion des coordonnées selon l'orientation de l'écran
        p.x = map(p.x, 240, 3800, 0, 320);
        p.y = map(p.y, 240, 3800, 0, 240);
        if (abs(p.x < 110) && (abs(p.x > 10)) && (abs(p.y < 50)) && (abs(p.y > 10))){               //bouton retour appuyer
          tft.fillScreen(theme_color);
          drawReturnButton();
          draw_MenuButton(50,60,220,50,"Theme",110,10,3,0x7BEF);
          draw_MenuButton(50,115,220,50,"Calibration",60,10,3,0x7BEF);
          draw_MenuButton(50,170,220,50,"Controle",90,10,3,0x7BEF);
          calibration_compteur = 0;
          digitalWrite(LED,LOW);
          compteur_skittle_position = 0;
          while(compteur_skittle_position != 2)
          {
            if ((digitalRead(SWITCH) == 0) && (flag_skittle_position == 0))
            { // skittle vis a vis du capteur de couleur
              compteur_skittle_position++;
              servo_360.write(90);                                                 // arrete le moteur
              flag_skittle_position = 1;
            }else if (flag_skittle_position == 1){
              while (digitalRead(SWITCH) == 0)
              {
                servo_360.write(60);
              }
              flag_skittle_position = 0;
            }else{
              servo_360.write(60);
            }
          }
          compteur_skittle_position = 0;
          appState = CONFIG;
        }


        if((abs(p.x < 270) && (abs(p.x > 50)) && (abs(p.y < 165)) && (abs(p.y > 115))) && calibration_compteur == 4){        //bouton OK appuyer
          
          while (compteur_skittle_position != 2)
          {
            if ((digitalRead(SWITCH) == 0) && (flag_skittle_position == 0))
            { // skittle vis a vis du capteur de couleur
              compteur_skittle_position++;
              servo_360.write(90);                                                 // arrete le moteur
              flag_skittle_position = 1;
            }
            else if (flag_skittle_position == 1)
            {
              while (digitalRead(SWITCH) == 0)
              {
                servo_360.write(60);
              }
              flag_skittle_position = 0;
            }
          }
          
          
          if(compteur_skittle_position == 2){
            servo_360.write(90);
            Read_Skittle_color(5);
            skittle_jaune_mesure_calibration[0] = red_data;                                                                           //calibration jaune           
            skittle_jaune_mesure_calibration[1] = green_data;
            skittle_jaune_mesure_calibration[2] = blue_data;
  

            for(int i=0;i<3;i++){       
              write_eeprom(skittle_jaune_mesure_calibration[i],(2*i)+26,(2*i)+27);            //espace mem de 26 a 31
            }
            Serial.println("data jaune: ");
            Serial.print("rouge :");
            Serial.println(red_data);
            Serial.print("vert :");
            Serial.println(green_data);
            Serial.print("bleu :");
            Serial.println(blue_data);
            Serial.println("");
            calibration_compteur = 0;                                                                                           
            tft.fillScreen(theme_color);
            drawReturnButton();
            draw_MenuButton(50,115,220,50,"OK",145,10,3,0x7BEF);
            tft.setTextSize(2);
            tft.setCursor(20, 60);
            tft.print("placez un skittle rouge");
            drawSkittle(165,95,ILI9341_RED);
            servo_redirection.write(position_jaune);
            compteur_skittle_position = 0;
          }

        }else if((abs(p.x < 270) && (abs(p.x > 50)) && (abs(p.y < 165)) && (abs(p.y > 115))) && calibration_compteur == 0){        //bouton OK appuyer
          
         while (compteur_skittle_position != 2)
          {
            if ((digitalRead(SWITCH) == 0) && (flag_skittle_position == 0))
            { // skittle vis a vis du capteur de couleur
              compteur_skittle_position++;
              servo_360.write(90);                                                 // arrete le moteur
              flag_skittle_position = 1;
            }
            else if (flag_skittle_position == 1)
            {
              while (digitalRead(SWITCH) == 0)
              {
                servo_360.write(60);
              }
              flag_skittle_position = 0;
            }else{
              servo_360.write(60);
            }
          }
          if(compteur_skittle_position == 2){
            servo_360.write(90);	
            Read_Skittle_color(5);
            skittle_rouge_mesure_calibration[0] = red_data;                                                                                 //calibration rouge                 
            skittle_rouge_mesure_calibration[1] = green_data;
            skittle_rouge_mesure_calibration[2] = blue_data;
           

            for(int i=0;i<3;i++){       
              write_eeprom(skittle_rouge_mesure_calibration[i],(2*i)+2,(2*i)+3);            //espace mem de 2 a 7
            }
            Serial.println("data rouge: ");
            Serial.print("rouge :");
            Serial.println(red_data);
            Serial.print("vert :");
            Serial.println(green_data);
            Serial.print("bleu :");
            Serial.println(blue_data);
            Serial.println("");
            calibration_compteur ++;                                                                                           
            tft.fillScreen(theme_color);
            drawReturnButton();
            draw_MenuButton(50,115,220,50,"OK",145,10,3,0x7BEF);
            tft.setTextSize(2);
            tft.setCursor(15, 60);
            tft.print("placez un skittle orange");
            drawSkittle(165,95,0xFD20);
            servo_redirection.write(position_rouge);
            compteur_skittle_position = 0;
          }

        }else if((abs(p.x < 270) && (abs(p.x > 50)) && (abs(p.y < 165)) && (abs(p.y > 115))) && calibration_compteur == 1){
          
          while (compteur_skittle_position != 2)
          {
            if ((digitalRead(SWITCH) == 0) && (flag_skittle_position == 0))
            { // skittle vis a vis du capteur de couleur
              compteur_skittle_position++;
              servo_360.write(90);                                                 // arrete le moteur
              flag_skittle_position = 1;
            }
            else if (flag_skittle_position == 1)
            {
              while (digitalRead(SWITCH) == 0)
              {
                servo_360.write(60);
              }
              flag_skittle_position = 0;
            }else{
              servo_360.write(60);
            }
          }
          
          
          
          if(compteur_skittle_position == 2){
            servo_360.write(90);
            Read_Skittle_color(5);
            skittle_orange_mesure_calibration[0] = red_data;                                                                         //calibration orange          
            skittle_orange_mesure_calibration[1] = green_data;
            skittle_orange_mesure_calibration[2] = blue_data;
            

            for(int i=0;i<3;i++){       
              write_eeprom(skittle_orange_mesure_calibration[i],(2*i)+8,(2*i)+9);            //espace mem de 8 a 13
            }
            Serial.println("data orange: ");
            Serial.print("rouge :");
            Serial.println(red_data);
            Serial.print("vert :");
            Serial.println(green_data);
            Serial.print("bleu :");
            Serial.println(blue_data);
            Serial.println("");
            calibration_compteur ++;                                                                                            
            tft.fillScreen(theme_color);
            drawReturnButton();
            draw_MenuButton(50,115,220,50,"OK",145,10,3,0x7BEF);
            tft.setTextSize(2);
            tft.setCursor(25, 60);
            tft.print("placez un skittle vert");
            drawSkittle(165,95,ILI9341_GREEN);
            servo_redirection.write(position_orange);
            compteur_skittle_position = 0;
          }

        }else if((abs(p.x < 270) && (abs(p.x > 50)) && (abs(p.y < 165)) && (abs(p.y > 115))) && calibration_compteur == 2){

          while (compteur_skittle_position != 2)
          {
            if ((digitalRead(SWITCH) == 0) && (flag_skittle_position == 0))
            { // skittle vis a vis du capteur de couleur
              compteur_skittle_position++;
              servo_360.write(90);                                                 // arrete le moteur
              flag_skittle_position = 1;
            }
            else if (flag_skittle_position == 1)
            {
              while (digitalRead(SWITCH) == 0)
              {
                servo_360.write(60);
              }
              flag_skittle_position = 0;
            }else{
              servo_360.write(60);
            }
          }


          if(compteur_skittle_position == 2){
            servo_360.write(90);
            Read_Skittle_color(5);
            skittle_vert_mesure_calibration[0] = red_data;                                                                           //calibration vert           
            skittle_vert_mesure_calibration[1] = green_data;
            skittle_vert_mesure_calibration[2] = blue_data;


            for(int i=0;i<3;i++){       
              write_eeprom(skittle_vert_mesure_calibration[i],(2*i)+14,(2*i)+15);            //espace mem de 14 a 19
            }
            Serial.println("data vert: ");
            Serial.print("rouge :");
            Serial.println(red_data);
            Serial.print("vert :");
            Serial.println(green_data);
            Serial.print("bleu :");
            Serial.println(blue_data);
            Serial.println("");
            calibration_compteur ++;                                                                                             
            tft.fillScreen(theme_color);
            drawReturnButton();
            draw_MenuButton(50,115,220,50,"OK",145,10,3,0x7BEF);
            tft.setTextSize(2);
            tft.setCursor(20, 60);
            tft.print("placez un skittle mauve");
            drawSkittle(165,95,0x780F);
            servo_redirection.write(position_vert);
            compteur_skittle_position = 0;
          }


        }else if((abs(p.x < 270) && (abs(p.x > 50)) && (abs(p.y < 165)) && (abs(p.y > 115))) && calibration_compteur == 3){
          
          while (compteur_skittle_position != 2)
          {
            if ((digitalRead(SWITCH) == 0) && (flag_skittle_position == 0))
            { // skittle vis a vis du capteur de couleur
              compteur_skittle_position++;
              servo_360.write(90);                                                 // arrete le moteur
              flag_skittle_position = 1;
            }
            else if (flag_skittle_position == 1)
            {
              while (digitalRead(SWITCH) == 0)
              {
                servo_360.write(60);
              }
              flag_skittle_position = 0;
            }else{
              servo_360.write(60);
            }
          }
          
          
          if(compteur_skittle_position == 2){
            servo_360.write(90);
            Read_Skittle_color(5);
            skittle_mauve_mesure_calibration[0] = red_data;                                                                           //calibration mauve   
            skittle_mauve_mesure_calibration[1] = green_data;
            skittle_mauve_mesure_calibration[2] = blue_data;


            for(int i=0;i<3;i++){       
              write_eeprom(skittle_mauve_mesure_calibration[i],(2*i)+20,(2*i)+21);            //espace mem de 20 a 25
            }
            Serial.println("data mauve: ");
            Serial.print("rouge :");
            Serial.println(red_data);
            Serial.print("vert :");
            Serial.println(green_data);
            Serial.print("bleu :");
            Serial.println(blue_data);
            Serial.println("");
            calibration_compteur ++;                                                                                              
            tft.fillScreen(theme_color);
            drawReturnButton();
            draw_MenuButton(50,115,220,50,"OK",145,10,3,0x7BEF);
            tft.setTextSize(2);
            tft.setCursor(15,60);
            tft.print("placez un skittle jaune");
            drawSkittle(165,95,ILI9341_YELLOW);
            servo_redirection.write(position_mauve);
            compteur_skittle_position = 0;
          }
        }
        singleTouch();   
      }
    break;
    
    case CONTROLE_CONFIG:
      if (ts.touched()) {

        TS_Point p = ts.getPoint();

        // Conversion des coordonnées selon l'orientation de l'écran
        p.x = map(p.x, 240, 3800, 0, 320);
        p.y = map(p.y, 240, 3800, 0, 240);

        if (abs(p.x - 10) < 100 && abs(p.y - 10) < 40){               //bouton retour appuyer
          tft.fillScreen(theme_color);
          drawReturnButton();
          draw_MenuButton(50,60,220,50,"Theme",110,10,3,0x7BEF);
          draw_MenuButton(50,115,220,50,"Calibration",60,10,3,0x7BEF);
          draw_MenuButton(50,170,220,50,"Controle",90,10,3,0x7BEF);
          boutton_spin_state = false;
          servo_360.write(90);
          appState = CONFIG;
        }

        if(abs(p.x < 230) && (abs(p.x > 80)) && (abs(p.y < 120)) && (abs(p.y > 70))){   //bouton spin appuyer
          delay(100);           
          if(boutton_spin_state == false){
            servo_360.write(70);
            draw_MenuButton(80,70,150,50,"Stop",120,15,3,0x7BEF);
            boutton_spin_state = true;
          }else{
            servo_360.write(90);
            draw_MenuButton(80,70,150,50,"Spin",120,15,3,0x7BEF);
            boutton_spin_state = false;
          } 
        }

          if(abs(p.x < 55) && (abs(p.x > 5)) && (abs(p.y < 200)) && (abs(p.y > 150))){ //bouton rouge appuyer
            servo_redirection.write(position_rouge);
          }else if(abs(p.x < 119) && (abs(p.x > 69)) && (abs(p.y < 200)) && (abs(p.y > 150))){  //bouton orange appuyer
            servo_redirection.write(position_orange);
          }else if(abs(p.x < 183) && (abs(p.x > 133)) && (abs(p.y < 200)) && (abs(p.y > 150))){  //bouton vert appuyer
            servo_redirection.write(position_vert);
          }else if(abs(p.x < 247) && (abs(p.x > 197)) && (abs(p.y < 200)) && (abs(p.y > 150))){  //bouton mauve appuyer
            servo_redirection.write(position_mauve);
          }else if(abs(p.x < 306) && (abs(p.x > 256)) && (abs(p.y < 200)) && (abs(p.y > 150))){  //bouton jaune appuyer
            servo_redirection.write(position_jaune);
          }
          
          if(abs(p.x < 70) && (abs(p.x > 10)) && (abs(p.y < 120)) && (abs(p.y > 70))){        //bouton home appuyer
            while(digitalRead(SWITCH) == 1){
	
              servo_360.write(70);
            }
            servo_360.write(90);
          }
           
        singleTouch();      
      } 
    break;
  }  
}


