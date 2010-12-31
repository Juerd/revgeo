#include <LiquidCrystal.h>
#include <NewSoftSerial.h>
#include <TinyGPS.h>

const byte BUTTON_PIN =  4;
const float TOLERANCE = 30;  // meters

LiquidCrystal lcd(7, 8, 5, 6, 11, 12);
NewSoftSerial nss(2, 3);
TinyGPS       gps;

const byte NOT = 0, SHORT = 1, LONG = 2;

byte button() {
  static unsigned long pressed = 0;
  unsigned long elapsed;
  
  if (pressed) {
    elapsed = millis() - pressed;
    if (elapsed < 100) return NOT;
    if (digitalRead(BUTTON_PIN) == LOW) return NOT;
    pressed = 0;
    return elapsed > 500 ? LONG : SHORT;
  }
  if (digitalRead(BUTTON_PIN) != LOW) return NOT;
  pressed = millis();
  return NOT;
}

void setup() {
  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(BUTTON_PIN, HIGH);
  lcd.begin(20, 4);
  lcd.clear();
  lcd.print("init...");
  nss.begin(9600);
  //Serial.begin(9600);
  delay(2000);
}

void loop() {
  static enum {
     SELECT_ROUTE_SETUP,  SELECT_ROUTE_UPDATE, SELECT_ROUTE,
     WAIT_FOR_FIX_SETUP,  WAIT_FOR_FIX_UPDATE, WAIT_FOR_FIX,
            ROUTE_SETUP,
         WAYPOINT_SETUP,      WAYPOINT_UPDATE, WAYPOINT,
                                               DESTINATION,
             FAIL_SETUP,                       FAIL,
    PROGRAM_YESNO_SETUP, PROGRAM_YESNO_UPDATE, PROGRAM_YESNO,
          PROGRAM_SETUP,                       PROGRAM,
                                               PROGRAM_DONE
  } state = SELECT_ROUTE_SETUP;
  
  static byte          route, waypoint, tries_left;
  static byte          dotstate = 0;
  static boolean       programming = false, yesno;
  static unsigned long fix = 0;
  static float         there_lat, there_lon, distance;

  while (nss.available()) {
    char c = nss.read();
    if (gps.encode(c)) {
      unsigned long age;
      float here_lat, here_lon;
      
      if (!fix) fix = millis();
      gps.f_get_position(&here_lat, &here_lon, &age);
      distance = TinyGPS::distance_between(here_lat, here_lon, there_lat, there_lon);
    }
  }
  
  switch (state) {
    case SELECT_ROUTE_SETUP:
      route = 1;
      lcd.clear();
      lcd.print("Kies route.");
      lcd.setCursor(0, 1);
      lcd.print("kort = +1, lang = OK");
      state = SELECT_ROUTE_UPDATE;
      break;

    case SELECT_ROUTE_UPDATE:
      lcd.setCursor(0, 2);
      lcd.print("Geselecteerd: ");
      lcd.print(route, DEC);
      lcd.print("  ");
      state = SELECT_ROUTE;
      break;
    
    case SELECT_ROUTE:
      switch (button()) {
        case SHORT:
          route++;
          if (route > 10) route = 1;  // TODO: real number
          state = SELECT_ROUTE_UPDATE;
          break;
        case LONG:
          state = WAIT_FOR_FIX_SETUP;
          break;
      }
      break;
 
    case WAIT_FOR_FIX_SETUP:
      fix = 0;
      lcd.clear();
      lcd.print("Wacht op GPS-fix...");
      state = WAIT_FOR_FIX_UPDATE;
      break;
      
    case WAIT_FOR_FIX_UPDATE:
      if (dotstate >= 100) dotstate = 0;
      lcd.setCursor(16, 0);
      for (byte i = 0; i <= dotstate / 25; i++) lcd.print(".");
      lcd.print("   ");
      state = WAIT_FOR_FIX;
      break;
    
    case WAIT_FOR_FIX:
      if (button() == LONG)
        state = PROGRAM_YESNO_SETUP;
      else if (fix && millis() - fix > 2000)
        state = programming ? PROGRAM_SETUP : ROUTE_SETUP;
      else if (++dotstate % 100)
        state = WAIT_FOR_FIX_UPDATE;
      break;
      
    case ROUTE_SETUP:
      waypoint = 1;
      state = WAYPOINT_SETUP;
      break;
      
    case WAYPOINT_SETUP:
      tries_left =  6;
      there_lat  = 52.073228; // TODO: read from memory
      there_lon  =  4.328249;
      distance   = -1;
      state = WAYPOINT_UPDATE;
      break;
    
    case WAYPOINT_UPDATE:
      lcd.clear();
      if (distance >= 0) {
        lcd.print("Afstand = ");
        lcd.print(distance, 0);
        lcd.print("m    ");
      }
      lcd.setCursor(0, 1);
      lcd.print("Je mag nog ");
      lcd.print(tries_left, DEC);
      lcd.print(" keer  ");
      lcd.setCursor(0, 2);
      lcd.print("drukken.");
      state = WAYPOINT;
      break;
      
    case WAYPOINT:
      if (distance < 0 || !button()) break;
      state = distance <= TOLERANCE ? DESTINATION
            : --tries_left          ? WAYPOINT_UPDATE
            :                         FAIL_SETUP;
      break;
    
    case DESTINATION:
      lcd.clear();
      lcd.print("WOOHOO!! YAY!!");
      // TODO: increment current waypoint index, state = ROUTE_SETUP
      // TODO: open box :)
      for (;;) delay(1000);  // ad infinitum
      break;
   
    case FAIL_SETUP:
      lcd.clear();
      lcd.print("GAME OVER :(");
      lcd.setCursor(0, 1);
      lcd.print("Afstand tot einddoel");
      // TODO: einddoel laden...
      state = FAIL;
      break;
    
    case FAIL:
      lcd.setCursor(0, 2);
      lcd.print(distance, 0);
      lcd.print(" m     ");
      break;
    
    case PROGRAM_YESNO_SETUP:
      yesno = false;
      lcd.clear();
      lcd.print("Route ");
      lcd.print(route, DEC);
      lcd.print(" programmeren?");
      lcd.setCursor(0, 2);
      lcd.print("kort = wissel");
      lcd.setCursor(0, 3);
      lcd.print("lang = bevestig");
      state = PROGRAM_YESNO_UPDATE;
      break;

    case PROGRAM_YESNO_UPDATE:
      lcd.setCursor(0, 1);
      lcd.print("Geselecteerd: ");
      lcd.print(yesno ? "JA " : "NEE");
      state = PROGRAM_YESNO;
      break;
     
    case PROGRAM_YESNO:
      switch (button()) {
        case SHORT:
          yesno = !yesno;
          state = PROGRAM_YESNO_UPDATE;
          break;
        case LONG:
          programming = yesno;
          state = WAIT_FOR_FIX_SETUP;
          break;
      }
      break;

    case PROGRAM_SETUP:
      lcd.clear();
      lcd.print("ERROR: NOT IMPL.");
      delay(4000);
      programming = false;
      state = WAIT_FOR_FIX_SETUP;
      break;
      
  }

  delay(10);
}


