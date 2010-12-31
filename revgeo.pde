
#include <EEPROM.h>
#include <LiquidCrystal.h>
#include <NewSoftSerial.h>
#include <TinyGPS.h>

const byte BUTTON_PIN   =  4;
const byte MAX_WAYPOINT = 10;
const byte MAX_ROUTE    = 10;
const byte TRIES[]      = { 15, 25, 35 };

LiquidCrystal lcd(7, 8, 5, 6, 11, 12);
NewSoftSerial nss(2, 3);
TinyGPS       gps;

// http://www.arduino.cc/playground/Code/EEPROMWriteAnything
template <class T> int EEPROM_writeAnything(int ee, const T& value) {
    const byte* p = (const byte*)(const void*)&value;
    int i;
    for (i = 0; i < sizeof(value); i++) EEPROM.write(ee++, *p++);
    return i;
}
template <class T> int EEPROM_readAnything(int ee, T& value) {
    byte* p = (byte*)(void*)&value;
    int i;
    for (i = 0; i < sizeof(value); i++) *p++ = EEPROM.read(ee++);
    return i;
}

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

int address_for(byte route, byte waypoint) {
  return (route - 1) * 100 + (waypoint - 1) * 10;
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

struct Waypoint {
  // Don't alter struct; stored in EEPROM
  float lat;
  float lon;
  byte tolerance;
  byte flags;
};

void loop() {
  static enum {
     SELECT_ROUTE_SETUP,  SELECT_ROUTE_UPDATE, SELECT_ROUTE,
     SELECT_TRIES_SETUP,  SELECT_TRIES_UPDATE, SELECT_TRIES,
     WAIT_FOR_FIX_SETUP,  WAIT_FOR_FIX_UPDATE, WAIT_FOR_FIX,
            ROUTE_SETUP,                                      ROUTE_DONE,
         WAYPOINT_SETUP,      WAYPOINT_UPDATE, WAYPOINT,      WAYPOINT_DONE,
             FAIL_SETUP,                       FAIL,
    PROGRAM_YESNO_SETUP, PROGRAM_YESNO_UPDATE, PROGRAM_YESNO,
          PROGRAM_SETUP,       PROGRAM_UPDATE, PROGRAM,       PROGRAM_DONE
  } state = SELECT_ROUTE_SETUP;
  
  static byte          route, waypoint, tries_left, triesidx;
  static byte          dotstate = 0;
  static boolean       programming = false, yesno;
  static unsigned long fix = 0;
  static float         distance;
  static Waypoint      there;
  static float         here_lat, here_lon;

  while (nss.available()) {
    char c = nss.read();
    if (gps.encode(c)) {
      unsigned long age;
      if (!fix) fix = millis();
      gps.f_get_position(&here_lat, &here_lon, &age);
      distance = TinyGPS::distance_between(here_lat, here_lon, there.lat, there.lon);
    }
  }
  
  switch (state) {
    case SELECT_ROUTE_SETUP: {
      route = 1;
      lcd.clear();
      lcd.print("Kies route.");
      lcd.setCursor(0, 1);
      lcd.print("kort = +1, lang = OK");
      state = SELECT_ROUTE_UPDATE;
      break;  
    }
    
    case SELECT_ROUTE_UPDATE: {
      lcd.setCursor(0, 2);
      lcd.print("Geselecteerd: ");
      lcd.print(route, DEC);
      lcd.print("  ");
      state = SELECT_ROUTE;
      break;
    }
    
    case SELECT_ROUTE: {
      switch (button()) {
        case SHORT:
          route++;
          if (route > MAX_ROUTE) route = 1;
          state = SELECT_ROUTE_UPDATE;
          break;
        case LONG:
          state = SELECT_TRIES_SETUP;
          break;
      }
      break;
    }
 
    case SELECT_TRIES_SETUP: {
      triesidx = 0;
      lcd.clear();
      lcd.print("Kies moeilijkheid.");
      lcd.setCursor(0, 1);
      lcd.print("lang = bevestig");
      lcd.setCursor(0, 3);
      lcd.print("            pogingen");
      state = SELECT_TRIES_UPDATE;
      break;  
    }
    
    case SELECT_TRIES_UPDATE: {
      lcd.setCursor(0, 2);
      lcd.print("Geselecteerd: ");
      lcd.print(TRIES[triesidx], DEC);
      lcd.print("  ");
      state = SELECT_TRIES;
      break;
    }
    
    case SELECT_TRIES: {
      switch (button()) {
        case SHORT:
          triesidx++;
          if (triesidx >= sizeof(TRIES)) triesidx = 0;
          state = SELECT_TRIES_UPDATE;
          break;
        case LONG:
          state = WAIT_FOR_FIX_SETUP;
          break;
      }
      break;
    }
 
    case WAIT_FOR_FIX_SETUP: {
      fix = 0;
      lcd.clear();
      lcd.print("Wacht op GPS-fix...");
      state = WAIT_FOR_FIX_UPDATE;
      break;
    }
      
    case WAIT_FOR_FIX_UPDATE: {
      if (dotstate >= 100) dotstate = 0;
      lcd.setCursor(16, 0);
      for (byte i = 0; i <= dotstate / 25; i++) lcd.print(".");
      lcd.print("   ");
      state = WAIT_FOR_FIX;
      break;
    }
    
    case WAIT_FOR_FIX: {
      if (button() == LONG)
        state = PROGRAM_YESNO_SETUP;
      else if (fix && millis() - fix > 2000)
        state = programming ? PROGRAM_SETUP : ROUTE_SETUP;
      else if (++dotstate % 100)
        state = WAIT_FOR_FIX_UPDATE;
      break;
    }
      
    case ROUTE_SETUP: {
      waypoint = 1;
      state = WAYPOINT_SETUP;
      break;
    }
      
    case WAYPOINT_SETUP: {
      lcd.clear();
      lcd.print("Onderweg naar #");
      lcd.print(waypoint, DEC);
      EEPROM_readAnything(address_for(route, waypoint), there);
      tries_left = TRIES[triesidx];
      distance   = -1;
      state = there.lat ? WAYPOINT_UPDATE : ROUTE_DONE;
      break;
    }
    
    case WAYPOINT_UPDATE: {
      if (distance >= 0) {
        lcd.setCursor(0, 1);
        lcd.print("Afstand = ");
        lcd.print(distance, 0);
        lcd.print("m    ");
      }
      lcd.setCursor(0, 2);
      lcd.print("Je mag nog ");
      lcd.print(tries_left, DEC);
      lcd.print(" keer  ");
      lcd.setCursor(0, 3);
      lcd.print("drukken.");
      state = WAYPOINT;
      break;
    }
      
    case WAYPOINT: {
      if (distance < 0 || !button()) break;
      state = distance <= there.tolerance ? WAYPOINT_DONE
            : --tries_left                ? WAYPOINT_UPDATE
            :                               FAIL_SETUP;
      break;
    }
    
    case WAYPOINT_DONE: {
      lcd.clear();
      lcd.print("WOOHOO!! YAY!!");
      delay(4000);
      if (waypoint == MAX_WAYPOINT) {
        state = ROUTE_DONE;
        break;
      }      
      waypoint++;      
      state = WAYPOINT_SETUP;
      break;
    }
    
    case ROUTE_DONE: {
      lcd.clear();
      lcd.print("Route klaar!");
      for (;;) delay(1000); // ad infinitum
      break;
    }
      
    case FAIL_SETUP: {
      lcd.clear();
      lcd.print("GAME OVER :(");
      while (there.lat && waypoint <= MAX_WAYPOINT)
        EEPROM_readAnything(address_for(route, ++waypoint), there);
      EEPROM_readAnything(address_for(route, --waypoint), there);
      lcd.setCursor(0, 1);
      lcd.print("Afstand tot einddoel");
      state = FAIL;
      break;
    }
    
    case FAIL: {
      lcd.setCursor(0, 2);
      lcd.print(distance, 0);
      lcd.print(" m     ");
      break;
    }
    
    case PROGRAM_YESNO_SETUP: {
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
    }

    case PROGRAM_YESNO_UPDATE: {
      lcd.setCursor(0, 1);
      lcd.print("Geselecteerd: ");
      lcd.print(yesno ? "JA " : "NEE");
      state = PROGRAM_YESNO;
      break;
    }
     
    case PROGRAM_YESNO: {
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
    }

    case PROGRAM_SETUP: {
      waypoint = 1;
      state = PROGRAM_UPDATE;
      break;
    }
  
    case PROGRAM_UPDATE: {
      lcd.clear();
      lcd.print("Ga naar waypoint ");
      lcd.print(waypoint, DEC);
      lcd.setCursor(0, 1);
      if (waypoint > 1) {
        lcd.print("Afstand tot wp ");
        lcd.print(waypoint - 1, DEC);
      }
      state = PROGRAM;
      break;
    }
      
    case PROGRAM: {
      switch (button()) {
        case NOT:
          if (waypoint < 2) break;
          lcd.setCursor(0, 2);
          lcd.print(distance, 0);
          lcd.print("m      ");
          break;
        case LONG:
          state = PROGRAM_DONE;
          break;
        case SHORT:
          state = PROGRAM_UPDATE;
          there.lat = here_lat;
          there.lon = here_lon;
          there.tolerance = 30;  // FIXME
          there.flags = 0;  // FIXME
          EEPROM_writeAnything(address_for(route, waypoint), there);
          break;
      }
      if (++waypoint > MAX_WAYPOINT) {
        state = PROGRAM_DONE;
        break;
      }
    }
      
    case PROGRAM_DONE: {
      if (waypoint <= MAX_WAYPOINT) {
        there.lat = 0;
        there.lon = 0;
        there.tolerance = 0;
        there.flags = 0;
        EEPROM_writeAnything(address_for(route, waypoint), there);
      }
      lcd.clear();
      lcd.print("That's all, folks!");
      delay(5000);
      programming = false;
      state = SELECT_ROUTE_SETUP;
      break;
    }
  }
  delay(10);
}
