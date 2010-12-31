
#include <EEPROM.h>
#include <PWMServo.h>
#include <Nokia5110.h>
#include <NewSoftSerial.h>
#include <TinyGPS.h>

const byte BUTTON_PIN    =  4;
const byte BACKLIGHT_PIN = 13;
const byte SERVO_PIN     = 10;  // 9 or 10 only
const byte TOR_PIN       =  6;

const byte MAX_WAYPOINT  = 10;
const byte MAX_ROUTE     = 10;
const byte TRIES[]       = { 15, 25, 35 };

Nokia5110     lcd(8, 9, 7, 11, 12);
NewSoftSerial nss(2, 3);
TinyGPS       gps;
PWMServo      servo;

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

const byte OFF = 0, CHECK = 1, ON = 2;

void backlight(unsigned int param) {
  static unsigned long endtime = 0;
  if (param == CHECK) {
    if (endtime && millis() >= endtime) {
      backlight(OFF);
      endtime = 0;
    }
    return;
  }
  digitalWrite(BACKLIGHT_PIN, param >= ON ? HIGH : LOW);
  endtime = param > ON ? millis() + param : 0;
}

void set_servo(int a) {
  static byte angle;
  static unsigned long endtime = 0;

  if (a >= 0) {
    angle = a;
    digitalWrite(TOR_PIN, HIGH);
    servo.write(angle);
    endtime = millis() + 1000;
  }
  if (endtime && millis() >= endtime) {
    digitalWrite(TOR_PIN, LOW);
    endtime = 0;
  }
}


int address_for(byte route, byte waypoint) {
  return (route - 1) * 100 + (waypoint - 1) * 10;
}

void setup() {
  
  pinMode(BACKLIGHT_PIN, OUTPUT);
  pinMode(TOR_PIN, OUTPUT);
  
  lcd.clear();
  lcd.setInverse();
  delay(500);
  lcd.print("\n\n    REV\n       GEO");
  delay(1000);
  lcd.noInverse();
  delay(500);
  
//  Serial.begin(9600);
  nss.begin(9600);
  servo.attach(SERVO_PIN);
  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(BUTTON_PIN, HIGH);
  lcd.clear();
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
                                               CRASHED,
     SELECT_ROUTE_SETUP,  SELECT_ROUTE_UPDATE, SELECT_ROUTE,
     SELECT_TRIES_SETUP,  SELECT_TRIES_UPDATE, SELECT_TRIES,
            CLOSE_SETUP,                       CLOSE,
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

  backlight(CHECK);
  set_servo(-1);

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
    case CRASHED: break;
    
    case SELECT_ROUTE_SETUP: {
      backlight(ON);
      set_servo(5);
      route = 1;
      lcd.clear();
      lcd.print("Kies route.\nkort: +1\nlang: OK\n\nGeselecteerd:\n");
      state = SELECT_ROUTE_UPDATE;
      break;  
    }
    
    case SELECT_ROUTE_UPDATE: {
      lcd.print("\rRoute #");
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
      lcd.print("Kies niveau.\nkort: volgende\nlang: bevestig\n\nGeselecteerd:\n");
      state = SELECT_TRIES_UPDATE;
      break;  
    }
    
    case SELECT_TRIES_UPDATE: {
      lcd.print("\r");
      lcd.print(TRIES[triesidx], DEC);
      lcd.print(" pogingen  ");
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
          state = CLOSE_SETUP;
          break;
      }
      break;
    }
    
    case CLOSE_SETUP: {
      lcd.clear();
      lcd.print("Sluit deksel.\nDruk knopje om\nhet doosje te\nsluiten.");
      state = CLOSE;
      break;
    }
    
    case CLOSE: {
      if (!button()) break;
      set_servo(90);
      state = WAIT_FOR_FIX_SETUP;
      break;
    }
 
    case WAIT_FOR_FIX_SETUP: {
      fix = 0;
      backlight(5000);
      lcd.clear();
      lcd.print("Wacht op\nGPS-fix...");
      state = WAIT_FOR_FIX_UPDATE;
      break;
    }
      
    case WAIT_FOR_FIX_UPDATE: {
      if (dotstate >= 100) dotstate = 0;
      lcd.setCursor(7, 1);
      for (byte i = 0; i <= dotstate / 25; i++) lcd.print(".");
      lcd.print("    ");
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
      lcd.print("Onderweg naar\nwaypoint ");
      lcd.print(waypoint, DEC);
      EEPROM_readAnything(address_for(route, waypoint), there);
      tries_left = TRIES[triesidx];
      distance   = -1;
      state = there.lat ? WAYPOINT_UPDATE : ROUTE_DONE;
      break;
    }
    
    case WAYPOINT_UPDATE: {
      backlight(15000);
      if (distance >= 0) {
        lcd.setCursor(0, 2);
        lcd.print("Afstand:\n");
        lcd.print(distance, 0);
        lcd.print("m    ");
      }
      lcd.setCursor(0, 4);
      lcd.print("Je mag nog ");
      lcd.print(tries_left, DEC);
      lcd.print(" \nkeer drukken.");
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
      backlight(ON);
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
      backlight(10000);
      set_servo(5);
      lcd.print("Route klaar!");
      state = CRASHED;
      break;
    }
      
    case FAIL_SETUP: {
      lcd.clear();
      lcd.print("GAME OVER :(");
      while (there.lat && waypoint <= MAX_WAYPOINT)
        EEPROM_readAnything(address_for(route, ++waypoint), there);
      EEPROM_readAnything(address_for(route, --waypoint), there);
      lcd.setCursor(0, 1);
      lcd.print("Afstand tot\neinddoel:\n");
      state = FAIL;
      break;
    }
    
    case FAIL: {
      lcd.print("\r");
      lcd.print(distance, 0);
      lcd.print(" m     ");
      break;
    }
    
    case PROGRAM_YESNO_SETUP: {
      yesno = false;
      lcd.clear();
      backlight(ON);
      lcd.print("Route ");
      lcd.print(route, DEC);
      lcd.print("\nprogrammeren?\nkort: wissel\nlang: bevestig\nGeselecteerd:\n");
      state = PROGRAM_YESNO_UPDATE;
      break;
    }

    case PROGRAM_YESNO_UPDATE: {
      lcd.print(yesno ? "\rJA " : "\rNEE");
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
      backlight(15000);
      lcd.clear();
      lcd.print("Ga naar\nwaypoint ");
      lcd.print(waypoint, DEC);
      if (waypoint > 1) {
        lcd.print("\nWP ");
        lcd.print(waypoint - 1, DEC);
        lcd.print(" -> hier:\n");
      }
      state = PROGRAM;
      break;
    }
      
    case PROGRAM: {
      switch (button()) {
        case NOT:
          if (waypoint < 2) break;
          lcd.print("\r");
          lcd.print(distance, 0);
          lcd.print("m      ");
          break;
        case LONG:
          state = PROGRAM_DONE;
          break;
        case SHORT:
          there.lat = here_lat;
          there.lon = here_lon;
          there.tolerance = 30;  // FIXME
          there.flags = 0;  // FIXME
          EEPROM_writeAnything(address_for(route, waypoint), there);
          state = ++waypoint > MAX_WAYPOINT ? PROGRAM_DONE : PROGRAM_UPDATE;
          break;
      }
      break;
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
      lcd.print("That's all,\nfolks!");
      delay(5000);
      programming = false;
      state = SELECT_ROUTE_SETUP;
      break;
    }
  }
  delay(10);
}
