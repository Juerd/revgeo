#include "AnythingEEPROM.h"
#include <EEPROM.h>
#include <PWMServo.h>
#include <Nokia5110.h>
#include <NewSoftSerial.h>
#include <TinyGPS.h>
#include <PowerPin.h>
#include <Button.h>

const byte MAX_WAYPOINT  = 10;
const byte MAX_ROUTE     = 10;
const byte TRIES[]       = { 15, 25, 35 };

Nokia5110     lcd(/*SCE*/7, /*RST*/8, /*DC*/9, /*SDIN*/11, /*SCLK*/12);
NewSoftSerial nss(/*RX*/2, /*TX*/14);
TinyGPS       gps;
PWMServo      servo;
PowerPin      servo_power(6);
PowerPin      backlight(13);
Button        button(4);

void setup() {
  servo.attach(10); // pin 9 or 10 only
  nss.begin(9600);
  intro();
}

void open_lock() {
  servo_power.on(1000);
  servo.write(180);
}

void close_lock() {
  servo_power.on(1000);
  servo.write(90);
}

int address_for(byte route, byte waypoint) {
  return (route - 1) * 100 + (waypoint - 1) * 10;
}

void intro() {
  lcd.clear();
  backlight.on();
  lcd.setInverse();
  delay(500);
  lcd.print("\n\n    REV\n       GEO");
  delay(1000);
  lcd.noInverse();
  delay(500);
//  Serial.begin(9600);
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
    PROGRAM_YESNO_SETUP,                       PROGRAM_YESNO,
          PROGRAM_SETUP,       PROGRAM_UPDATE, PROGRAM,       PROGRAM_DONE
  } state = SELECT_ROUTE_SETUP;
  
  static byte          route, waypoint, tries_left, triesidx;
  static byte          dotstate = 0;
  static boolean       programming = false, yesno;
  static unsigned long fix = 0;
  static float         distance;
  static Waypoint      there;
  static float         here_lat, here_lon;

  backlight.check();
  servo_power.check();

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
      open_lock();;
      route = 1;
      lcd.clear();
      lcd.print("Kies route.\n\n\n\nvolgende    OK\nkort      lang");
      state = SELECT_ROUTE_UPDATE;
      break;  
    }
    
    case SELECT_ROUTE_UPDATE: {
      lcd.setCursor(3,2);
      lcd.print("Route #");
      lcd.print(route, DEC);
      lcd.print("  ");
      state = SELECT_ROUTE;
      break;
    }
    
    case SELECT_ROUTE: {
      switch (button.pressed()) {
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
      lcd.print("Kies niveau.\n\n\n\nvolgende    OK\nkort      lang");
      state = SELECT_TRIES_UPDATE;
      break;  
    }
    
    case SELECT_TRIES_UPDATE: {
      lcd.setCursor(2,2);
      lcd.print(TRIES[triesidx], DEC);
      lcd.print(" pogingen  ");
      state = SELECT_TRIES;
      break;
    }
    
    case SELECT_TRIES: {
      switch (button.pressed()) {
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
      lcd.print("Mag de deksel \nop slot?");
      lcd.setCursor(0,4);
      lcd.print("            OK\n          lang");
      state = CLOSE;
      break;
    }
    
    case CLOSE: {
      if (button.pressed() != LONG) break;
      close_lock();
      state = WAIT_FOR_FIX_SETUP;
      break;
    }
 
    case WAIT_FOR_FIX_SETUP: {
      fix = 0;
      backlight.on(5000);
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
      switch(button.pressed()) {
        case SHORT:
          backlight.on(1500);
          break;
        case LONG:
          state = PROGRAM_YESNO_SETUP;
          break;
        default:
          if (fix && gps.hor_acc() < 500 && millis() - fix > 2000)
            state = programming ? PROGRAM_SETUP : ROUTE_SETUP;
          else if (++dotstate % 100)
            state = WAIT_FOR_FIX_UPDATE;
      }
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
      backlight.on(15000);
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
      if (distance < 0 || !button.pressed()) break;
      state = distance <= there.tolerance ? WAYPOINT_DONE
            : --tries_left                ? WAYPOINT_UPDATE
            :                               FAIL_SETUP;
      break;
    }
    
    case WAYPOINT_DONE: {
      lcd.clear();
      backlight.on();
      lcd.print("Waypoint ");
      lcd.print(waypoint,DEC);
      lcd.print("\nbereikt.");
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
      backlight.on(10000);
      open_lock();
      lcd.print("Gefeliciteerd!");
      lcd.print("Je bent er! :)");
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
      lcd.print("Afstand tot\neindpunt:\n");
      state = FAIL;
      break;
    }
    
    case FAIL: {
      switch (button.pressed(60000)) {
        case SHORT:
          backlight.on(1500);
          break;
        case LONG:
          open_lock();  // backdoor
          break;
      }
      lcd.print("\r");
      lcd.print(distance, 0);
      lcd.print(" m     ");
      break;
    }
    
    case PROGRAM_YESNO_SETUP: {
      open_lock();  // Pre-game backdoor
      yesno = false;
      lcd.clear();
      backlight.on();
      lcd.print("Route ");
      lcd.print(route, DEC);
      lcd.print("\nprogrammeren? ");
      lcd.setCursor(0,4);
      lcd.print("terug       OKkort      lang");
      state = PROGRAM_YESNO;
      break;
    }
     
    case PROGRAM_YESNO: {
      switch (button.pressed()) {
        case SHORT:
          state = CLOSE_SETUP;
          programming = 0;
          break;
        case LONG:
          state = WAIT_FOR_FIX_SETUP;
          programming = 1;
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
      backlight.on(15000);
      lcd.clear();
      lcd.print("Ga naar WP ");
      lcd.print(waypoint, DEC);
      lcd.print(".\n");
      if (waypoint > 1)
        lcd.print("Tussenafstand:");
      lcd.setCursor(0,4);
      lcd.print("WP       eindekort      lang");
      state = PROGRAM;
      break;
    }
      
    case PROGRAM: {
      switch (button.pressed()) {
        case NOT:
          lcd.setCursor(4, 2);
          if (waypoint < 2) break;
          lcd.print(distance, 0);
          lcd.print("m    ");
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
      lcd.print("Programmeren\nafgerond.");
      delay(5000);
      programming = false;
      state = SELECT_ROUTE_SETUP;
      break;
    }
  }
  delay(10);
}
