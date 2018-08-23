#include <U8g2lib.h>
#include "AnythingEEPROM.h"
#include <EEPROM.h>
//#include <PWMServo.h>
#include <TinyGPS.h>
#include <PowerPin.h>
#include <Button.h>

#define seconds (1000)
#define second  (1000)

const byte  MAX_WAYPOINT = 10;
const byte  MAX_ROUTE    = 10;
const byte  TRIES[]      = { 15, 20, 25 };

const byte  VPIN         = A13;
const float VFACTOR      = 4096 / 7.4;
// Vbatt--[68k]--VPIN--[120k]--GND
// Theoretical factor is 68/(68+120) == .36, measured is .31
const float MINIMUM_STARTUP_VOLTAGE = 3.6;
const float EMERGENCY_OPEN_VOLTAGE  = 3.0;

#define default_font u8g2_font_7x13B_tf
static unsigned long fix = 0;  // millis of last position

//Nokia5110     lcd(/*SCE*/7, /*RST*/8, /*DC*/9, /*SDIN*/11, /*SCLK*/12);

U8G2_SH1106_128X64_NONAME_F_HW_I2C lcd(U8G2_R0);

HardwareSerial gps_serial(2);
TinyGPS       gps;
//PWMServo      servo;
//PowerPin      servo_power(6);
PowerPin      backlight(13);
Button        button(14);
int v_usb_pin = A10;

void draw_icons(bool send = false) {
  lcd.setDrawColor(0);
  lcd.drawBox(128 - 16, 0, 16, 30);
  lcd.setDrawColor(1);
  
  if (analogRead(v_usb_pin) > 1000) {
    lcd.setFont(u8g2_font_open_iconic_embedded_2x_t);
    lcd.drawStr(128 - 16, 0, "C");
    lcd.setFont(default_font);  
  } else {
    lcd.drawBox(128 - 16 + 4 + 2, 0, 3, 1);  // battery top
    lcd.drawFrame(128 - 16 + 4, 1, 7, 15);
    float v = battery_voltage();
    if (v > 4)   lcd.drawBox(128 - 16 + 4 + 2,  3, 3, 2);
    if (v > 3.7) lcd.drawBox(128 - 16 + 4 + 2,  6, 3, 2);
    if (v > 3.6) lcd.drawBox(128 - 16 + 4 + 2,  9, 3, 2);
    if (v > 3.5) lcd.drawBox(128 - 16 + 4 + 2, 12, 3, 2);
  }
  
  lcd.setFont(u8g2_font_open_iconic_all_2x_t);
  int hdop = gps.hor_acc();
  lcd.drawStr(128 - 16, 18, !fix ? "\xcf" : hdop < 150 ? "\xb1" : hdop < 250 ? "\xb2" : "\xb3");
  lcd.setFont(default_font);
  if (send) lcd.sendBuffer();
}

void clear() {
  lcd.clear();
  draw_icons();
}

void setup() {
//  servo.attach(10); // pin 9 or 10 only
  EEPROM.begin(1024);
  gps_serial.begin(9600);
  Serial.begin(115200);
  Serial.println("setup");
  pinMode(14, INPUT_PULLUP);
  //pinMode(v_usb_pin, INPUT_PULLDOWN);
  lcd.begin();
  lcd.setContrast(255);
  lcd.setLineHeight(10);
  lcd.setFontPosTop();
  lcd.setFont(default_font);
  lcd.setFontMode(0);  // non-transparent background
  for (int i = 0; i < 1024; i++) {
    byte c = EEPROM.read(i);
    Serial.printf("%02x ", c);
    
  }
  intro();
}

boolean lock_is_open = false;

void open_lock() {
//  servo_power.on(1 *second);
//  servo.write(180);
  lock_is_open = true;
}

void close_lock() {
//  servo_power.on(1 *second);
//  servo.write(90);data
  lock_is_open = false;
}

float battery_voltage() {
  return analogRead(VPIN) / VFACTOR;
}

int address_for(byte route, byte waypoint) {
  // route and waypoint are >= 1
  return (route - 1) * 100 + (waypoint - 1) * 10;
}

void intro() {
  Serial.println("setup");

  clear();
  
  // Check voltage *before* using servo.
  if (battery_voltage() < MINIMUM_STARTUP_VOLTAGE) {
    Serial.println("batt");
    lcd.setCursor(0, 10);
    lcd.printf("Batterijspanning\nte laag! (%.2f V)", battery_voltage());
    lcd.sendBuffer();
    lcd.setContrast(5);
    // Draw attention and deny further usage
    for (;;) {
      delay(0.4 *seconds); lcd.setPowerSave(true);  // display off
      delay(0.4 *seconds); lcd.setPowerSave(false);
    }
  }

  open_lock();
  backlight.on();
  
  //lcd.setInverse();
  delay(1/2 *second);
  clear();
  lcd.print("\n\n    REV\n       GEO\n\n");
  lcd.sendBuffer();
  delay(1 *second);
  
  //lcd.noInverse();
  delay(1 *second);
  clear();
  Serial.println("setup klaar");
  
}

struct Waypoint {
  // Don't alter struct; stored in EEPROM
  float lat;
  float lon;
  byte tolerance;
  byte flags;
};

enum state_enum {
                                             CRASHED,
   SELECT_ROUTE_SETUP,  SELECT_ROUTE_UPDATE, SELECT_ROUTE,
   SELECT_TRIES_SETUP,  SELECT_TRIES_UPDATE, SELECT_TRIES,
          CLOSE_SETUP,                       CLOSE,
   WAIT_FOR_FIX_SETUP,  WAIT_FOR_FIX_UPDATE, WAIT_FOR_FIX,
          ROUTE_SETUP,                                      ROUTE_DONE,
       WAYPOINT_SETUP,      WAYPOINT_UPDATE, WAYPOINT,      WAYPOINT_DONE,
                                                   WAYPOINT_CHECK,
        FAILURE_SETUP,                       FAILURE,
  PROGRAM_YESNO_SETUP,                       PROGRAM_YESNO,
        PROGRAM_SETUP,       PROGRAM_UPDATE, PROGRAM,       PROGRAM_DONE,
                                                   PROGRAM_STORE,
       PROGRESS_SETUP,      PROGRESS_UPDATE, PROGRESS,
};

void loop() {
  static state_enum    state = SELECT_ROUTE_SETUP, nextstate;
  static byte          route, waypoint, tries_left, triesidx, progress;
  static boolean       yesno;
  static unsigned long statetimer = 0, lastVcheck = 0;
  static float         distance;
  static Waypoint      there;
  static float         here_lat, here_lon;

  if (lock_is_open && (millis() - lastVcheck > 10 *seconds)) {
    lastVcheck = millis();
    if (battery_voltage() < EMERGENCY_OPEN_VOLTAGE) {
      open_lock();
      clear();
      lcd.print("Slot is open\nomdat de\nbatterijen\nbijna leeg\nzijn.");
      delay(10 *seconds);
    }
  }

  backlight.check();
  //servo_power.check();

  while (gps_serial.available()) {
    char c = gps_serial.read();
    Serial.print(c);
    if (gps.encode(c)) {
      unsigned long age;
      if (!fix) fix = millis();
      gps.f_get_position(&here_lat, &here_lon, &age);
      distance = TinyGPS::distance_between(
        here_lat, here_lon,
        there.lat, there.lon
      );
      Serial.printf("%f/%f - %f/%f = %f\n", here_lat, here_lon, there.lat, there.lon, distance);
    }
  }

  static int once_every = 0;
  if (once_every++ > 100) {
    once_every = 0;
    draw_icons(true);
  }
  
  switch (state) {
    case CRASHED: break;
    
    case SELECT_ROUTE_SETUP: {
      route = 1;
      clear();
      lcd.print("Kies route.\n\n\n\nvolgende    OK\nkort      lang");
      lcd.sendBuffer();
      state = SELECT_ROUTE_UPDATE;
      break;  
    }
    
    case SELECT_ROUTE_UPDATE: {
      lcd.setCursor(0,20);
      lcd.print("Route #");
      lcd.print(route, DEC);
      lcd.print("  ");
      lcd.sendBuffer();
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
      clear();
      lcd.print("Kies niveau.\n\n\n\nvolgende    OK\nkort      lang");
      lcd.sendBuffer();
      state = SELECT_TRIES_UPDATE;
      break;  
    }
    
    case SELECT_TRIES_UPDATE: {
      lcd.setCursor(0,20);
      lcd.print(TRIES[triesidx], DEC);
      lcd.print(" pogingen  ");
      lcd.sendBuffer();
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
      clear();
      lcd.print("Mag de deksel \nop slot?");
      lcd.setCursor(0,40);
      lcd.print("            OK\n          lang");
      lcd.sendBuffer();
      state = CLOSE;
      break;
    }
    
    case CLOSE: {
      if (button.pressed() != LONG) break;
      close_lock();
      state = WAIT_FOR_FIX_SETUP;
      nextstate = ROUTE_SETUP;
      break;
    }
 
    case WAIT_FOR_FIX_SETUP: {
      fix = 0;
      progress = 0;  // number of dots
      backlight.on(5 *seconds);
      clear();
      lcd.print("Wacht op\nGPS-fix...");
      lcd.sendBuffer();
      state = WAIT_FOR_FIX_UPDATE;
      break;
    }
      
    case WAIT_FOR_FIX_UPDATE: {
      progress++;
      if (progress > 4) progress = 1;
      lcd.setCursor(70, 10);
      lcd.setDrawColor(0);
      lcd.drawBox(70,10,40,12);
      lcd.setDrawColor(1);
      for (byte i = 0; i < progress; i++) lcd.print(".");
      lcd.sendBuffer();
      statetimer = millis();
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
          if (fix && gps.hor_acc() < 150 && millis() - fix > 2 *seconds)
            state = nextstate;
          else if ((millis() - statetimer) > 400)
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
      EEPROM_readAnything(address_for(route, waypoint), there);
      tries_left = TRIES[triesidx];
      distance   = -1;
      state = there.lat ? WAYPOINT_UPDATE : ROUTE_DONE;
      break;
    }
    
    case WAYPOINT_UPDATE: {
      clear();
      backlight.on(15 *seconds);
      lcd.print("Onderweg naar\nwaypoint ");
      lcd.print(waypoint, DEC);
      lcd.print(".\n");
      if (distance >= 0) {
        lcd.print("Afstand:\n    ");
        lcd.print(distance, 0);
        lcd.print(" m");
      }
      lcd.setCursor(0, 40);
      lcd.print("Je mag ");
      lcd.print(
        waypoint > 1 && tries_left == TRIES[triesidx]
        ? "weer "
        : "nog "
      );
      lcd.print(tries_left, DEC);
      lcd.print(" \nkeer drukken.");
      lcd.sendBuffer();
      state = WAYPOINT;
      break;
    }
      
    case WAYPOINT: {
      if (distance < 0 || !button.pressed()) break;
      state = PROGRESS_SETUP;
      nextstate = WAYPOINT_CHECK;
      break;
    }

    case WAYPOINT_CHECK: {
      state = distance <= there.tolerance ? WAYPOINT_DONE
            : --tries_left                ? WAYPOINT_UPDATE
            :                               FAILURE_SETUP;
      break;
    }

    
    case WAYPOINT_DONE: {
      clear();
      backlight.on();
      lcd.print("Waypoint ");
      lcd.print(waypoint,DEC);
      lcd.print("\nbereikt.");
      lcd.sendBuffer();
      delay(4 *seconds);
      if (waypoint == MAX_WAYPOINT) {
        state = ROUTE_DONE;
        break;
      }      
      waypoint++;      
      state = WAYPOINT_SETUP;
      break;
    }
    
    case ROUTE_DONE: {
      clear();
      backlight.on(10 *seconds);
      open_lock();
      lcd.print("Gefeliciteerd!\n");
      lcd.print("Je bent er! :)");
      lcd.sendBuffer();
      state = CRASHED;
      break;
    }
      
    case FAILURE_SETUP: {
      clear();
      backlight.on(15 *seconds);
      lcd.print("GAME OVER :(");

      // Null terminated, so read until the null and then go back one entry.
      while (there.lat && waypoint <= MAX_WAYPOINT)
        EEPROM_readAnything(address_for(route, ++waypoint), there);
      EEPROM_readAnything(address_for(route, --waypoint), there);

      lcd.setCursor(0, 10);
      lcd.print("Afstand tot\neindpunt:\n");
      lcd.sendBuffer();
      state = FAILURE;
      break;
    }
    
    case FAILURE: {
      switch (button.pressed(60000)) {  // 60 *seconds werkt niet!! :(
        case SHORT:
          backlight.on(1500);
          break;
        case LONG:
          open_lock();  // Backdoor
          break;
      }
      lcd.print("\r");
      lcd.print(distance, 0);
      lcd.print(" m     ");
      lcd.sendBuffer();
      break;
    }
    
    case PROGRAM_YESNO_SETUP: {
      open_lock();  // Pre-game backdoor
      yesno = false;
      clear();
      backlight.on();
      lcd.print("Route ");
      lcd.print(route, DEC);
      lcd.print("\nprogrammeren? ");
      lcd.setCursor(0,40);
      lcd.print("terug       OK\nkort      lang");
      lcd.sendBuffer();
      state = PROGRAM_YESNO;
      break;
    }
     
    case PROGRAM_YESNO: {
      switch (button.pressed()) {
        case SHORT:
          state = CLOSE_SETUP;
          break;
        case LONG:
          state = WAIT_FOR_FIX_SETUP;
          nextstate = PROGRAM_SETUP;
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
      backlight.on(15 *seconds);
      clear();
      lcd.print("Ga naar WP ");
      lcd.print(waypoint, DEC);
      lcd.print(".\n");
      if (waypoint > 1)
        lcd.print("Tussenafstand:");
      lcd.setCursor(0,40);
      lcd.print("WP       einde\nkort      lang");
      lcd.sendBuffer();
      state = PROGRAM;
      break;
    }
      
    case PROGRAM: {
      switch (button.pressed()) {
        case NOT:
          lcd.setDrawColor(0);
          lcd.drawBox(0,20,128,10);
          lcd.setDrawColor(1);
          lcd.setCursor(0, 20);
          if (waypoint < 2) break;
          lcd.print(distance, 0);
          lcd.print(" m       ");
          lcd.sendBuffer();
          break;
        case LONG:
          state = PROGRAM_DONE;
          break;
        case SHORT:
          state = PROGRESS_SETUP;
          nextstate = PROGRAM_STORE;
          break;
      }
      break;
    }
      
    case PROGRAM_STORE: {
      there.lat = here_lat;
      there.lon = here_lon;  
      there.tolerance = 15;  // FIXME
      there.flags = 0;  // FIXME
      EEPROM_writeAnything(address_for(route, waypoint), there);
      EEPROM.commit();
      state = ++waypoint > MAX_WAYPOINT ? PROGRAM_DONE : PROGRAM_UPDATE;
      break;
    }
    
    case PROGRAM_DONE: {
      backlight.on(10 *seconds);
      if (waypoint <= MAX_WAYPOINT) {
        // Terminate with null
        there.lat = 0;
        there.lon = 0;
        there.tolerance = 0;
        there.flags = 0;
        EEPROM_writeAnything(address_for(route, waypoint), there);
        EEPROM.commit();
      }
      clear();
      lcd.print("Programmeren\nafgerond.");
      lcd.sendBuffer();
      delay(5 *seconds);
      state = SELECT_ROUTE_SETUP;
      break;
    }
    
    case PROGRESS_SETUP: {
      clear();
      backlight.on();
      lcd.print("Blijf waar je\nbent.\n\nWacht...");
      lcd.setCursor(0, 60);
      lcd.sendBuffer();
      progress = 0;
      state = PROGRESS_UPDATE;
      break; 
    }
    
    case PROGRESS_UPDATE: {
      statetimer = millis();
      lcd.drawBox(0,50, progress,14);
      lcd.sendBuffer();
      state = PROGRESS;
      break; 
    }
    
    case PROGRESS: {
      if ((millis() - statetimer) < 100) break;
      state = ++progress < 128 ? PROGRESS_UPDATE : nextstate;
      break;
    }
      
  }
  delay(10);
}
