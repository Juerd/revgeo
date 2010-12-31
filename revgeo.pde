#include <NewSoftSerial.h>
#include <LiquidCrystal.h>
#include <TinyGPS.h>

LiquidCrystal lcd(7, 8, 5, 6, 11, 12);
TinyGPS       gps;
NewSoftSerial nss(2, 3);

boolean       fix        = false;
int           tries_left = 10;
float         here_lat;
float         here_lon;
float         dest_lat   = 52.070132;
float         dest_lon   =  4.343376;
unsigned long age;

void setup() {
  lcd.begin(20, 4);
  lcd.clear();
  lcd.println("Wacht op GPS-fix...");
  nss.begin(9600);
  Serial.begin(9600);
  delay(500);
}

int prevdist;

void loop() {
  int dist;

  while (nss.available()) {
    char c = nss.read();
    if (gps.encode(c)) {
      fix = 1;
      gps.f_get_position(&here_lat, &here_lon, &age);
    }
  }
  if (!fix) return;
  
  dist = (int) TinyGPS::distance_between(here_lat, here_lon, dest_lat, dest_lon);
  
  if (dist != prevdist) {
    lcd.setCursor(0, 0);
    lcd.print("Afstand = ");
    lcd.print(dist);
    lcd.print("m    ");
    prevdist = dist;
  }
  delay(100);
}


