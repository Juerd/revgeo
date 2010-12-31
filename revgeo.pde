#include <NewSoftSerial.h>
#include <LiquidCrystal.h>
#include <TinyGPS.h>

LiquidCrystal lcd(7, 8, 5, 6, 11, 12);
TinyGPS gps;
NewSoftSerial nss(2, 3);
int x;
int tellertje = 0;

void setup() {
  lcd.begin(20, 4);
  lcd.print("setup()");
  nss.begin(9600);
  Serial.begin(9600);
  delay(500);
  lcd.clear();
}

void loop() {
  float lat, lon;
  unsigned long age;
  float dist;
  char* s;


  while (nss.available()) {
    char c = nss.read();
    Serial.print(c);
    if (gps.encode(c)) {
      Serial.println("fix?");

      gps.f_get_position(&lat, &lon, &age);
      if (age == TinyGPS::GPS_INVALID_AGE) {
        lcd.print("No fix");
      } else {
        dist = TinyGPS::distance_between(
          lat, lon,
          52.070132, 4.343376
        );
        lcd.setCursor(0, 0);
        lcd.print(++tellertje);
        lcd.setCursor(0, 1);
        lcd.print(lat, 6);
        lcd.setCursor(0, 2);
        lcd.print(lon, 6);
        lcd.setCursor(0, 3);
        lcd.print(dist);
        if (dist < 20) {
          lcd.setCursor(0, 0);
          lcd.print("FOR GREAT JUSTICE!");
        }
      }
    } else {
      if (1 && ((++x % 200) == 0)) {
        unsigned long chars;
        unsigned short sentences;
        unsigned short failed;
        gps.stats(&chars, &sentences, &failed);
        Serial.print("\n******* Chars: ");
        Serial.print(chars);
        Serial.print(", Sentences: ");
        Serial.print(sentences);
        Serial.print(", Failed: ");
        Serial.println(failed);
      }
    }
    delay(0);
  }
}


