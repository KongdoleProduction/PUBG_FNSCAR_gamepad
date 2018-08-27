#define EMULATE

#ifdef EMULATE
#include <Mouse.h>
#include <Keyboard.h>
#endif

#include <SoftwareSerial.h>
#include <Crc16.h>

SoftwareSerial gunSerial(8, 2);

Crc16 crc;

void setup() {
  // put your setup code here, to run once:
  delay(500);

#ifdef EMULATE
  Keyboard.begin();
  Mouse.begin();
#else
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Begin!");
#endif

  gunSerial.begin(19200);
}

void loop() {
  static uint8_t isReading = false;
  static uint8_t buf_in_prev;
  static uint8_t buf[10]; 
  static uint8_t buf_idx = 0;
  const uint8_t packet_len = 10;

  if (gunSerial.available() > 0) {
    // check if incoming are start bytes
    uint8_t buf_in = gunSerial.read();
    if (isReading) {
      buf[buf_idx++] = buf_in;
      // check crc
      if (buf_idx == packet_len - 2) {
        isReading = false;
        uint16_t crc_num = (uint16_t)buf[packet_len-4] + ((uint16_t)buf[packet_len-3] << 8);
        //Serial.println(crc_num, HEX);
        if (crc_num == crc.XModemCrc(buf, 0, packet_len - 4)) {
          // reading done
#ifdef EMULATE
          uint8_t sw_trg, sw_rld, sw_mode, sw_scope, sw_joy, sw_grip;
          uint16_t mouseXRaw, mouseYRaw;
          int mouseX, mouseY;
          mouseXRaw = (uint16_t)buf[0] + ((uint16_t)buf[1] << 8);
          mouseYRaw = (uint16_t)buf[2] + ((uint16_t)buf[3] << 8);
          mouseX = (int)mouseXRaw - 32768;
          mouseY = (int)mouseYRaw - 32768;
          Mouse.move(mouseX / 10, mouseY / 10, 0);
#else
          for (int i=0; i<packet_len-4; i++) {
            Serial.print(buf[i], HEX); Serial.print(" ");
          }
          Serial.println("");
#endif
        }
      }
    }
    if (buf_in_prev == 0xFF && buf_in == 0xFF) {
      buf_idx = 0;
      isReading = true;
    }
    buf_in_prev = buf_in;
  }



  /*
  if (emulate) {

    //Serial.println(reload);
    if (trigger == LOW) {
      Serial.println("Fire!");
      digitalWrite(9, HIGH);
      delay(50);
      digitalWrite(9, LOW);
      delay(40);
    }

    if (trigger == LOW && trigger_prev == HIGH) {
      Serial.println("press");
      Mouse.press();
    } else if (trigger == HIGH && trigger_prev == LOW) {
      
      Serial.println("release");
      Mouse.release();
    }

    if (reload == LOW && reload_prev == HIGH) {
      Keyboard.write('R');
    }

    trigger_prev = trigger;
    reload_prev = reload;
  }*/

}
