#define EMULATE

#ifdef EMULATE
#include <Mouse.h>
#include <Keyboard.h>
#endif

#include <SoftwareSerial.h>
#include <Crc16.h>

SoftwareSerial gunSerial(8, 2);

Crc16 crc;

const char KEY_RELOAD = 'r'; const char KEY_MODE = 'b';
const char KEY_SCOPE_ON = 'o';
const char KEY_SCOPE_OFF = 'p';
const char KEY_SIT = 'c';
const char KEY_STAND_UP = ' ';
const char KEY_JUMP = ' ';
const char KEY_RIGHT = 'd';
const char KEY_LEFT = 'a';
const char KEY_UP = 'w';
const char KEY_DOWN = 's';
const char KEY_INTERACT = 'f';

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
          uint8_t sw_trg, sw_rld, sw_mode, sw_scope, sw_joy, sw_grip, isJumping;
          static uint8_t sw_trg_prev, sw_rld_prev, sw_mode_prev,
                         sw_scope_prev, sw_joy_prev, sw_grip_prev, isJumpingPrev;
          sw_trg = buf[5] & 0x1;
          sw_rld = (buf[5] >> 1) & 0x1;
          sw_mode = (buf[5] >> 2) & 0x1;
          sw_scope = (buf[5] >> 3) & 0x1;
          sw_joy = (buf[5] >> 4) & 0x1;
          sw_grip = (buf[5] >> 5) & 0x1;
          isJumping = (buf[5] >> 6) & 0x1;

          uint8_t joy_up, joy_down, joy_left, joy_right, joy_run, joy_walk;
          static uint8_t joy_up_prev = true, joy_down_prev = true, joy_left_prev = true,
                         joy_right_prev = true, joy_run_prev, joy_walk_prev;

          joy_right = buf[4] & 0x01;
          joy_left = buf[4] & 0x02;
          joy_up = buf[4] & 0x04;
          joy_down = buf[4] & 0x08;
          joy_run = buf[4] & 0x10;
          joy_walk = buf[4] & 0x20;
          
          uint16_t mouseXRaw, mouseYRaw;
          int mouseX, mouseY;
          mouseXRaw = (uint16_t)buf[0] + ((uint16_t)buf[1] << 8);
          mouseYRaw = (uint16_t)buf[2] + ((uint16_t)buf[3] << 8);
          mouseX = (int)mouseXRaw - 32768;
          mouseY = (int)mouseYRaw - 32768;

          if (!sw_trg && sw_trg_prev) {
            Mouse.press();
          } else if (sw_trg && !sw_trg_prev) {
            Mouse.release();
          }

          if (!sw_rld && sw_rld_prev) {
            Keyboard.write(KEY_RELOAD);
          }

          if (!sw_mode && sw_mode_prev) {
            //Keyboard.write(KEY_MODE);
            Mouse.press(MOUSE_RIGHT);
          } else if (sw_mode && !sw_mode_prev) {
            Mouse.release(MOUSE_RIGHT);
          }

          if (!sw_scope && sw_scope_prev) {
            Keyboard.write('q');
          } /*else if (sw_scope && !sw_scope_prev) {
            Keyboard.write(KEY_SCOPE_OFF);
          }*/

          if (!sw_joy && sw_joy_prev) {
            Keyboard.write(KEY_LEFT_SHIFT);
          } else if (sw_joy && !sw_joy_prev) {
            //Keyboard.write(KEY_STAND_UP);
          }

          if (!isJumping && isJumpingPrev) {
            Keyboard.write(KEY_JUMP);
          }

          if (joy_up && !joy_up_prev) {
            Keyboard.press(KEY_UP);
          } else if (!joy_up && joy_up_prev) {
            Keyboard.release(KEY_UP);
          }

          if (joy_down && !joy_down_prev) {
            Keyboard.press(KEY_DOWN);
          } else if (!joy_down && joy_down_prev) {
            Keyboard.release(KEY_DOWN);
          }

          if (joy_left && !joy_left_prev) {
            Keyboard.press(KEY_LEFT);
          } else if (!joy_left && joy_left_prev) {
            Keyboard.release(KEY_LEFT);
          }

          if (joy_right && !joy_right_prev) {
            Keyboard.press(KEY_RIGHT);
          } else if (!joy_right && joy_right_prev) {
            Keyboard.release(KEY_RIGHT);
          }

          /*
          if (joy_walk && !joy_walk_prev) {
            Keyboard.press(KEY_LEFT_CTRL);
          } else if (!joy_walk && joy_walk_prev) {
            Keyboard.release(KEY_LEFT_CTRL);
          }*/

          /*
          if (joy_run && !joy_run_prev) {
            Keyboard.press(KEY_LEFT_SHIFT);
          } else if (!joy_run && joy_run_prev) {
            Keyboard.release(KEY_LEFT_SHIFT);
          }
          */
          
          if (!sw_grip) {
            Mouse.move(mouseX / 15.0, -mouseY / 15, 0);
          }

          sw_trg_prev = sw_trg;
          sw_rld_prev = sw_rld;
          sw_mode_prev = sw_mode;
          sw_scope_prev = sw_scope;
          sw_joy_prev = sw_joy;
          sw_grip_prev = sw_grip;
          isJumpingPrev = isJumping;

          joy_left_prev = joy_left;
          joy_right_prev = joy_right;
          joy_up_prev = joy_up;
          joy_down_prev = joy_down;
          joy_walk_prev = joy_walk;
          joy_run_prev = joy_run;
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
