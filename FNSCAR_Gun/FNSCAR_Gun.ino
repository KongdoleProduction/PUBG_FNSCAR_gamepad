#include <MPU6050_tockn.h>
#include <Wire.h>
#include <Crc16.h>

//#define IMU_CALIB

#define SW_TRG 4
#define SW_RLD 5
#define SW_MODE 6
#define SW_SCOPE 7
#define SW_JOY 8
#define FEEDBACK 2
#define SW_GRIP 9
#define VRx A0
#define VRy A1

MPU6050 mpu6050(Wire);

Crc16 crc;

void setup() {
  Wire.begin();
  Serial.begin(19200);
  mpu6050.begin();

#ifdef IMU_CALIB
  double gyro_x_bias = 0, gyro_y_bias = 0, gyro_z_bias = 0;
  const int SAMPLE_NUM = 300;
  for(int i=0; i<SAMPLE_NUM; i++) {
    mpu6050.update();
    gyro_x_bias += (mpu6050.getGyroX() / SAMPLE_NUM);
    gyro_y_bias += (mpu6050.getGyroY() / SAMPLE_NUM);
    gyro_z_bias += (mpu6050.getGyroZ() / SAMPLE_NUM);
    delay(10);
  }
  Serial.println(gyro_x_bias);
  Serial.println(gyro_y_bias);
  Serial.println(gyro_z_bias);
  while(true);
#endif

  pinMode(SW_TRG, INPUT_PULLUP);
  pinMode(SW_RLD, INPUT_PULLUP);
  pinMode(SW_MODE, INPUT_PULLUP);
  pinMode(SW_SCOPE, INPUT_PULLUP);
  pinMode(SW_JOY, INPUT_PULLUP);
  pinMode(SW_GRIP, INPUT_PULLUP);
  pinMode(FEEDBACK, OUTPUT);
}

void loop() {
  static unsigned long timer = 0;
  mpu6050.update();

  // recoil feedback
  bool sw_trg;
  static byte fb_state;
  static unsigned long fb_ts;
  static int fb_speed_offset = 10;
  static bool fb_en = true;

  /*
  if (Serial.available() > 0) {
    fb_speed_offset = Serial.read() - '0';
    if (fb_speed_offset == 0) {
      fb_en = false;
    } else {
      fb_en = true;
    }
  }*/

  if(millis() - timer > 10){

    sw_trg = digitalRead(SW_TRG);
    if (!sw_trg && fb_en) {
      if (fb_state == 0) {
        digitalWrite(2, HIGH);
        fb_state = 1;
        fb_ts = millis();
      } else if (fb_state == 1 && millis() - fb_ts > 40) {
        digitalWrite(2, LOW);
        fb_state = 2;
        fb_ts = millis();
      } else if (fb_state == 2 && millis() - fb_ts > (10 + fb_speed_offset)) {
        fb_state = 0;
      }
    } else {
      fb_state = 0;
      digitalWrite(2, LOW);
    }

    float gyro_x_raw, gyro_y_raw, gyro_z_raw;
    const float gyro_x_bias = -1.03;
    const float gyro_y_bias = 1.35;
    const float gyro_z_bias = 1.34;
    float acc_x_raw, acc_y_raw, acc_z_raw;
    float ang_bias_z = 3.14 / 6;  // 30 deg
    float tilt, pan;
    gyro_x_raw = mpu6050.getGyroX() - gyro_x_bias;
    gyro_y_raw = mpu6050.getGyroY() - gyro_y_bias;
    gyro_z_raw = mpu6050.getGyroZ() - gyro_z_bias;
    
    acc_x_raw = mpu6050.getAccX();
    acc_y_raw = mpu6050.getAccY();
    acc_z_raw = mpu6050.getAccZ();

    //ang_z = mpu6050.getAngleZ();

    float gyro_x, gyro_y, gyro_z;
    float acc_x, acc_y, acc_z;

    gyro_x = cos(ang_bias_z) * gyro_x_raw + sin(ang_bias_z) * gyro_y_raw;
    gyro_y = -sin(ang_bias_z) * gyro_x_raw + cos(ang_bias_z) * gyro_y_raw;
    gyro_z = gyro_z_raw;

    acc_x = cos(ang_bias_z) * acc_x_raw + sin(ang_bias_z) * acc_y_raw;
    acc_y = -sin(ang_bias_z) * acc_x_raw + cos(ang_bias_z) * acc_y_raw;
    acc_z = acc_z_raw;

    pan = gyro_y;
    tilt = gyro_z;

    uint16_t mouseX = 32768, mouseY = 32768;

    if (tilt > 2 || tilt < -2) {
      mouseY = tilt * 10 + 32768;
      if (mouseY == 0xFFFF) mouseY = 0xFFFE;
    }
    if (pan > 2 || pan < -2) {
      mouseX = pan * 10 + 32768;
      if (mouseX == 0xFFFF) mouseX = 0xFFFE;
    }

    uint8_t movement = 0;
    float joyX = (float)(analogRead(VRx) - 512) / 512;
    float joyY = (float)(analogRead(VRy) - 512) / 512;
    float joySize = sqrt(pow(joyX, 2) + pow(joyY, 2));
    if (joyX > 0.05) {
      movement |= 0b00000001;
    } else if (joyX < -0.05) {
      movement |= 0b00000010;
    }
    if (joyY > 0.05) {
      movement |= 0b00000100;
    } else if (joyY < -0.05) {
      movement |= 0b00001000;
    }
    if (joySize > 0.8) {
      movement |= 0b00010000;
    } else if (joySize > 0.05 && joySize < 0.3) {
      movement |= 0b00100000;
    }

    boolean isJumping = (acc_y > 0);

    const uint8_t packet_len = 10;
    uint8_t packet[packet_len];
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = mouseX & 0xFF;
    packet[3] = (mouseX >> 8) & 0xFF;
    packet[4] = mouseY & 0xFF;
    packet[5] = (mouseY >> 8) & 0xFF;
    packet[6] = movement;
    packet[7] = 0;
    packet[7] |= (uint8_t)digitalRead(SW_TRG);
    packet[7] |= (uint8_t)digitalRead(SW_RLD)   << 1;
    packet[7] |= (uint8_t)digitalRead(SW_MODE)  << 2;
    packet[7] |= (uint8_t)digitalRead(SW_SCOPE) << 3;
    packet[7] |= (uint8_t)digitalRead(SW_JOY)   << 4;
    packet[7] |= (uint8_t)digitalRead(SW_GRIP)  << 5;
    packet[7] |= (uint8_t)isJumping             << 6;
    uint16_t crc_num = crc.XModemCrc(packet, 2, packet_len-4);
    packet[8] = crc_num & 0xFF;
    packet[9] = (crc_num >> 8) & 0xFF;
    Serial.write(packet, packet_len);
    
    timer = millis();
  }

}
