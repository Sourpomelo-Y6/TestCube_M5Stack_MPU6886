//#include <M5StickC.h>
#include <M5StickCPlus.h>
#include <SPIFFS.h> 
#include <LovyanGFX.hpp>
#include <Preferences.h>
#include "MadgwickAHRS.h"

Preferences preferences;
Madgwick *filter = new Madgwick();

static LGFX lcd;
static LGFX_Sprite sprite[2];

//#pragma GCC optimize ("O3")
struct point3df{ float x, y, z;};
struct surface{ uint8_t p[4]; int16_t z;};
#define U  70         // size of cube
 
struct point3df cubef[8] ={ // cube edge length is 2*U
  { -U, -U,  U },
  {  U, -U,  U },
  {  U, -U, -U },
  { -U, -U, -U },
  { -U,  U,  U },
  {  U,  U,  U },
  {  U,  U, -U },
  { -U,  U, -U },
};
 
struct surface s[6] = {// define the surfaces
  { {0, 1, 2, 3}, 0 }, // bottom
  { {4, 5, 6, 7}, 0 }, // top
  { {0, 1, 5, 4}, 0 }, // back
  { {3, 7, 6, 2}, 0 }, // front
  { {1, 2, 6, 5}, 0 }, // right
  { {0, 3, 7, 4}, 0 }, // left
};
 
struct point3df cubef2[8];

float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;

float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;

float pitch = 0.0F;
float roll  = 0.0F;
float yaw   = 0.0F;

double sum_accX = 0.0;
double sum_accY = 0.0;
double sum_accZ = 0.0;

double sum_gyroX = 0.0;
double sum_gyroY = 0.0;
double sum_gyroZ = 0.0;

float calb_accX = 0.0;
float calb_accY = 0.0;
float calb_accZ = 0.0;

float calb_gyroX = 0.0;
float calb_gyroY = 0.0;
float calb_gyroZ = 0.0;

bool flip;
uint32_t pre_calc_time = 0;
uint32_t pre_show_time = 0;

int ws;
int hs;

void rotate_cube_xyz( float roll, float pitch, float yaw){
  uint8_t i;

  float DistanceCamera = 300;
  float DistanceScreen = 550;

  float cosyaw   = cos(yaw);
  float sinyaw   = sin(yaw);
  float cospitch = cos(pitch);
  float sinpitch = sin(pitch);
  float cosroll  = cos(roll);
  float sinroll  = sin(roll);

  float sinyaw_sinroll = sinyaw * sinroll;
  float sinyaw_cosroll = sinyaw * cosroll;
  float cosyaw_sinroll = cosyaw * sinroll;
  float cosyaw_cosroll = cosyaw * cosroll;

  float x_x = cosyaw * cospitch;
  float x_y = cosyaw_sinroll * sinpitch - sinyaw_cosroll;
  float x_z = cosyaw_cosroll * sinpitch + sinyaw_sinroll;

  float y_x = sinyaw * cospitch;
  float y_y = sinyaw_sinroll * sinpitch + cosyaw_cosroll;
  float y_z = sinyaw_cosroll * sinpitch - cosyaw_sinroll;

  float z_x = -sinpitch;
  float z_y = cospitch * sinroll;
  float z_z = cospitch * cosroll;

  for (i = 0; i < 8; i++){
    float x = x_x * cubef[i].x
            + x_y * cubef[i].y
            + x_z * cubef[i].z;
    float y = y_x * cubef[i].x
            + y_y * cubef[i].y
            + y_z * cubef[i].z;
    float z = z_x * cubef[i].x
            + z_y * cubef[i].y
            + z_z * cubef[i].z;

    cubef2[i].x = (x * DistanceCamera) / (z + DistanceCamera + DistanceScreen) + (ws>>1);
    cubef2[i].y = (y * DistanceCamera) / (z + DistanceCamera + DistanceScreen) + (hs>>1);
    cubef2[i].z = z;
  }
}


void setup(void){ 
  M5.begin();
  M5.IMU.Init();
  
  preferences.begin("imu_calb_data", true);
  calb_accX = preferences.getFloat("ax", 0);
  calb_accY = preferences.getFloat("ay", 0);
  calb_accZ = preferences.getFloat("az", 0);
  calb_gyroX = preferences.getFloat("gx", 0);
  calb_gyroY = preferences.getFloat("gy", 0);
  calb_gyroZ = preferences.getFloat("gz", 0);
  Serial.println(calb_accX);
  Serial.println(calb_accY);
  Serial.println(calb_accZ);
  Serial.println(calb_gyroX);
  Serial.println(calb_gyroY);
  Serial.println(calb_gyroZ);
  preferences.end();

  lcd.init();

  lcd.setRotation(1);


// バックライトの輝度を 0～255 の範囲で設定します。
//  lcd.setBrightness(80);

  lcd.setColorDepth(16);  // RGB565の16ビットに設定

  lcd.fillScreen(0);

  //ws = lcd.width();
  //hs = lcd.height();
  ws = 160;
  hs = 80;

  sprite[0].createSprite(ws,hs);
  sprite[1].createSprite(ws,hs);
  lcd.startWrite();
  lcd.fillScreen(TFT_DARKGREY);
}

float smoothMove(float dst, float src)
{
  if (     dst + M_PI < src) src -= M_PI * 2;
  else if (src + M_PI < dst) src += M_PI * 2;
  return (dst + src * 19.0) / 20.0;
}

void loop(void)
{
  int calc_time = millis() - pre_calc_time;
  
  //count = (count + 1) & 3;
  //if (count == 0)
  if (calc_time >= 9)
  {  
    M5.update();
    if(M5.BtnA.isPressed()){
      Serial.println("Calibration Start");
      getCalibrationVal();
    }

    M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
    M5.IMU.getAccelData(&accX,&accY,&accZ);

    filter->MadgwickAHRSupdateIMU(
      PI/180.0F*(gyroX - calb_gyroX), 
      PI/180.0F*(gyroY - calb_gyroY), 
      PI/180.0F*(gyroZ - calb_gyroZ),
      accX - calb_accX, 
      accY - calb_accY, 
      accZ - calb_accZ
      );

    pre_calc_time = millis();
  }

  pitch = smoothMove(filter->getPitch(), pitch);
  roll  = smoothMove(filter->getRoll() , roll );
  yaw   = smoothMove(filter->getYaw()  , yaw  );

  rotate_cube_xyz(roll, pitch, yaw);

  //描写する面の順番に並び替え
  int ss[6]={0,1,2,3,4,5};
  float sf[6]={0};
  for (int i = 0; i < 6; i++)
  {
    float wz = 0;
    for(int j=0;j<4;j++){
      wz += cubef2[s[i].p[j]].z;
    }
    sf[i] = wz;
  }
  //交換ソート
  for (int j = 5; j > 0; j--){
    for (int i = 0; i < j; i++)
    {
        if(sf[i] < sf[i+1])
        {
          float work = sf[i];
          sf[i] = sf[i+1];
          sf[i+1] = work;
          
          int iw = ss[i];
          ss[i] = ss[i+1];
          ss[i+1] = iw;
        }
    }
  }

  flip = !flip;
  sprite[flip].clear();
  for (int i = 0; i < 8; i++)
  {
    sprite[flip].drawRect( (int)cubef2[i].x-2, (int)cubef2[i].y-2, 4, 4 , 0xF000);
    //Serial.printf("%d,%f,%f,\r\n",i,cubef2[i].x, cubef2[i].y); 
  }

  for (int i = 0; i < 6; i++)
  {
    int ii = ss[i];
    
    sprite[flip].setColor( lcd.color565(((( ii + 1) &  1)      * 255),
                                  ((((ii + 1) >> 1) & 1) * 255),
                                  ((((ii + 1) >> 2) & 1) * 255)
                   )             );
    sprite[flip].fillTriangle(    cubef2[s[ii].p[0]].x, cubef2[s[ii].p[0]].y,
                            cubef2[s[ii].p[1]].x, cubef2[s[ii].p[1]].y,
                            cubef2[s[ii].p[2]].x, cubef2[s[ii].p[2]].y);
    sprite[flip].fillTriangle(    cubef2[s[ii].p[2]].x, cubef2[s[ii].p[2]].y,
                            cubef2[s[ii].p[3]].x, cubef2[s[ii].p[3]].y,
                            cubef2[s[ii].p[0]].x, cubef2[s[ii].p[0]].y);
  }
  
  int show_time = millis() - pre_show_time;
  pre_show_time = millis();
  sprite[flip].setCursor(0, 50);
  sprite[flip].printf("%5d",show_time);

  if (calc_time >= 9){
    sprite[flip].setCursor(0, 60);
    sprite[flip].printf("%5d",calc_time);
  }
  
  sprite[flip].pushSprite(&lcd, 0, 0);
}

void getCalibrationVal()
{
    const int CalbNum = 1000;
    sum_accX = 0.0;
    sum_accY = 0.0;
    sum_accZ = 0.0;

    sum_gyroX = 0.0;
    sum_gyroY = 0.0;
    sum_gyroZ = 0.0;  
    
    for(int i = 4 ; i >= 0 ; i--){
      lcd.setCursor(0, 70);
      lcd.printf("Start Calb %d sec",i);
      delay(1000);
    }
      
    for(int i = 0 ; i < CalbNum ; i++){
        M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
        M5.IMU.getAccelData(&accX,&accY,&accZ);

        sum_accX += accX;
        sum_accY += accY;
        sum_accZ += accZ;

        sum_gyroX += gyroX;
        sum_gyroY += gyroY;
        sum_gyroZ += gyroZ;
        delay(10);
    }

    calb_accX = (float)(sum_accX/CalbNum);
    calb_accY = (float)(sum_accY/CalbNum);
    calb_accZ = (float)(sum_accZ/CalbNum) - 1.0F;

    calb_gyroX = (float)(sum_gyroX/CalbNum);
    calb_gyroY = (float)(sum_gyroY/CalbNum);
    calb_gyroZ = (float)(sum_gyroZ/CalbNum);

    preferences.begin("imu_calb_data", false);
    preferences.putFloat("ax", calb_accX);
    preferences.putFloat("ay", calb_accY);
    preferences.putFloat("az", calb_accX);
    preferences.putFloat("gx", calb_gyroX);
    preferences.putFloat("gy", calb_gyroY);
    preferences.putFloat("gz", calb_gyroZ);
    preferences.end();
    
    lcd.setCursor(0, 70);
    lcd.printf("End Calb");
}
