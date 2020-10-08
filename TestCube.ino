//#include <M5StickC.h>
#include "M5StickCPlus.h"
#include <SPIFFS.h> 
#include <LovyanGFX.hpp>
#include <Preferences.h>
#include "MadgwickAHRS.h"

Preferences preferences;
Madgwick *filter = new Madgwick();

static LGFX lcd;                 // LGFXのインスタンスを作成。
static LGFX_Sprite sprite(&lcd); // スプライトを使う場合はLGFX_Spriteのインスタンスを作成。

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

void rotate_cube_xyz( float roll, float pitch, float yaw){
  uint8_t i;

  float DistanceCamera = 300;
  float DistanceScreen = 550;

  for (i = 0; i < 8; i++){
    
    float x = (cos(yaw) * cos(pitch) ) * cubef[i].x
            + (cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll) ) * cubef[i].y
            + (cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll) ) * cubef[i].z;
    float y = (sin(yaw) * cos(pitch)) * cubef[i].x
            + (sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll)) * cubef[i].y
            + (sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll)) * cubef[i].z;
    float z = (-sin(pitch)) * cubef[i].x
            + (cos(pitch) * sin(roll) ) * cubef[i].y
            + (cos(pitch) * cos(roll) ) * cubef[i].z;

    float workx = (x * DistanceCamera) / (z + DistanceCamera + DistanceScreen);
    float worky = (y * DistanceCamera) / (z + DistanceCamera + DistanceScreen);
    
    cubef2[i].x = workx + 80;
    cubef2[i].y = worky + 40;
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

// 最初に初期化関数を呼び出します。
  lcd.init();


// 回転方向を 0～3 の4方向から設定します。(4～7を使用すると上下反転になります。)
  lcd.setRotation(1);


// バックライトの輝度を 0～255 の範囲で設定します。
  lcd.setBrightness(255); // の範囲で設定


// 必要に応じてカラーモードを設定します。（初期値は16）
// 16の方がSPI通信量が少なく高速に動作しますが、赤と青の諧調が5bitになります。
// 24の方がSPI通信量が多くなりますが、諧調表現が綺麗になります。
//lcd.setColorDepth(16);  // RGB565の16ビットに設定
  lcd.setColorDepth(24);  // RGB888の24ビットに設定(表示される色数はパネル性能によりRGB666の18ビットになります)


// clearまたはfillScreenで画面全体を塗り潰します。
// どちらも同じ動作をしますが、clearは引数を省略でき、その場合は黒で塗り潰します。
  lcd.fillScreen(0);  // 黒で塗り潰し
  lcd.clear(0xFFFF);  // 白で塗り潰し
  lcd.clear();        // 黒で塗り潰し
}

unsigned int pre_time =0;

void loop(void){
  M5.update();

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

  pitch = filter->getPitch()*180.0F/PI;
  roll = filter->getRoll()*180.0F/PI;
  yaw = filter->getYaw()*180.0F/PI;

  //Serial.printf("%f,%f,%f,\r\n",pitch,roll,yaw);
  
  rotate_cube_xyz(filter->getRoll(),filter->getPitch(),filter->getYaw());
  int ws = lcd.width();
  int hs = lcd.height();
  
  sprite.createSprite(ws,hs);
  //sprite.drawRect(0, 0, ws,hs, 0xF000);
  for (int i = 0; i < 8; i++)
  {
    sprite.drawRect( (int)cubef2[i].x-2, (int)cubef2[i].y-2, 4, 4 , 0xF000);
    //Serial.printf("%d,%f,%f,\r\n",i,cubef2[i].x, cubef2[i].y); 
  }

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

  for (int i = 0; i < 6; i++)
  {
    int ii = ss[i];
    
    uint16_t color = lcd.color565((uint8_t)((( ii + 1) &  1)      * 255),
                              (uint8_t)((((ii + 1) >> 1) & 1) * 255),
                              (uint8_t)((((ii + 1) >> 2) & 1) * 255));
    sprite.fillTriangle(    cubef2[s[ii].p[0]].x, cubef2[s[ii].p[0]].y,
                            cubef2[s[ii].p[1]].x, cubef2[s[ii].p[1]].y,
                            cubef2[s[ii].p[2]].x, cubef2[s[ii].p[2]].y,
                            color);
    sprite.fillTriangle(    cubef2[s[ii].p[2]].x, cubef2[s[ii].p[2]].y,
                            cubef2[s[ii].p[3]].x, cubef2[s[ii].p[3]].y,
                            cubef2[s[ii].p[0]].x, cubef2[s[ii].p[0]].y,
                            color);
  }
  sprite.pushSprite(0, 0);

  sprite.deleteSprite();

  int deftime = millis() - pre_time;
  while(deftime < 10)
  {
    delay(1);
    deftime = millis() - pre_time;
  }
  pre_time = millis();
  lcd.setCursor(0, 70);
  lcd.printf("%5d",deftime);
  
  if(M5.BtnA.isPressed()){
    Serial.println("Calibration Start");
    getCalibrationVal();
  }

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
