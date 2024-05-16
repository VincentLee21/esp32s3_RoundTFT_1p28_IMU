
#include "Arduino.h"
#include <TFT_eSPI.h>
#include <SPI.h>
#include <Ticker.h>
#include "DEV_Debug.h"
#include <CST816S.h>
#include <FIR.h>
#include "QMI8658.h"
#include "DEV_Config.h"

#ifdef ESP_IDF_VERSION_MAJOR // IDF 4+
#if CONFIG_IDF_TARGET_ESP32 // ESP32/PICO-D4
#include "esp32/rom/rtc.h"
#elif CONFIG_IDF_TARGET_ESP32S2
#include "esp32s2/rom/rtc.h"
#elif CONFIG_IDF_TARGET_ESP32C3
#include "esp32c3/rom/rtc.h"
#elif CONFIG_IDF_TARGET_ESP32S3
#include "esp32s3/rom/rtc.h"
#else
#error Target CONFIG_IDF_TARGET is not supported
#endif
#else // ESP32 Before IDF 4.0
#include "rom/rtc.h"
#endif

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */

#define QMI_ADDRESS 0x6B                  // Define QMI8658C I2C address
#define QMI8658C_I2C_FREQUENCY 400000
#define IMU_GPIO_INT0 4
#define IMU_GPIO_INT1 3

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite img = TFT_eSprite(&tft);

// Qmi8658c qmi8658c(QMI_ADDRESS, QMI8658C_I2C_FREQUENCY, false);

// qmi8658_result_t qmi8658_result;
// qmi_data_t IMU_data;
float fXGyroOff = 0.0, fYGyroOff = 0.0, fZGyroOff = 0.0;
float fXAccOff = 0.0, fYAccOff = 0.0, fZAccOff = 0.0;
float fGX = 0.0, fGY = 0.0, fGZ = 0.0;

// float fXAcc = 0.0, fYAcc = 0.0;
volatile float fIMU_XAcc = 0.0;
volatile float fIMU_YAcc = 0.0;
const int iIMU_sample_delayUS = 1000;
float acc[3], gyro[3];
unsigned int tim_count = 0;
uint16_t result;
// Ticker ticker;

volatile bool bIMU_Update = false;

void TaskRead_IMU(void *pvParameters);

// void tcr10s() {
//   // Serial.printf("SRAM fee size: %d\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
//   Debug("Total heap: %d\n", ESP.getHeapSize());
//   Debug("Free heap: %d\n", ESP.getFreeHeap());
//   Debug("Total PSRAM: %d\n", ESP.getPsramSize());
//   Debug("Free PSRAM: %d\n\n\n", ESP.getFreePsram());
// }

void setup() {
  // put your setup code here, to run once:
  Serial.begin( 115200);
  Debug("\n\n\nESP32-S3 RoundTFT 1.28\n");

  // ticker.attach( 10, tcr10s);

  if( psramInit()) {
    Debug("PSRAM is correctly initialized\n");
  } else {
    Debug("PSRAM not available\n");
  }

  Debug( "Init LCD\n");
  // pinMode(2, OUTPUT);
  // digitalWrite(2, HIGH);
  ledcSetup(0, 500, 8);
  ledcAttachPin(2, 0);
  ledcWrite(0, 255);

  if (DEV_Module_Init() != 0)
    Serial.println("GPIO Init Fail!");
  else
    Serial.println("GPIO Init successful!");

  // Setup LCD
  tft.init();
  tft.setRotation(0);
  img.createSprite(240, 240);
  img.setTextColor(TFT_BLACK, TFT_WHITE, true);
  img.fillScreen(TFT_WHITE);
  // img.pushSprite(0, 0);
  Debug( "Init LCD done\n");


  img.fillRect( 0, 0, 100, 100, TFT_GREEN);
  // img.drawString("calibration run", 60, 60, 2);

  QMI8658_init();
  IMU_calibrate();

  // img.drawString("calibration done", 60, 80, 2);
  String str = String("XAccOff ") + String(fXAccOff,2);
  img.drawString(str, 60, 100, 2);
  str = String("YAccOff ") + String(fYAccOff,2);
  img.drawString(str, 60, 120, 2);
  str = String("ZAccOff ") + String(fZAccOff,2);
  img.drawString(str, 60, 140, 2);

  xTaskCreate(
    TaskRead_IMU
    ,  "Task IMU" // A name just for humans
    ,  4096        // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
    ,  (void*) &iIMU_sample_delayUS // Task parameter which can modify the task behavior. This must be passed as pointer to void.
    ,  1  // Priority
    ,  NULL // Task handle is not used here - simply pass NULL
    );

  img.pushSprite(0, 0);
}

void IMU_calibrate( void) {
  int i = 0;
  int iMax = 1000;
  float fXGyroSum = 0.0, fYGyroSum = 0.0, fZGyroSum = 0.0;
  float fXAccSum = 0.0, fYAccSum = 0.0, fZAccSum = 0.0;

  for( i = 0; i < iMax; i++) {
    QMI8658_read_xyz(acc, gyro, &tim_count);
    fXAccSum += acc[0];
    fYAccSum += acc[1];
    fZAccSum += acc[2];
    // Debug( "accX %3.2f\n", acc[0]);
    // Debug( "accY %3.2f\n", acc[1]);
    // Debug( "accZ %3.2f\n", acc[2]);

    fXGyroSum += gyro[0];
    fYGyroSum += gyro[1];
    fZGyroSum += gyro[2];
    // usleep(1000);
  }
  fXAccOff = fXAccSum * 0.001;
  fYAccOff = fYAccSum * 0.001;
  fZAccOff = fZAccSum * 0.001;

  fXGyroOff = fXGyroSum * 0.001;
  fYGyroOff = fYGyroSum * 0.001;
  fZGyroOff = fZGyroSum * 0.001;
}
/*
float FIR_coeff[63] = {
  -0.000253459943825063,
  0.000000000000000000,
  0.000302830339756015,
  0.000674517240403855,
  0.001116900068473316,
  0.001602074199064700,
  0.002065262319798868,
  0.002405258635128460,
  0.002493904450069028,
  0.002194520824768990,
  0.001387538282060154,
  -0.000000000000000001,
  -0.001965553032366435,
  -0.004407851448190564,
  -0.007114138083339106,
  -0.009761681719536000,
  -0.011935650779369327,
  -0.013163227073164923,
  -0.012961439782744198,
  -0.010893952772232074,
  -0.006630287645948816,
  0.000000000000000003,
  0.008965663696416900,
  0.020011059275436848,
  0.032656834152381096,
  0.046225759097354151,
  0.059892372368256709,
  0.072752056320232072,
  0.083902556849476798,
  0.092529150734204355,
  0.097983928188092970,
  0.099850110478682322,
  0.097983928188092970,
  0.092529150734204355,
  0.083902556849476798,
  0.072752056320232072,
  0.059892372368256723,
  0.046225759097354158,
  0.032656834152381096,
  0.020011059275436852,
  0.008965663696416900,
  0.000000000000000003,
  -0.006630287645948815,
  -0.010893952772232075,
  -0.012961439782744206,
  -0.013163227073164927,
  -0.011935650779369335,
  -0.009761681719535996,
  -0.007114138083339107,
  -0.004407851448190566,
  -0.001965553032366435,
  -0.000000000000000001,
  0.001387538282060154,
  0.002194520824768992,
  0.002493904450069028,
  0.002405258635128459,
  0.002065262319798868,
  0.001602074199064702,
  0.001116900068473316,
  0.000674517240403855,
  0.000302830339756015,
  0.000000000000000000,
  -0.000253459943825063
};
*/
// SR 1Khz, CutOff 30Hz, BW 30Hz
float FIR_coeff[] = {
    0.000000000000000000,
    0.000000616770332650,
    0.000002549635021818,
    0.000005728267352965,
    0.000009811799467181,
    0.000014189975844758,
    0.000018004467511079,
    0.000020191228569673,
    0.000019543929464478,
    0.000014797476454717,
    0.000004729430117159,
    -0.000011724203757601,
    -0.000035343765148512,
    -0.000066497702390987,
    -0.000105019035717396,
    -0.000150095373347144,
    -0.000200182105041612,
    -0.000252948690997799,
    -0.000305267656847485,
    -0.000353254902664462,
    -0.000392368175556237,
    -0.000417568028534577,
    -0.000423542334691192,
    -0.000404991546843302,
    -0.000356967551097851,
    -0.000275254377692488,
    -0.000156774472303968,
    0.000000000000000000,
    0.000194654924633415,
    0.000424487791876528,
    0.000684235077575752,
    0.000965901297550738,
    0.001258715678991929,
    0.001549238681932089,
    0.001821635094451181,
    0.002058123236821410,
    0.002239601179373175,
    0.002346441156621078,
    0.002359432998029152,
    0.002260846928557149,
    0.002035576112038472,
    0.001672310435934203,
    0.001164685876561792,
    0.000512348902781982,
    -0.000278126746694626,
    -0.001192532579742413,
    -0.002208584496427941,
    -0.003295798189562315,
    -0.004415719480531432,
    -0.005522534420984364,
    -0.006564067751327779,
    -0.007483160357261186,
    -0.008219397522783304,
    -0.008711140958791809,
    -0.008897799761500307,
    -0.008722259599004235,
    -0.008133376462415590,
    -0.007088432072408103,
    -0.005555443172896996,
    -0.003515216945584909,
    -0.000963049887295778,
    0.002090022300929761,
    0.005616501227505299,
    0.009572286802491682,
    0.013897157909382193,
    0.018515882520286211,
    0.023339940154371273,
    0.028269812532764414,
    0.033197772344019119,
    0.038011076473507897,
    0.042595450052711072,
    0.046838732286456339,
    0.050634545048524715,
    0.053885841277664041,
    0.056508192550203513,
    0.058432683840440432,
    0.059608298085711983,
    0.060003693134018272,
    0.059608298085711983,
    0.058432683840440432,
    0.056508192550203513,
    0.053885841277664048,
    0.050634545048524729,
    0.046838732286456339,
    0.042595450052711072,
    0.038011076473507897,
    0.033197772344019126,
    0.028269812532764421,
    0.023339940154371283,
    0.018515882520286214,
    0.013897157909382195,
    0.009572286802491682,
    0.005616501227505300,
    0.002090022300929761,
    -0.000963049887295778,
    -0.003515216945584909,
    -0.005555443172896996,
    -0.007088432072408106,
    -0.008133376462415593,
    -0.008722259599004236,
    -0.008897799761500307,
    -0.008711140958791809,
    -0.008219397522783306,
    -0.007483160357261186,
    -0.006564067751327774,
    -0.005522534420984365,
    -0.004415719480531433,
    -0.003295798189562314,
    -0.002208584496427943,
    -0.001192532579742414,
    -0.000278126746694626,
    0.000512348902781982,
    0.001164685876561792,
    0.001672310435934205,
    0.002035576112038472,
    0.002260846928557147,
    0.002359432998029152,
    0.002346441156621079,
    0.002239601179373175,
    0.002058123236821412,
    0.001821635094451183,
    0.001549238681932089,
    0.001258715678991929,
    0.000965901297550738,
    0.000684235077575753,
    0.000424487791876528,
    0.000194654924633415,
    0.000000000000000000,
    -0.000156774472303968,
    -0.000275254377692488,
    -0.000356967551097851,
    -0.000404991546843302,
    -0.000423542334691193,
    -0.000417568028534576,
    -0.000392368175556237,
    -0.000353254902664462,
    -0.000305267656847485,
    -0.000252948690997799,
    -0.000200182105041612,
    -0.000150095373347145,
    -0.000105019035717397,
    -0.000066497702390987,
    -0.000035343765148512,
    -0.000011724203757601,
    0.000004729430117159,
    0.000014797476454717,
    0.000019543929464478,
    0.000020191228569673,
    0.000018004467511079,
    0.000014189975844758,
    0.000009811799467181,
    0.000005728267352965,
    0.000002549635021818,
    0.000000616770332650,
    0.000000000000000000
};
FIR<float, 155> firX;
FIR<float, 155> firY;
FIR<float, 155> firZ;

void TaskRead_IMU(void *pvParameters) {
  const float M_PI_INV  = (1/M_PI);
  const int iNoofSample = 155;
  float fTmpAX, fTmpAY, fTmpAZ;
  float fTmpGyroX, fTmpGyroY, fTmpGyroZ;
  float fTmpAccX[iNoofSample];
  float fTmpAccY[iNoofSample];
  float fTmpAccZ[iNoofSample];
  uint32_t iDelayUS = *((uint32_t*) pvParameters);
  int i;

  firX.setFilterCoeffs(FIR_coeff);
  firY.setFilterCoeffs(FIR_coeff);
  firZ.setFilterCoeffs(FIR_coeff);

  while( 1) {
    for( i = 0; i < (iNoofSample -1); i++) // shift 1 sample
      fTmpAccX[i+1] = fTmpAccX[i];
    for( i = 0; i < (iNoofSample -1); i++) // shift 1 sample
      fTmpAccY[i+1] = fTmpAccY[i];
    for( i = 0; i < (iNoofSample -1); i++) // shift 1 sample
      fTmpAccZ[i+1] = fTmpAccZ[i];

    QMI8658_read_xyz(acc, gyro, &tim_count);
    fTmpAccX[0] = acc[0] - fXAccOff;
    fTmpAccY[0] = acc[1] - fYAccOff;
    fTmpAccZ[0] = acc[2];

    fTmpGyroX = gyro[0] - fXGyroOff;
    fTmpGyroY = gyro[1] - fYGyroOff;
    fTmpGyroZ = gyro[2] - fZGyroOff;

    // fTmpAX = acc[0];
    // fTmpAY = acc[1];
    // fTmpAZ = acc[2];
    // FIR filter
    for( i = 0; i < iNoofSample; i++) {
      fTmpAX = firX.processReading(fTmpAccX[i]);
      fTmpAY = firY.processReading(fTmpAccY[i]);
      fTmpAZ = firZ.processReading(fTmpAccZ[i]);
    }

    fIMU_XAcc = atan2(fTmpAX, sqrt(pow(fTmpAY, 2) + pow(fTmpAZ, 2))) * 180.0 * M_PI_INV;
    fIMU_YAcc = atan2(fTmpAY, sqrt(pow(fTmpAX, 2) + pow(fTmpAZ, 2))) * 180.0 * M_PI_INV;

    fGX = fGX + (fTmpGyroX * 0.001);
    fGY = fGY - (fTmpGyroY * 0.001);
    fGZ = fGZ + (fTmpGyroZ * 0.001);

    fGX = fGX * 0.96 + fIMU_XAcc * 0.04;
    fGY = fGY * 0.96 + fIMU_YAcc * 0.04;

    bIMU_Update = true;
    usleep(iDelayUS);
  }
}

void loop() {
  // put your main code here, to run repeatedly:

  if( bIMU_Update) {
    bIMU_Update = false;
    // Print angle data
    Serial.printf("aX: %3.3f , ", fGX);
    Serial.printf("aY: %3.3f ,", fGY);
    Serial.printf("aZ: %3.3f\r\n", fGZ);
  }

  // Print gyroscope data
  // Serial.printf("gyro_x: %3.3f , ", gyro[0]);
  // Serial.printf("gyro_y: %3.3f , ", gyro[1]);
  // Serial.printf("gyro_z: %3.3f\r\n", gyro[2]);
  usleep(100);
}