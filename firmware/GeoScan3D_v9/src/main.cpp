// =============================================================
// GeoScan3D v9.3 — نظام مسح أرضي ثلاثي الأبعاد
// M5Stack CoreS3 + ADS1256 (polling) + ثنائي FG-3C
// v8.5: مقياس مغناطيسية LIS3MDL على PORT.A (I2C 0x1C/0x1E) — بوصلة مُعوَّضة بالميل
// v8.5.1: تشخيص I2C عند الإقلاع + استرداد تلقائي بتبديل SDA/SCL (PORT.A)
// v9.0: حذف وحدة القرص الدوّار (I2C 0x40) كلياً — العتبة عبر BLE (THR) فقط،
//       وشريط لمس سفلي بزرين فقط: MODE (SCAN↔LIVE) وPULSE (قصيرة=STEP، مطوّلة=SKIP)
// v9.1: اسم BLE أصبح «GeoScan3D» (بدل A2)؛ حزمة BLE أصبحت 34 بايت —
//       أُضيف float yaw (الاتجاه بالدرجات) قبل crc16 مباشرة.
//       التطبيق يقبل 30 بايت (فيرموير قديم) و34 بايت معاً.
// v9.3: شريط لمس سفلي بثلاثة أزرار [MODE][CAL/SENS][PULSE] + مخطط حي + حزمة نبضة قياسية
//       (نفس علم أمر BLE "CAL") مع ومضة تأكيد بصرية قصيرة.
// v9.5: اسم جهاز BLE أصبح «CH05» — يظهر في مسح البلوتوث على الهاتف/الحاسوب باسم
//       CH05 بدل GeoScan3D، تسهيلاً للربط مع تطبيقات/سكربتات مُعدّة مسبقاً لاسم
//       وحدة CH05 الخارجية. الهوية الفعلية للبروتوكول (UUIDs، تنسيق الحزمة،
//       أوامر BLE) لم تتغيّر — هذا تغيير اسم العرض (advertising name) فقط.
//       تنبيه هام: شريحة ESP32-S3 (M5Stack CoreS3) لا تملك راديو Bluetooth
//       Classic إطلاقاً، لذا لا يمكن فعلياً محاكاة CH05/HC-05 كمنفذ COM تسلسلي
//       كلاسيكي (SPP) على ويندوز بمجرد تغيير الاسم؛ الاتصال هنا يبقى عبر BLE
//       (Nordic UART Service) كما في v9.4 — أي تطبيق عميل (مثل Visualizer 3D)
//       يجب أن يتصل عبر BLE GATT لا عبر منفذ COM كلاسيكي.
// ملف أحادي متكامل حسب SPEC.md (§1..§13)
// =============================================================

#include <Arduino.h>
#include <M5CoreS3.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <LittleFS.h>        // التخزين البديل على الفلاش الداخلي (بلا بطاقة)
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <math.h>
#include <string.h>
#include <strings.h>   // strncasecmp
#include <stdio.h>

// ===== CONFIG ================================================
// كل الثوابت في مكان واحد (SPEC §13)

// ---- خريطة الأطراف (ثابتة — SPEC §2) ----
#define PIN_ADC_SCLK   17      // ساعة SPI للمحول ADS1256 (Port.C)
#define PIN_ADC_MISO   18      // ADS1256 DOUT
#define PIN_ADC_MOSI   8       // ADS1256 DIN (Port.B)
#define PIN_ADC_CS     9       // ADS1256 CS
#define PIN_SD_CS      4       // microSD الداخلي في CoreS3
#define PIN_SD_SCK     36      // ناقل SD/LCD المشترك
#define PIN_SD_MISO    35
#define PIN_SD_MOSI    37

// ---- مشغل ADS1256 (SPEC §3) ----
#define ADC_SPI_HOST   HSPI              // ناقل مستقل للمحول
#define ADC_SPI_HZ     1000000           // 1 MHz
#define ADS_VREF       2.5f              // جهد المرجع
#define ADS_PGA        1.0f              // PGA=1
#define DIVIDER_GAIN   2.0f              // تعويض مجهز الجهد 10k/10k: realV = adcV * 2.0
#define SAT_RAW_LIMIT  0x7FFF00          // حد التشبع (حارس التشبع)
#define MAINS_50HZ     0                 // 1 = 60SPS لرفض تداخل الشبكة 50/60Hz

// أوامر ADS1256
#define ADS_CMD_WAKEUP   0x00
#define ADS_CMD_RDATA    0x01
#define ADS_CMD_SDATAC   0x0F
#define ADS_CMD_SELFCAL  0xF0
#define ADS_CMD_SYNC     0xFC
#define ADS_CMD_RESET    0xFE
#define ADS_CMD_WREG     0x50
// عناوين الريجسترات
#define ADS_REG_STATUS   0x00
#define ADS_REG_MUX      0x01
#define ADS_REG_ADCON    0x02
#define ADS_REG_DRATE    0x03
// قيم التهيئة: STATUS=0x04 (MSB أولاً، Auto-Cal ON، Buffer OFF — Buffer ON يقتطع >1.3V مع AVDD=3.3V)
#define ADS_STATUS_VAL   0x04
#define ADS_ADCON_VAL    0x00            // PGA=1، clock out off، sensor detect off

#if MAINS_50HZ
  #define ADS_DRATE_VAL  0x72            // 60 SPS — رفض طنين الشبكة
  #define ADS_SETTLE_MS  20              // زمن الاستقرار (SPEC §3)
#else
  #define ADS_DRATE_VAL  0x82            // 100 SPS الافتراضي
  #define ADS_SETTLE_MS  12
#endif

// ---- تحويل FG-3C (SPEC §4) ----
#define FG_VMIN            0.5f
#define FG_VMAX            3.0f
#define FG_RANGE_UT        100.0f                                  // ±100 µT
#define FG_SCALE_UT_PER_V  ((2.0f*FG_RANGE_UT)/(FG_VMAX-FG_VMIN)) // 80 µT/V
#define FG_VOFFSET         ((FG_VMIN+FG_VMAX)/2.0f)               // 1.75 V

// ---- سلسلة المعالجة (SPEC §5) ----
#define EMA_ALPHA        0.25f           // معامل EMA
#define GRAD_EMA_ALPHA   0.30f           // تنعيم gradKf — للعرض في الواجهة وBLE فقط (لا يدخل في anomaly/depth/detect)
#define KALMAN_Q         1e-3f           // ضجيج العملية
#define KALMAN_R         0.5f            // ضجيج القياس
#define K_DEPTH          40.0f           // ثابت تقدير العمق
#define DEPTH_MAX_CM     500.0f          // سقف العمق
#define DETECT_THRESH_UT 5.0f            // عتبة الكشف µT (الافتراضية عند الإقلاع)
#define THRESH_MIN_UT    1.0f            // حدود ضبط العتبة عبر أمر BLE "THR,<float>"
#define THRESH_MAX_UT    50.0f
#define THRESH_HINT_MS   1500            // مدة تلميح «THRESH: x.x uT» على الشاشة
#define VAR_MAX          25.0f           // أقصى تباين مقبول للخلية (µT²)

// ---- محرك المسح الشبكي (SPEC §6) ----
#define GRID_MAX         32              // أبعاد قصوى 32×32
#define GRID_DEFAULT_W   8
#define GRID_DEFAULT_H   8
#define CELL_DWELL_MS    3000            // زمن المكوث لكل خلية (الوضع التلقائي القديم)
#define SCAN_STEP_MODE   1               // v8.3: 1 = نبضات خطوية (STEP عبر BLE/لمس)، 0 = dwell تلقائي
#define STEP_PULSE_MS    2500            // مدة تجميع النبضة الواحدة (غير حاجب — مؤقّت millis)
#define MODE_LIVE        0
#define MODE_SCAN        1
#define MODE_REVIEW      2

// ---- مسجل SD (SPEC §7) ----
#define LOG_QUEUE_LEN    48              // عمق طابور الأسطر
// v8.2: أسوأ حالة للسطر بعد إضافة tilt_deg,yaw_deg:
// t(10)+m(1)+x,y(≤4×2)+s1,s2(≤7×2)+grad(8)+anom(8)+depth(6)+q(3)+det(1)
// +tilt(6)+yaw(6)+فواصل(12)+\n(1) ≈ 84 < 112 — لا حاجة لرفع الحجم
#define LOG_LINE_LEN     112             // حجم السطر char[112]
#define LOG_LIVE_MS      500             // سطور LIVE بمعدل 2Hz
#define LOG_FLUSH_MS     2000            // flush كل ثانيتين أو عند اكتمال خلية
#define SD_DIR           "/geoscan"
#define CSV_HEADER       "t_ms,mode,x,y,s1_uT,s2_uT,grad_uT,anomaly_uT,depth_cm,quality,detect,tilt_deg,yaw_deg"

// ---- BLE (SPEC §8) ----
#define BLE_DEVICE_NAME  "CH05"          // v9.5: اسم العرض عبر BLE أصبح "CH05" (بدل GeoScan3D)
                                          // (لن يظهر في مسح Bluetooth الكلاسيكي — قيد شريحة S3)
#define BLE_MTU          64
#define BLE_SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_TX_UUID      "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  // notify — حزمة حية
#define BLE_RX_UUID      "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  // write — أوامر
#define BLE_LIVE_MS      200             // 5 Hz في LIVE
#define BLE_SCAN_MS      500             // 2 Hz في SCAN/REVIEW
#define BLE_CMD_MAX      32              // حد أقصى لطول الأمر (فحص الطول)
#define BLE_DUMP_CHUNK   100             // ≤100 بايت لكل notify أثناء DUMP

// ---- v9.4: بديل CH05/HC-05 عبر BLE (لا يوجد Classic BT/SPP على شريحة ESP32-S3) ----
// CoreS3 مبني على ESP32-S3 وهذه الشريحة لا تملك راديو Bluetooth Classic إطلاقاً؛
// لذا لا يمكن تشغيل بروتوكول SPP الكلاسيكي (كما في CH05/HC-05) على هذا العتاد،
// وهذا قيد فيزيائي في الشريحة نفسه وليس قيد برمجة. البديل العملي المطبَّق هنا هو:
// إرسال سطر نصي عادي (ASCII, \r\n) عبر نفس خدمة BLE UART (Nordic UART Service) —
// أي تطبيق يدعم "BLE Serial" (مثل Serial Bluetooth Terminal على أندرويد، أو أي
// جسر BLE↔COM على الحاسوب) يستقبله ويعرضه بالضبط كما لو كان قادماً من CH05 سيريال حقيقي.
#define TEXT_STREAM_ENABLE   1           // 1 = تفعيل بث سطر ASCII دوري بجانب الحزمة الثنائية
#define TEXT_STREAM_LIVE_MS  200         // 5 Hz في LIVE
#define TEXT_STREAM_SCAN_MS  500         // 2 Hz في SCAN/REVIEW
volatile bool g_textStreamOn = true;     // قابل للتبديل بأمر BLE "TXT,0" / "TXT,1"

// ---- المعايرة (SPEC §9) ----
#define CAL_TIME_MS      5000            // متوسط 5 ثوانٍ
#define PREFS_NS         "geoscan"       // namespace في NVS

// ---- الواجهة (SPEC §10) ----
#define UI_DRAW_MS       100             // تخنيق الرسم ~10Hz
#define UI_FOOTER_Y      208             // بداية شريط الأزرار
#define UI_HEAT_Y        24              // بداية منطقة الخريطة الحرارية
#define UI_HEAT_H        176             // ارتفاعها
#define HEAT_FULL_UT     20.0f           // تطبيع الخريطة الحرارية
#define BAR_FULL_UT      50.0f           // تطبيع شريط التدرج
#define LONG_PRESS_MS    600             // عتبة الضغطة المطوّلة (v9.0: تخطّي الخلية بـ PULSE)
#define POPUP_MS         3000            // مدة نافذة القيم في REVIEW
// v9.3: ومضة تأكيد زر CAL في التذييل (لم يعد هناك زر علوي)
#define CAL_FLASH_MS     600             // مدة ومضة التأكيد البصرية بعد الضغط

// ---- مقياس المغناطيسية LIS3MDL على PORT.A (I2C 0x1C/0x1E) — v8.5 ----
// الجهاز الوحيد على M5.Ex_I2C في v9.0 (حُذفت وحدة القرص الدوّار)
#define MAG_I2C_FREQ        100000       // سقف LIS3MDL 100kHz — كل مرور PORT.A عنده
#define MAG_ADDR_PRIMARY    0x1C         // الافتراضي (SDO منخفض)
#define MAG_ADDR_FALLBACK   0x1E         // (SDO مرتفع)
#define MAG_REG_WHO_AM_I    0x0F         // يجب أن يقرأ 0x3D
#define MAG_WHO_AM_I_VAL    0x3D
#define MAG_REG_CTRL1       0x20
#define MAG_REG_CTRL2       0x21
#define MAG_REG_CTRL3       0x22
#define MAG_REG_CTRL4       0x23
#define MAG_CTRL1_VAL       0x70         // UHP XY + ODR 80Hz (حساس الحرارة معطّل — bit7=0)
#define MAG_CTRL2_VAL       0x00         // ±4 gauss
#define MAG_CTRL3_VAL       0x00         // وضع التحويل المستمر
#define MAG_CTRL4_VAL       0x0C         // UHP Z
#define MAG_REG_OUT_X_L     0x28         // 6 بايت x,y,z int16 LE (bit7=1 للزيادة التلقائية)
#define MAG_LSB_PER_GAUSS   6842.0f      // حساسية ±4 gauss
#define MAG_POLL_MS         50           // استطلاع ~20Hz (غير حاجب، من loop فقط)
#define MAG_CAL_MS          15000        // نافذة جمع min/max بعد أمر MAGCAL
#define MAG_FUSE_ALPHA      0.02f        // معامل الترشيح التكميلي gyro↔mag

// ---- تشخيص/استرداد ناقل PORT.A الخارجي (v8.5.1) ----
// CoreS3 PORT.A الافتراضي: SDA=GPIO2, SCL=GPIO1. الكابل الملحوم قد يعكسهما.
#define PIN_PORTA_SDA     2              // SDA افتراضي
#define PIN_PORTA_SCL     1              // SCL افتراضي
#define I2C_SWAP_TRY      1              // 1 = جرّب إعادة التهيئة بأطراف معكوسة عند ناقل فارغ
#define I2C_SCAN_FREQ     100000         // تردد الفحص — مقيّد بـ 100kHz كبقية المرور

// =============================================================
// البنى والبيانات المشتركة
// =============================================================

// بيانات المستشعر المشتركة (تحت xMutex)
struct SensorData {
  float ut1 = 0, ut2 = 0;        // µT بعد Kalman لكل قناة
  float grad = 0;                // kf0 - kf1
  float gradKf = 0;              // grad بعد تنعيم EMA
  float anomaly = 0;             // grad - baseline (SPEC §5 حرفياً)
  float depthCm = 0;             // العمق المقدّر
  uint8_t detect = 0;
  uint8_t quality = 100;         // جودة لحظية (50 عند التشبع)
  bool saturated = false;
  // ---- IMU (BMI270 الداخلي — v8.2) ----
  float pitch = 0, roll = 0;     // درجات، منعّمة بـ EMA
  float tiltDeg = 0;             // مقدار الميل الكلي عن المستوى
  float yawDeg = 0;              // انحراف نسبي مُكامَل من الجيروسكوب [-180,180]
  uint8_t imuOK = 0;             // 1 إذا كان BMI270 متاحاً
};

// خلية مسح (SPEC §6)
struct ScanCell {
  float grad;
  float depthCm;
  uint8_t quality;
  uint8_t filled;
};

// حزمة BLE المضغوطة (SPEC §8) — نفس أسماء حقول المستخدم الحالية
struct __attribute__((packed)) BlePacket {
  uint8_t  header0 = 0xA5, header1 = 0x5A;
  uint32_t timestamp;            // millis()
  float ut1, ut2, grad, gradKf, depthCm;
  uint8_t detect, mode;
  float yaw;                     // v9.1: الاتجاه بالدرجات (قبل CRC مباشرة)
  uint16_t crc16;                // crc16_ccitt على البايتات السابقة
};
// v9.1: الحجم الفعلي = 34 بايت (30 في v9.0 قبل إضافة yaw). التطبيق يقبل
// الطولين: يجرّب CRC على 30 ثم على 34. نحسب CRC على sizeof(BlePacket)-2
// حتى يبقى صحيحاً مع أي تعديل، وMTU=64 يكفي في الحالتين.

// ---- مقابض FreeRTOS ----
SemaphoreHandle_t xMutex = NULL;         // حماية g_sensor + الشبكة
QueueHandle_t     xLogQueue = NULL;      // طابور سطور CSV

// ---- الحالة العامة (أسماء تعكس نمط المستخدم الحالي) ----
SensorData g_sensor;
volatile uint8_t g_mode = MODE_LIVE;

ScanCell g_grid[GRID_MAX][GRID_MAX];     // ~12KB ثابتة (SPEC §6)
static ScanCell s_gridCopy[GRID_MAX][GRID_MAX]; // نسخة للرسم
volatile int g_gridW = GRID_DEFAULT_W;
volatile int g_gridH = GRID_DEFAULT_H;
volatile int g_cellIndex = 0;            // الخلية النشطة (ترتيب أفعواني)
volatile bool g_cellActive = false;
volatile int g_filledCount = 0;
uint32_t g_cellStartMs = 0;
// مجمّعات الخلية الحالية
double   g_cellSumGrad = 0, g_cellSumSq = 0, g_cellSumDepth = 0;
uint32_t g_cellCount = 0;
bool     g_cellSatSeen = false;

// ---- IMU (BMI270 عبر M5.Imu — v8.2) ----
static bool g_imuOK = false;             // يُضبط مرة واحدة في setup بعد M5.begin
#define IMU_PERIOD_MS     50             // فترة قراءة IMU داخل sensorTask
#define IMU_EMA_ALPHA     0.2f           // تنعيم pitch/roll
#define IMU_STILL_DPS     0.5f           // تحتها يُعتبر الجهاز ساكناً (لا تكامل yaw)
#define GYRO_ZOFF_ALPHA   0.001f         // EMA بطيئة لأوفست صفر الجيروسكوب
static float g_yawDeg = 0.0f;            // yaw نسبي مُكامَل [-180,180] (sensorTask فقط)
static float g_gyroZOff = 0.0f;          // أوفست صفر gz المقدّر (sensorTask فقط)
// v8.5 — LIS3MDL: تُكتب من loop (pollMag/magInit)، تُقرأ من sensorTask/drawUI (float ذرّية)
volatile bool  g_magOK = false;          // LIS3MDL متاح
volatile float g_headingDeg = 0.0f;      // الاتجاه المُعوَّض بالميل 0..360
volatile float g_magYawFused = 0.0f;     // yaw مدموج gyro+mag [-180,180]
bool     g_cellTiltBad = false;          // ميل >10° رُصد أثناء تجميع الخلية الحالية
// #define EXT_COMPASS 1  // مستقبلاً: مقياس مغناطيسية I2C على Port.A (G1/G2) للشمال المطلق

volatile float g_baseline = 0;           // خط الأساس المجمّد عند بدء المسح/المعايرة
// v9.0: عتبة الكشف قابلة للضبط من BLE فقط (أمر THR,<float>، محصورة 1..50 µT).
// تُكتب من loop وتُقرأ من sensorTask — float ذرّية القراءة/الكتابة على Xtensa.
volatile float thresh = DETECT_THRESH_UT;
// مؤقّت تلميح العتبة على الشاشة (يُقرأ من drawUI، يُضبط عند أمر THR)
static uint32_t g_threshHintUntil = 0;
volatile bool  g_baselinePending = false;
volatile float g_calOffsetV[2] = {0.0f, 0.0f}; // أوفست المعايرة لكل قناة (فولت) — volatile: تُكتب من loop وتُقرأ من sensorTask

// أعلام الطلبات (نمط g_bCalRequest الحالي)
volatile bool g_bCalRequest = false;
volatile bool g_bCalInProgress = false;
volatile bool g_bFilterReset = false;
volatile bool g_bStartRequest = false;
volatile bool g_bStopRequest = false;
volatile bool g_bSkipCell = false;
volatile bool g_bOpenLogRequest = false;
volatile bool g_bCloseLogRequest = false;
volatile bool g_bSaveGridRequest = false;
volatile bool g_bFlushRequest = false;
volatile bool g_bMagCalRequest = false;    // v8.5: أمر BLE "MAGCAL" — تصفير min/max + نافذة جمع 15s

// ---- نظام النبضات الخطوية (v8.3, SCAN_STEP_MODE=1) ----
volatile bool     g_bStepRequest = false;   // يُضبط من BLE (STEP/PULSE) أو زر اللمس PULSE، يُستهلك في loop
volatile bool     g_pulseActive = false;    // نبضة جارية: تجميع لمدة STEP_PULSE_MS (مؤقّت غير حاجب)
volatile uint32_t g_pulseStartMs = 0;       // بداية النبضة الجارية
static char g_stepResult[64];               // سطر النتيجة «R,x,y,grad,depth,conf,q\n»
static volatile float g_lastStepGrad = 0.0f; // v9.3: تدرج آخر خلية مثبّتة — للحزمة القياسية
volatile bool g_bStepResultReady = false;   // يُضبط تحت xMutex في sensorTask، يُستهلك في pumpStepResult

volatile bool g_sdOK = false;            // = «وحدة تخزين فعّالة» (SD أو LittleFS)
File g_logFile;                          // يُستخدم حصرياً من loggerTask
static bool initSD();                    // تهيئة/استرجاع SD على الناقل المشترك مع الشاشة
static bool sdMountOnce(uint32_t freq);  // تركيب واحد بلا إعادة تهيئة ناقل (لا تعتيم للشاشة)
static bool mountLFS();                  // تركيب LittleFS على الفلاش الداخلي

// ===== طبقة التخزين الموحّدة: SD أولاً ثم LittleFS تلقائياً =====
// مفعّل: تجاوز البطاقة نهائياً (وحدة المستخدم بلا بطاقة عاملة)
// أعد التعليق على السطر إن ركّبت بطاقة SD سليمة يوماً ما
#define FORCE_LITTLEFS 1
enum StorageType : uint8_t { ST_NONE = 0, ST_SD = 1, ST_LFS = 2 };
volatile uint8_t g_storage = ST_NONE;
static fs::FS* g_fs = nullptr;           // يُضبط من loggerTask فقط — كل I/O عبره
static const char* storageName() {
  return g_storage == ST_SD ? "SD" : (g_storage == ST_LFS ? "LFS" : "--");
}

// BLE
BLEServer*         g_bleServer = NULL;
BLECharacteristic* g_bleTxChar = NULL;
volatile bool g_bleConnected = false;
volatile bool g_blePacketReady = false;
volatile bool g_bleSending = false;
volatile bool g_dumpActive = false;
volatile int g_dumpIndex = 0;            // يُصفَّر من رد نداء BLE ويُقرأ/يُزاد من loop
volatile bool g_filesReq = false;        // أمر FILES: بث قائمة ملفات التخزين
volatile bool g_getReq = false;          // أمر GET: بث محتوى ملف
char g_getFileName[40] = {0};

// واجهة: نافذة قيم الخلية في REVIEW
uint32_t g_popupUntil = 0;
int g_popupX = -1, g_popupY = -1;
// v9.2: مؤقّت ومضة تأكيد زر CAL اللمسي (يُضبط في onTap، يُقرأ من drawUI)
static uint32_t g_calFlashUntil = 0;

// =============================================================
// CRC16-CCITT (poly 0x1021, init 0xFFFF) — نمط المستخدم الحالي
// =============================================================
uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  while (len--) {
    crc ^= (uint16_t)(*data++) << 8;
    for (int i = 0; i < 8; i++)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

// =============================================================
// مشغل ADS1256 على مستوى الريجسترات (SPEC §3) — بدون مكتبة خارجية
// =============================================================
SPIClass adcSPI(ADC_SPI_HOST);
static SPISettings adsSPISettings(ADC_SPI_HZ, MSBFIRST, SPI_MODE1);

// كتابة ريجستر (يُستدعى والمعاملة مفتوحة وCS منخفض)
static void adsWriteRegLocked(uint8_t reg, uint8_t val) {
  adcSPI.transfer(ADS_CMD_WREG | reg);
  adcSPI.transfer(0x00);                 // بايت واحد
  adcSPI.transfer(val);
}

void adsInit() {
  pinMode(PIN_ADC_CS, OUTPUT);
  digitalWrite(PIN_ADC_CS, HIGH);        // CS عالٍ قبل البدء
  adcSPI.begin(PIN_ADC_SCLK, PIN_ADC_MISO, PIN_ADC_MOSI, PIN_ADC_CS);

  adcSPI.beginTransaction(adsSPISettings);
  digitalWrite(PIN_ADC_CS, LOW);
  adcSPI.transfer(ADS_CMD_RESET);        // نبضة reset
  delay(1);
  adcSPI.transfer(ADS_CMD_SDATAC);       // إيقاف وضع القراءة المستمرة
  adsWriteRegLocked(ADS_REG_STATUS, ADS_STATUS_VAL);
  adsWriteRegLocked(ADS_REG_ADCON,  ADS_ADCON_VAL);
  adsWriteRegLocked(ADS_REG_DRATE,  ADS_DRATE_VAL);
  adcSPI.transfer(ADS_CMD_SELFCAL);      // معايرة ذاتية
  digitalWrite(PIN_ADC_CS, HIGH);
  adcSPI.endTransaction();
  delay(200);                            // انتظار SELFCAL
}

// قراءة قناة (0 أو 1، single-ended مقابل AINCOM) بأسلوب polling بدون DRDY
int32_t adsReadChannel(uint8_t ch) {
  adcSPI.beginTransaction(adsSPISettings);
  digitalWrite(PIN_ADC_CS, LOW);
  // 1) MUX = (ch<<4)|AINCOM
  adcSPI.transfer(ADS_CMD_WREG | ADS_REG_MUX);
  adcSPI.transfer(0x00);
  adcSPI.transfer((uint8_t)((ch << 4) | 0x08));
  delayMicroseconds(10);
  // 2) SYNC + WAKEUP
  adcSPI.transfer(ADS_CMD_SYNC);
  adcSPI.transfer(ADS_CMD_WAKEUP);
  // 3) زمن استقرار (بديل DRDY)
  delay(ADS_SETTLE_MS);
  // 4) RDATA ثم قراءة 3 بايت MSB→LSB
  adcSPI.transfer(ADS_CMD_RDATA);
  delayMicroseconds(10);
  uint32_t b0 = adcSPI.transfer(0xFF);
  uint32_t b1 = adcSPI.transfer(0xFF);
  uint32_t b2 = adcSPI.transfer(0xFF);
  digitalWrite(PIN_ADC_CS, HIGH);
  adcSPI.endTransaction();
  // 5) sign-extension من 24 إلى 32 بت
  int32_t raw = (int32_t)((b0 << 16) | (b1 << 8) | b2);
  if (raw & 0x00800000) raw |= (int32_t)0xFF000000;
  return raw;
}

// raw → فولت حقيقي على مدخل الحساس (يتضمن تعويض مجهز الجهد ×2.0)
float rawToVolts(int32_t raw) {
  float adcV = (float)raw * (2.0f * ADS_VREF) / (ADS_PGA * 8388608.0f);
  return adcV * DIVIDER_GAIN;
}

// حارس التشبع (SPEC §3/§12.2)
bool isSaturated(int32_t raw) {
  return (raw >= (int32_t)SAT_RAW_LIMIT) || (raw <= -(int32_t)SAT_RAW_LIMIT);
}

// =============================================================
// المرشحات: Median-of-5 → EMA → Kalman 1D (SPEC §5)
// =============================================================
struct Median5 {
  float buf[5];
  uint8_t idx = 0, n = 0;
  void reset() { idx = 0; n = 0; }
  float push(float v) {
    buf[idx] = v; idx = (idx + 1) % 5; if (n < 5) n++;
    float t[5];
    for (uint8_t i = 0; i < n; i++) t[i] = buf[i];
    for (uint8_t i = 1; i < n; i++) {           // insertion sort
      float k = t[i]; int j = (int)i - 1;
      while (j >= 0 && t[j] > k) { t[j + 1] = t[j]; j--; }
      t[j + 1] = k;
    }
    return t[n / 2];
  }
};

struct Kalman1D {
  float x = 0, p = 1.0f;
  bool init = false;
  void reset() { x = 0; p = 1.0f; init = false; }
  float update(float z) {
    if (!init) { x = z; p = 1.0f; init = true; return x; } // init بأول عينة
    p += KALMAN_Q;
    float k = p / (p + KALMAN_R);
    x += k * (z - x);
    p *= (1.0f - k);
    return x;
  }
};

// =============================================================
// محرك المسح الشبكي (SPEC §6)
// =============================================================

// إعلان مسبق: تُستخدم في سطر نتيجة النبضة داخل sensorTask (التعريف في قسم الواجهة)
static float confPercent(float anomaly, float quality);

// إحداثيات الخلية بالترتيب الأفعواني (serpentine)
void cellCoord(int idx, int& x, int& y) {
  y = idx / g_gridW;
  int col = idx % g_gridW;
  x = (y & 1) ? (g_gridW - 1 - col) : col;
}

static void resetCellAccum() {
  g_cellSumGrad = g_cellSumSq = g_cellSumDepth = 0;
  g_cellCount = 0;
  g_cellSatSeen = false;
  g_cellTiltBad = false;                 // v8.2: تصفير علم الميل لكل خلية جديدة
}

// دفع سطر CSV إلى الطابور (غير حاجب — آمن من sensorTask)
static void pushLogLine(const char* line) {
  if (xLogQueue) xQueueSend(xLogQueue, line, 0);
}

static void pushCellLogLine(int x, int y, const ScanCell& c) {
  char line[LOG_LINE_LEN];
  float anomaly = c.grad - g_baseline;
  snprintf(line, sizeof(line), "%lu,%u,%d,%d,%.3f,%.3f,%.3f,%.3f,%.1f,%u,%u,%.1f,%.1f\n",
           (unsigned long)millis(), (unsigned)g_mode, x, y,
           (double)g_sensor.ut1, (double)g_sensor.ut2, (double)c.grad,
           (double)anomaly, (double)c.depthCm,
           (unsigned)c.quality, (unsigned)(fabsf(anomaly) > thresh ? 1 : 0),
           (double)g_sensor.tiltDeg, (double)g_sensor.yawDeg);
  pushLogLine(line);
}

// الانتقال للخلية التالية — يُستدعى وxMutex مأخوذ
static void advanceCellLocked() {
  g_cellIndex++;
  if (g_cellIndex >= g_gridW * g_gridH) {   // اكتملت الشبكة
    g_cellActive = false;
    g_mode = MODE_REVIEW;
    g_bSaveGridRequest = true;              // loggerTask يحفظ grid_NNNN.csv
    g_bCloseLogRequest = true;              // loggerTask: flush نهائي ثم إغلاق scan_NNNN.csv
  } else {
    resetCellAccum();
    g_cellStartMs = millis();
  }
}

// تثبيت نتيجة الخلية — يُستدعى وxMutex مأخوذ
static void commitCellLocked() {
  int x, y; cellCoord(g_cellIndex, x, y);
  ScanCell& c = g_grid[y][x];
  if (g_cellCount > 0) {
    float mean = (float)(g_cellSumGrad / g_cellCount);
    float var  = (float)(g_cellSumSq / g_cellCount) - mean * mean;
    if (var < 0) var = 0;
    c.grad    = mean;
    c.depthCm = (float)(g_cellSumDepth / g_cellCount);
    int q = 100;
    if (g_cellSatSeen) q -= 50;             // تشبع ضمن نافذة الخلية
    if (var > VAR_MAX) q -= 20;             // تباين مرتفع
    if (g_cellTiltBad) q -= 30;             // v8.2: ميل >10° أثناء أي عينة من الخلية
    c.quality = (uint8_t)constrain(q, 0, 100);
  } else {
    c.grad = 0; c.depthCm = 0; c.quality = 0;
  }
  c.filled = 1;
  g_filledCount++;
  pushCellLogLine(x, y, c);                 // سطر اكتمال الخلية
  g_bFlushRequest = true;                   // flush عند اكتمال خلية (SPEC §7)
  advanceCellLocked();
}

void startScan() {
  xSemaphoreTake(xMutex, portMAX_DELAY);
  memset(g_grid, 0, sizeof(g_grid));        // مسح الشبكة
  g_filledCount = 0;
  g_cellIndex = 0;
  g_cellActive = true;
  g_pulseActive = false;                    // v8.3: مسح جديد يبدأ مسلّحاً (ARMED) بلا نبضة
  g_bStepResultReady = false;
  resetCellAccum();
  g_cellStartMs = millis();
  g_baseline = g_sensor.grad;               // تجميد خط الأساس عند البدء (grad من كالمان — SPEC §5/§6)
  g_yawDeg = 0.0f;                          // v8.2: تصفير yaw النسبي عند بدء مسح جديد
  g_sensor.yawDeg = 0.0f;
  g_mode = MODE_SCAN;
  xSemaphoreGive(xMutex);
  g_bOpenLogRequest = true;                 // loggerTask يفتح scan_NNNN.csv
}

void stopScan() {
  xSemaphoreTake(xMutex, portMAX_DELAY);
  g_cellActive = false;
  g_pulseActive = false;                    // v8.3: إيقاف أي نبضة جارية عند الإيقاف
  g_mode = MODE_LIVE;
  xSemaphoreGive(xMutex);
  g_bCloseLogRequest = true;
}

// تخطّي الخلية الحالية بدون تثبيت (ضغطة مطوّلة على زر PULSE)
void skipCell() {
  xSemaphoreTake(xMutex, portMAX_DELAY);
  if (g_mode == MODE_SCAN && g_cellActive) advanceCellLocked();
  xSemaphoreGive(xMutex);
}

// =============================================================
// المعايرة (SPEC §9): متوسط 5 ثوانٍ + حفظ NVS + إعادة تهيئة المرشحات
// =============================================================
void runCalibration() {
  g_bCalInProgress = true;                  // sensorTask يتوقف مؤقتاً
  delay(60);                                // انتظار انتهاء أي قراءة ADC جارية (ناقل مشترك)
  M5.Display.fillRect(0, 90, 320, 60, TFT_BLACK);
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(TFT_YELLOW, TFT_BLACK);
  M5.Display.setCursor(40, 110);
  M5.Display.print("CAL ... 5s");

  double sum[2] = {0, 0};
  uint32_t n = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < CAL_TIME_MS) {
    int32_t r0 = adsReadChannel(0);
    int32_t r1 = adsReadChannel(1);
    if (!isSaturated(r0) && !isSaturated(r1)) {   // استبعاد المشبع
      sum[0] += rawToVolts(r0);
      sum[1] += rawToVolts(r1);
      n++;
    }
    delay(5);
  }
  g_bCalInProgress = false;

  if (n > 4) {
    g_calOffsetV[0] = (float)(sum[0] / n) - FG_VOFFSET;
    g_calOffsetV[1] = (float)(sum[1] / n) - FG_VOFFSET;
    // حفظ في NVS
    Preferences prefs;
    prefs.begin(PREFS_NS, false);
    prefs.putFloat("cal0", g_calOffsetV[0]);
    prefs.putFloat("cal1", g_calOffsetV[1]);
    prefs.end();
    // إعادة تهيئة Kalman/EMA/Median + إعادة حساب خط الأساس
    g_bFilterReset = true;
    xSemaphoreTake(xMutex, portMAX_DELAY);
    g_baselinePending = true;
    g_yawDeg = 0.0f;                       // v8.2: تصفير yaw عند المعايرة
    g_sensor.yawDeg = 0.0f;
    xSemaphoreGive(xMutex);
    Serial.printf("CAL OK: off0=%.4fV off1=%.4fV\n", g_calOffsetV[0], g_calOffsetV[1]);
  } else {
    Serial.println("CAL FAIL: no valid samples");
  }
}

// =============================================================
// sensorTask — core 1, prio 3 (SPEC §11): §3–§5 loop
// لا يوجد أي استدعاء SD هنا إطلاقاً (SPEC §12.4)
// =============================================================
void sensorTask(void* pv) {
  (void)pv;
  adsInit();

  Median5  med[2];
  Kalman1D kf[2];
  float ema[2] = {0, 0};
  bool  emaInit[2] = {false, false};
  float gradEma = 0;
  bool  gradInit = false;
  uint32_t lastLiveLog = 0;
  // ---- حالة IMU المحلية (v8.2) ----
  float imuPitch = 0, imuRoll = 0;         // EMA
  bool  imuInit = false;
  uint32_t lastImuMs = 0;
  uint32_t lastYawMs = 0;                  // لحساب dt للتكامل

  for (;;) {
    if (g_bCalInProgress) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }

    if (g_bFilterReset) {                   // إعادة تهيئة بعد المعايرة
      med[0].reset(); med[1].reset();
      kf[0].reset();  kf[1].reset();
      emaInit[0] = emaInit[1] = gradInit = false;
      g_bFilterReset = false;
    }

    int32_t raw0 = adsReadChannel(0);
    int32_t raw1 = adsReadChannel(1);
    bool sat0 = isSaturated(raw0);
    bool sat1 = isSaturated(raw1);

    // سلسلة المعالجة لكل قناة: median → volts → µT → EMA → Kalman
    float kv[2];
    int32_t raws[2] = {raw0, raw1};
    bool    sats[2] = {sat0, sat1};
    for (int ch = 0; ch < 2; ch++) {
      if (sats[ch]) { kv[ch] = kf[ch].x; continue; }  // استبعاد العينة المشبعة
      float mraw  = med[ch].push((float)raws[ch]);
      float realV = rawToVolts((int32_t)mraw);        // يتضمن DIVIDER_GAIN
      float ut    = (realV - FG_VOFFSET - g_calOffsetV[ch]) * FG_SCALE_UT_PER_V;
      if (!emaInit[ch]) { ema[ch] = ut; emaInit[ch] = true; }
      else ema[ch] = EMA_ALPHA * ut + (1.0f - EMA_ALPHA) * ema[ch];
      kv[ch] = kf[ch].update(ema[ch]);
    }
    float grad = kv[0] - kv[1];
    if (!gradInit) { gradEma = grad; gradInit = true; }
    else gradEma = GRAD_EMA_ALPHA * grad + (1.0f - GRAD_EMA_ALPHA) * gradEma;

    // ---- قراءة IMU كل ~50ms (v8.2) — خارج xMutex (وصول I2C عبر M5Unified) ----
    if (g_imuOK && millis() - lastImuMs >= IMU_PERIOD_MS) {
      lastImuMs = millis();
      float ax, ay, az, gx, gy, gz;
      M5.Imu.getAccel(&ax, &ay, &az);      // g
      M5.Imu.getGyro(&gx, &gy, &gz);       // درجة/ثانية (dps)
      (void)gx; (void)gy;
      float p = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
      float r = atan2f(ay, az) * RAD_TO_DEG;
      if (!imuInit) { imuPitch = p; imuRoll = r; imuInit = true; }
      else {
        imuPitch = IMU_EMA_ALPHA * p + (1.0f - IMU_EMA_ALPHA) * imuPitch;
        imuRoll  = IMU_EMA_ALPHA * r + (1.0f - IMU_EMA_ALPHA) * imuRoll;
      }
      // yaw نسبي: تكامل gz المصحّح بالأوفست، مع تجميد عند السكون
      float dt = (lastYawMs > 0) ? (millis() - lastYawMs) / 1000.0f : 0.0f;
      lastYawMs = millis();
      if (dt > 0.25f) dt = 0.25f;          // حارس ضد فجوات التوقف (معايرة مثلاً)
      float gzC = gz - g_gyroZOff;
      if (fabsf(gz) < IMU_STILL_DPS) {
        // سكون: لا تكامل + تحديث أوفست الصفر بـ EMA بطيئة
        g_gyroZOff = GYRO_ZOFF_ALPHA * gz + (1.0f - GYRO_ZOFF_ALPHA) * g_gyroZOff;
      } else {
        g_yawDeg += gzC * dt;
        while (g_yawDeg >  180.0f) g_yawDeg -= 360.0f;
        while (g_yawDeg < -180.0f) g_yawDeg += 360.0f;
      }
    }
    float tiltNow = sqrtf(imuPitch * imuPitch + imuRoll * imuRoll); // تقريب كافٍ للتسوية

    // ---- تحديث الحالة المشتركة + تجميع الخلية (تحت xMutex) ----
    xSemaphoreTake(xMutex, portMAX_DELAY);
    g_sensor.ut1 = kv[0];
    g_sensor.ut2 = kv[1];
    g_sensor.grad = grad;
    g_sensor.gradKf = gradEma;            // نسخة منعّمة للعرض وBLE فقط
    g_sensor.anomaly = grad - g_baseline; // SPEC §5: anomaly = grad - baseline حرفياً
    float a = fabsf(g_sensor.anomaly);
    // تقدير العمق: K_DEPTH / sqrt(|anomaly|+0.05) محدود بـ 500 سم
    g_sensor.depthCm = fminf(DEPTH_MAX_CM, K_DEPTH / sqrtf(a + 0.05f));
    g_sensor.detect = (a > thresh) ? 1 : 0;   // v9.0: thresh قابلة للضبط من BLE (THR)
    g_sensor.saturated = sat0 || sat1;
    g_sensor.quality = g_sensor.saturated ? 50 : 100;
    g_sensor.pitch = imuPitch;            // v8.2: حقول IMU (أصفار تلقائياً إن !g_imuOK)
    g_sensor.roll = imuRoll;
    g_sensor.tiltDeg = tiltNow;
    g_sensor.yawDeg = g_magOK ? g_magYawFused : g_yawDeg; // v8.5: دمج gyro+mag عند توفر LIS3MDL
    g_sensor.imuOK = g_imuOK ? 1 : 0;
    g_blePacketReady = true;
    if (g_baselinePending) { g_baseline = grad; g_baselinePending = false; }

    if (g_mode == MODE_SCAN && g_cellActive) {
#if SCAN_STEP_MODE
      // v8.3 — نبضات خطوية: لا تجميع/تقدّم إلا أثناء نبضة فعّالة (ARMED بقية الوقت)
      if (g_pulseActive) {
        if (!g_sensor.saturated) {          // العينة المشبعة لا تدخل متوسط الخلية
          g_cellSumGrad  += grad;           // تجميع على grad الحرفي (كالمان) لا على EMA
          g_cellSumSq    += (double)grad * grad;
          g_cellSumDepth += g_sensor.depthCm;
          g_cellCount++;
        } else g_cellSatSeen = true;
        if (g_imuOK && tiltNow > 10.0f) g_cellTiltBad = true; // ميل أثناء عينة النبضة
        if (millis() - g_pulseStartMs >= STEP_PULSE_MS) {     // مؤقّت غير حاجب
          int px, py;
          cellCoord(g_cellIndex, px, py);   // إحداثيات الخلية قبل التقدّم
          commitCellLocked();               // commitCellLocked + advanceCellLocked كما هما
          g_pulseActive = false;            // عودة إلى ARMED (اكتمال الشبكة → REVIEW كما هو)
          // سطر النتيجة من الخلية المُودعة: R,x,y,grad,depth,conf,q
          const ScanCell& cc = g_grid[py][px];
          float conf = confPercent(cc.grad - g_baseline, (float)cc.quality);
          snprintf(g_stepResult, sizeof(g_stepResult), "R,%d,%d,%.2f,%.1f,%.0f,%u\n",
                   px, py, (double)cc.grad, (double)cc.depthCm, (double)conf,
                   (unsigned)cc.quality);
          g_lastStepGrad = cc.grad;           // v9.3: لحزمة النبضة القياسية
          g_bStepResultReady = true;
        }
      }
#else
      if (!g_sensor.saturated) {            // العينة المشبعة لا تدخل متوسط الخلية
        g_cellSumGrad  += grad;             // تجميع على grad الحرفي (كالمان) لا على EMA
        g_cellSumSq    += (double)grad * grad;
        g_cellSumDepth += g_sensor.depthCm;
        g_cellCount++;
      } else g_cellSatSeen = true;
      if (g_imuOK && tiltNow > 10.0f) g_cellTiltBad = true; // v8.2: ميل أثناء العينة
      if (millis() - g_cellStartMs >= CELL_DWELL_MS) commitCellLocked();
#endif
    }
    xSemaphoreGive(xMutex);

    // ---- سطور LIVE بمعدل 2Hz إلى طابور السجل ----
    if (millis() - lastLiveLog >= LOG_LIVE_MS) {
      lastLiveLog = millis();
      SensorData s; int cx = -1, cy = -1; uint8_t m;
      xSemaphoreTake(xMutex, portMAX_DELAY);
      s = g_sensor; m = g_mode;
      if (m == MODE_SCAN && g_cellActive) cellCoord(g_cellIndex, cx, cy);
      xSemaphoreGive(xMutex);
      // دفع سطور LIVE الخام في MODE_LIVE وMODE_SCAN فقط (لا دفع في REVIEW)
      if (m == MODE_LIVE || m == MODE_SCAN) {
        char line[LOG_LINE_LEN];
        snprintf(line, sizeof(line), "%lu,%u,%d,%d,%.3f,%.3f,%.3f,%.3f,%.1f,%u,%u,%.1f,%.1f\n",
                 (unsigned long)millis(), (unsigned)m, cx, cy,
                 (double)s.ut1, (double)s.ut2, (double)s.gradKf, (double)s.anomaly,
                 (double)s.depthCm, (unsigned)s.quality, (unsigned)s.detect,
                 (double)s.tiltDeg, (double)s.yawDeg);
        pushLogLine(line);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// =============================================================
// مسجل SD — loggerTask, core 0, prio 2 (SPEC §7)
// كل وصول SD يتم هنا حصرياً (SPEC §12.4)
// =============================================================

// أول مسار غير موجود بالنمط المعطى (ترقيم تلقائي NNNN)
static bool nextPath(const char* fmt, char* out, size_t n) {
  for (int i = 1; i < 10000; i++) {
    snprintf(out, n, fmt, i);
    if (!g_fs->exists(out)) return true;
  }
  return false;
}

static void openLog() {
  if (!g_sdOK || !g_fs) {
    // استرجاع واحد هادئ: SD (إن كانت النشطة) ثم LittleFS كملاذ أخير
    g_sdOK = (g_storage == ST_SD && sdMountOnce(10000000)) || mountLFS();
    if (!g_sdOK) return;
    g_fs->mkdir(SD_DIR);
  }
  char path[40];
  if (!nextPath(SD_DIR "/scan_%04d.csv", path, sizeof(path))) { g_sdOK = false; return; }
  g_logFile = g_fs->open(path, FILE_WRITE);  // فتح واحد لكل مسح (SPEC §12.7)
  if (g_logFile) {
    g_logFile.print(CSV_HEADER);
    g_logFile.print("\n");                  // LF موحّد مع بقية الأسطر
    g_logFile.flush();
    Serial.printf("%s log: %s\n", storageName(), path);
  } else {
    g_sdOK = false;                         // فشل الفتح → وسم التخزين كفاشل (SPEC §7)
  }
}

static void closeLog() {
  if (g_logFile) { g_logFile.flush(); g_logFile.close(); }
}

// حفظ الشبكة grid_NNNN.csv — الصفوف: x,y,grad,depthCm,quality
static void saveGridCSV() {
  if (!g_sdOK) return;
  char path[40];
  if (!nextPath(SD_DIR "/grid_%04d.csv", path, sizeof(path))) return;
  File f = g_fs->open(path, FILE_WRITE);
  if (!f) { g_sdOK = false; return; }
  // نسخ الشبكة والأبعاد تحت القفل ثم تحريره فوراً — كتابة SD (حتى 1024 سطر)
  // تتم من النسخة بدون حجز xMutex حتى لا يتجمّد sensorTask (إصلاح P1).
  static ScanCell s_saveGrid[GRID_MAX][GRID_MAX];
  static uint8_t s_saveW = 0, s_saveH = 0;
  xSemaphoreTake(xMutex, portMAX_DELAY);
  memcpy(s_saveGrid, g_grid, sizeof(g_grid));
  s_saveW = (uint8_t)g_gridW;
  s_saveH = (uint8_t)g_gridH;
  xSemaphoreGive(xMutex);
  for (int y = 0; y < s_saveH; y++)
    for (int x = 0; x < s_saveW; x++) {
      ScanCell c = s_saveGrid[y][x];
      f.printf("%d,%d,%.3f,%.1f,%u\n", x, y, (double)c.grad,
               (double)c.depthCm, (unsigned)c.quality);
    }
  f.close();
  Serial.printf("SD grid: %s\n", path);
}

// تحميل أحدث شبكة عند الإقلاع إلى وضع REVIEW (SPEC §7 اختياري)
static bool loadLatestGrid() {
  char path[40];
  int best = -1;
  for (int i = 1; i < 10000; i++) {
    char p[40];
    snprintf(p, sizeof(p), SD_DIR "/grid_%04d.csv", i);
    if (g_fs->exists(p)) { best = i; strncpy(path, p, sizeof(path)); }
    else break;
  }
  if (best < 0) return false;
  File f = g_fs->open(path, FILE_READ);
  if (!f) return false;

  // تمريرة 1: تحديد الأبعاد
  int maxx = -1, maxy = -1;
  char lb[96]; int li = 0;
  while (f.available()) {
    char c = (char)f.read();
    if (c == '\n' || li >= (int)sizeof(lb) - 1) {
      lb[li] = 0; li = 0;
      int x, y; float g, d; int q;
      if (sscanf(lb, "%d,%d,%f,%f,%d", &x, &y, &g, &d, &q) == 5) {
        if (x > maxx) maxx = x;
        if (y > maxy) maxy = y;
      }
    } else lb[li++] = c;
  }
  if (maxx < 0 || maxy < 0) { f.close(); return false; }
  f.seek(0);

  // تمريرة 2: التعبئة (تحت xMutex)
  xSemaphoreTake(xMutex, portMAX_DELAY);
  memset(g_grid, 0, sizeof(g_grid));
  g_gridW = constrain(maxx + 1, 1, GRID_MAX);
  g_gridH = constrain(maxy + 1, 1, GRID_MAX);
  int cnt = 0;
  li = 0;
  while (f.available()) {
    char c = (char)f.read();
    if (c == '\n' || li >= (int)sizeof(lb) - 1) {
      lb[li] = 0; li = 0;
      int x, y; float g, d; int q;
      if (sscanf(lb, "%d,%d,%f,%f,%d", &x, &y, &g, &d, &q) == 5 &&
          x >= 0 && x < g_gridW && y >= 0 && y < g_gridH) {
        g_grid[y][x].grad = g;
        g_grid[y][x].depthCm = d;
        g_grid[y][x].quality = (uint8_t)constrain(q, 0, 100);
        g_grid[y][x].filled = 1;
        cnt++;
      }
    } else lb[li++] = c;
  }
  g_filledCount = cnt;
  g_mode = MODE_REVIEW;
  g_baseline = 0;                          // قيم الخلايا مخزنة كـ grad خام
  xSemaphoreGive(xMutex);
  f.close();
  Serial.printf("SD loaded %s (%d cells)\n", path, cnt);
  return true;
}

// تهيئة SD على الناقل المشترك مع الشاشة (SCK=36, MISO=35, MOSI=37, CS=4)
// نستخدم كائن SPI العام (نفس host الذي هيّأه M5GFX) — كائن SPIClass منفصل
// يتصارع مع مضمّن الشاشة ويسبب فشل CMD0 (GO_IDLE_STATE failed / no token)
static bool s_sdSpiBegun = false;         // SPI.begin مرة واحدة فقط في عمر النظام
                                          // إعادته تعيد تهيئة ناقل الشاشة → تعتيم فوري

// تركيب واحد: لا يلمس تهيئة الناقل بعد أول مرة → آمن أثناء عمل الشاشة
static bool sdMountOnce(uint32_t freq) {
  if (!s_sdSpiBegun) {
    SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, -1);
    pinMode(PIN_SD_CS, OUTPUT);
    digitalWrite(PIN_SD_CS, HIGH);
    delay(5);
    s_sdSpiBegun = true;
  }
  if (SD.begin(PIN_SD_CS, SPI, freq, "/sd", 5, false)) return true;
  SD.end();
  delay(50);
  return false;
}

// التهيئة عند الإقلاع: سلّم ترددات كامل (الشاشة ما زالت في مرحلة آمنة هنا)
static bool initSD() {
  static const uint32_t freqs[] = { 25000000, 10000000, 4000000, 1000000 };
  for (uint8_t i = 0; i < sizeof(freqs) / sizeof(freqs[0]); i++) {
    if (sdMountOnce(freqs[i])) {
      Serial.printf("SD OK @ %lu Hz\n", (unsigned long)freqs[i]);
      return true;
    }
    Serial.printf("SD retry @ %lu Hz failed\n", (unsigned long)freqs[i]);
    delay(120);
  }
  Serial.println("SD init FAIL — keep running");
  return false;
}

// تركيب LittleFS على الفلاش الداخلي (~9MB بقسمة default_16MB) — بلا بطاقة إطلاقاً
static bool mountLFS() {
  LittleFS.end();                          // آمن حتى لو لم تكن مركّبة
  if (LittleFS.begin(true)) {              // true = تهيئة تلقائية عند أول فشل تركيب
    g_fs = &LittleFS;
    g_storage = ST_LFS;
    Serial.printf("LFS OK — internal flash (%u KB total)\n",
                  (unsigned)(LittleFS.totalBytes() / 1024));
    return true;
  }
  Serial.println("LFS mount FAIL");
  return false;
}

// قرار التخزين عند الإقلاع: SD أولاً ثم LittleFS تلقائياً (أو LittleFS مباشرة بـ FORCE_LITTLEFS)
static bool initStorage() {
#ifndef FORCE_LITTLEFS
  if (initSD()) {
    g_fs = &SD;
    g_storage = ST_SD;
    return true;
  }
  Serial.println("SD unusable — falling back to LittleFS (internal flash)");
#endif
  return mountLFS();
}

void loggerTask(void* pv) {
  (void)pv;
  g_sdOK = initStorage();
  if (g_sdOK) {
    g_fs->mkdir(SD_DIR);
    loadLatestGrid();
  }

  char line[LOG_LINE_LEN];
  uint32_t lastFlush = millis();
  for (;;) {
    // تصريف الطابور أولاً — حتى لا تضيع أسطر معلّقة (مثل سطر آخر خلية)
    // عند معالجة طلب الإغلاق بعد اكتمال الشبكة
    bool wrote = false;
    while (xQueueReceive(xLogQueue, line, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (g_logFile) {
        g_logFile.write((const uint8_t*)line, strnlen(line, LOG_LINE_LEN));
        wrote = true;
      }
    }
    // سياسة flush: كل 2s أو عند اكتمال خلية
    if (g_logFile && wrote && (g_bFlushRequest || millis() - lastFlush >= LOG_FLUSH_MS)) {
      g_logFile.flush();
      lastFlush = millis();
      g_bFlushRequest = false;
    }
    // طلبات الفتح/الإغلاق/الحفظ — بعد التصريف؛ closeLog يجري flush نهائياً قبل الإغلاق
    if (g_bOpenLogRequest)  { g_bOpenLogRequest = false;  openLog(); }
    if (g_bCloseLogRequest) { g_bCloseLogRequest = false; closeLog(); }
    if (g_bSaveGridRequest) { g_bSaveGridRequest = false; if (g_sdOK) saveGridCSV(); }
  }
}

// =============================================================
// BLE (SPEC §8) — نفس نمط المستخدم الحالي
// =============================================================

class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    (void)s;
    g_bleConnected = true;
  }
  void onDisconnect(BLEServer* s) override {
    g_bleConnected = false;
    g_dumpActive = false;
    s->getAdvertising()->start();         // إعادة الإعلان
  }
};

// محلل أوامر RX — مع فحص الطول ≤32B (SPEC §12.6)
static void parseBleCommand(const char* cmd) {
  if (!strncasecmp(cmd, "START", 5))      g_bStartRequest = true;
  else if (!strncasecmp(cmd, "STOP", 4))  g_bStopRequest = true;
  else if (!strncasecmp(cmd, "CAL", 3))   g_bCalRequest = true;
  else if (!strncasecmp(cmd, "MAGCAL", 6)) g_bMagCalRequest = true; // v8.5: معايرة hard-iron (15s دوران)
  else if (!strncasecmp(cmd, "TXT", 3)) {   // v9.4: TXT,0 / TXT,1 — تبديل بث السطر النصي (بديل CH05)
    const char* p = cmd + 3;
    if (*p == ',' || *p == ' ') p++;
    if (*p == '0') g_textStreamOn = false;
    else if (*p == '1') g_textStreamOn = true;
    Serial.printf("TXT STREAM -> %s\n", g_textStreamOn ? "ON" : "off");
  }
  else if (!strncasecmp(cmd, "THR", 3)) {   // v9.0: THR,<float> — ضبط عتبة الكشف (المصدر الوحيد)
    const char* p = cmd + 3;
    if (*p == ',' || *p == ' ') p++;        // الصيغة المتوقعة «THR,5.0» (نقبل مسافة أيضاً)
    float t;
    if (sscanf(p, "%f", &t) == 1) {
      t = constrain(t, THRESH_MIN_UT, THRESH_MAX_UT);   // clamp [1.0, 50.0] µT
      thresh = t;
      g_threshHintUntil = millis() + THRESH_HINT_MS;    // تلميح على الشاشة كما كان سابقاً
      Serial.printf("THRESH -> %.1f uT\n", (double)t);
    }
  }
#if SCAN_STEP_MODE
  // v8.3: نبضة خطوية — PULSE اسم بديل لـ STEP؛ الاستهلاك والطباعة في loop
  else if (!strncasecmp(cmd, "STEP", 4) || !strncasecmp(cmd, "PULSE", 5))
    g_bStepRequest = true;
#endif
  else if (!strncasecmp(cmd, "DUMP", 4)) {
    if (g_bleConnected) { g_dumpActive = true; g_dumpIndex = 0; }
  }
  else if (!strncasecmp(cmd, "FILES", 5)) {
    if (g_bleConnected) g_filesReq = true;
  }
  else if (!strncasecmp(cmd, "GET", 3)) {
    if (g_bleConnected) {
      const char* p = cmd + 3;
      while (*p == ' ') p++;             // تخطي المسافات البادئة
      size_t j = 0;                      // نسخ معقّم: مسار آمن فقط وبدون «..»
      for (size_t i = 0; p[i] && j < sizeof(g_getFileName) - 1; i++) {
        char c = p[i];
        if (isalnum((uint8_t)c) || c == '/' || c == '_' || c == '-' || c == '.')
          g_getFileName[j++] = c;
      }
      g_getFileName[j] = 0;
      if (j > 0 && !strstr(g_getFileName, "..")) g_getReq = true;
    }
  }
  else if (!strncasecmp(cmd, "GRID", 4)) {
    int w, h;
    if (sscanf(cmd + 4, "%d %d", &w, &h) == 2) {
      xSemaphoreTake(xMutex, portMAX_DELAY);
      if (g_mode != MODE_SCAN) {          // لا تغيير للأبعاد أثناء المسح
        g_gridW = constrain(w, 1, GRID_MAX);
        g_gridH = constrain(h, 1, GRID_MAX);
        memset(g_grid, 0, sizeof(g_grid));
        g_filledCount = 0;
      }
      xSemaphoreGive(xMutex);
    }
  }
}

class RxCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    size_t len = c->getLength();          // getData/getLength متوافقة مع 2.x و3.x
    if (len == 0 || len > BLE_CMD_MAX) return;
    char cmd[BLE_CMD_MAX + 1];
    memcpy(cmd, c->getData(), len);
    cmd[len] = 0;
    parseBleCommand(cmd);
  }
};

void bleInit() {
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEDevice::setMTU(BLE_MTU);             // MTU 64 بعد init (SPEC §8)
  g_bleServer = BLEDevice::createServer();
  g_bleServer->setCallbacks(new ServerCB());
  BLEService* svc = g_bleServer->createService(BLE_SERVICE_UUID);

  g_bleTxChar = svc->createCharacteristic(BLE_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  g_bleTxChar->addDescriptor(new BLE2902());

  BLECharacteristic* rx = svc->createCharacteristic(
      BLE_RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RxCB());

  svc->start();
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_SERVICE_UUID);
  adv->setScanResponse(true);
  adv->start();
}

// إرسال الحزمة الحية — نسخ تحت xMutex ثم notify (نمط المستخدم الحالي)
void sendBlePacket() {
  BlePacket pkt;
  pkt.timestamp = millis();
  xSemaphoreTake(xMutex, portMAX_DELAY);
  pkt.ut1 = g_sensor.ut1;
  pkt.ut2 = g_sensor.ut2;
  pkt.grad = g_sensor.grad;
  pkt.gradKf = g_sensor.gradKf;
  pkt.depthCm = g_sensor.depthCm;
  pkt.detect = g_sensor.detect;
  pkt.mode = g_mode;
  pkt.yaw = g_sensor.yawDeg;          // v9.1: الاتجاه قبل حساب CRC
  xSemaphoreGive(xMutex);
  pkt.crc16 = crc16_ccitt((const uint8_t*)&pkt, sizeof(BlePacket) - sizeof(uint16_t));

  // فحص الاشتراك: لا يلزم فحص صريح هنا — في مكتبة Bluedroid (Arduino-ESP32 BLE)
  // تقوم BLECharacteristic::notify() داخلياً بالمرور على المتصلين المفعّل لديهم
  // بت الإشعارات في وصف BLE2902 فقط، والإرسال بدون مشترك عملية لا-شيء. كما أن
  // المكتبة لا توفر getSubscribedCount() على مستوى BLE2902 (موجودة في NimBLE فقط).
  g_bleSending = true;
  g_bleTxChar->setValue((uint8_t*)&pkt, sizeof(BlePacket));
  g_bleTxChar->notify();
  g_bleSending = false;
}

#if TEXT_STREAM_ENABLE
// v9.4: سطر نصي ASCII بسيط عبر نفس قناة BLE UART — بديل عملي لمخرجات CH05/HC-05
// الصيغة: GRAD,<grad_uT>,DEPTH,<cm>,DET,<0|1>,MODE,<L|S|R>\r\n
// اختيرت صيغة CSV مفتاحية (key,value) بدل رقم خام لأنها تُقرأ مباشرة في أي طرفية
// نصية (Serial Bluetooth Terminal ونحوها) دون الحاجة لفك تشفير بنية ثنائية.
void sendBleTextLine() {
  if (!g_bleConnected || g_bleSending || !g_bleTxChar) return;
  SensorData s; uint8_t m;
  xSemaphoreTake(xMutex, portMAX_DELAY);
  s = g_sensor; m = g_mode;
  xSemaphoreGive(xMutex);
  const char* mc = (m == MODE_SCAN) ? "S" : (m == MODE_REVIEW ? "R" : "L");
  char line[80];
  int n = snprintf(line, sizeof(line), "GRAD,%.2f,DEPTH,%.1f,DET,%u,MODE,%s\r\n",
                   (double)s.anomaly, (double)s.depthCm, (unsigned)s.detect, mc);
  g_bleSending = true;
  g_bleTxChar->setValue((uint8_t*)line, (size_t)n);
  g_bleTxChar->notify();
  g_bleSending = false;
}
#endif

// DUMP: بث الشبكة كنص «G,x,y,grad,depth,q» بقطع ≤100B ثم GEND
void pumpBleDump() {
  if (!g_dumpActive || !g_bleConnected || g_bleSending) return;
  char buf[BLE_DUMP_CHUNK + 4];
  size_t used = 0;

  xSemaphoreTake(xMutex, portMAX_DELAY);
  int total = g_gridW * g_gridH;
  while (g_dumpIndex < total) {
    int x, y; cellCoord(g_dumpIndex, x, y);
    ScanCell c = g_grid[y][x];
    char row[40];
    int rl = snprintf(row, sizeof(row), "G,%d,%d,%.2f,%.1f,%u\n",
                      x, y, (double)c.grad, (double)c.depthCm, (unsigned)c.quality);
    if (used + rl > BLE_DUMP_CHUNK) break;
    memcpy(buf + used, row, rl);
    used += rl;
    g_dumpIndex++;
  }
  bool done = (g_dumpIndex >= total);
  xSemaphoreGive(xMutex);

  if (used > 0) {
    g_bleSending = true;
    g_bleTxChar->setValue((uint8_t*)buf, used);
    g_bleTxChar->notify();
    g_bleSending = false;
  } else if (done) {
    const char* end = "GEND\n";
    g_bleSending = true;
    g_bleTxChar->setValue((uint8_t*)end, strlen(end));
    g_bleTxChar->notify();
    g_bleSending = false;
    g_dumpActive = false;
  }
}

#if SCAN_STEP_MODE
// v8.3: بث سطر نتيجة النبضة «R,x,y,grad,depth,conf,q» — نفس نمط serviceFileRequests
void pumpStepResult() {
  if (!g_bStepResultReady || !g_bleConnected || g_bleSending || !g_bleTxChar) return;
  char line[sizeof(g_stepResult)];
  memcpy(line, g_stepResult, sizeof(line)); // نسخة محلية — العلم يُصفَّر قبل الإرسال
  g_bStepResultReady = false;
  g_bleSending = true;
  g_bleTxChar->setValue((uint8_t*)line, strnlen(line, sizeof(line)));
  g_bleTxChar->notify();
  g_bleSending = false;
  // v9.3: الحزمة القياسية العالمية للفيزليزر — قيمة int = grad_uT × 100 منتهية بـ \r\n
  // تُرسل مع كل نبضة مثبّتة (زر PULSE أو أمر STEP) وتعمل مع تطبيقات FG القياسية
  {
    char std[20];
    int sl = snprintf(std, sizeof(std), "%d\r\n", (int)lroundf(g_lastStepGrad * 100.0f));
    delay(20);                            // فاصل قصير بين إشعارين متتاليين
    g_bleSending = true;
    g_bleTxChar->setValue((uint8_t*)std, sl);
    g_bleTxChar->notify();
    g_bleSending = false;
  }
}
#endif

// FILES/GET: تنزيل ملفات التخزين (SD أو LittleFS) عبر BLE بقطع ≤100B
void serviceFileRequests() {
  if (!g_bleConnected || g_bleSending || !g_bleTxChar) return;

  if (g_filesReq) {                      // قائمة الملفات: «L,<name>,<size>» ثم LEND
    g_filesReq = false;
    if (g_fs) {
      File dir = g_fs->open(SD_DIR);
      if (dir) {
        char buf[BLE_DUMP_CHUNK + 4];
        size_t used = 0;
        File f = dir.openNextFile();
        while (f) {
          char row[48];
          int rl = snprintf(row, sizeof(row), "L,%s,%u\n", f.name(), (unsigned)f.size());
          if (used + (size_t)rl > BLE_DUMP_CHUNK) {
            g_bleSending = true;
            g_bleTxChar->setValue((uint8_t*)buf, used);
            g_bleTxChar->notify();
            g_bleSending = false;
            delay(30);
            used = 0;
          }
          memcpy(buf + used, row, rl);
          used += rl;
          f = dir.openNextFile();
        }
        int tl = snprintf(buf + used, sizeof(buf) - used, "LEND\n");
        g_bleSending = true;
        g_bleTxChar->setValue((uint8_t*)buf, used + tl);
        g_bleTxChar->notify();
        g_bleSending = false;
        dir.close();
      }
    }
  }

  if (g_getReq) {                        // محتوى ملف: FBEGIN <name> <size> … FEND
    g_getReq = false;
    if (g_fs && g_getFileName[0]) {
      if (g_logFile) {                   // لا قراءة أثناء سجل مفتوح
        const char* busy = "FBUSY\n";
        g_bleSending = true;
        g_bleTxChar->setValue((uint8_t*)busy, strlen(busy));
        g_bleTxChar->notify();
        g_bleSending = false;
      } else {
        File f = g_fs->open(g_getFileName, FILE_READ);
        if (f) {
          char hdr[56];
          int hl = snprintf(hdr, sizeof(hdr), "FBEGIN %s %u\n",
                            g_getFileName, (unsigned)f.size());
          g_bleSending = true;
          g_bleTxChar->setValue((uint8_t*)hdr, hl);
          g_bleTxChar->notify();
          delay(30);
          uint8_t buf[BLE_DUMP_CHUNK];
          int n;
          while ((n = f.read(buf, BLE_DUMP_CHUNK)) > 0) {
            g_bleTxChar->setValue(buf, n);
            g_bleTxChar->notify();
            delay(25);                   // إتاحة لمكدس BLE — لا فيض إشعارات
          }
          f.close();
          const char* end = "FEND\n";
          g_bleTxChar->setValue((uint8_t*)end, strlen(end));
          g_bleTxChar->notify();
          g_bleSending = false;
        }
      }
    }
  }
}

// =============================================================
// تشخيص ناقل PORT.A عند الإقلاع + استرداد تلقائي بتبديل الأطراف — v8.5.1
// المشكلة الميدانية: فشل كشف الجهاز على PORT.A (MAG absent)
// رغم وصول التغذية — أسباب مرجّحة: غياب begin صريح أو SDA/SCL معكوسان
// في كابل GROVE الملحوم. هذا القسم يفحص الناقل ويسترده قبل magInit.
// v9.0: المتوقع الآن 0x1C (أو 0x1E) فقط — لا جهاز آخر على الناقل.
// =============================================================
static bool g_i2cSwapped = false;         // true = الناقل يعمل بأطراف معكوسة

// فحص عناوين 0x08..0x77 عبر M5.Ex_I2C — يطبع سطر Serial ويظهر نتيجة مختصرة
// على شاشة الإقلاع (1.5s). يعيد عدد العناوين التي أجابت بـ ACK.
static int i2cScanPortA(bool showScreen) {
  char line[96];
  int n = snprintf(line, sizeof(line), "I2C scan:");
  int found = 0;
  for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
    // M5Unified I2C_Class: bool start(uint8_t addr, bool read, uint32_t freq)
    // تعيد true عند ACK؛ يلزم stop() لتحرير الناقل بعد كل جسّ.
    if (M5.Ex_I2C.start(addr, false, I2C_SCAN_FREQ)) {
      found++;
      if (n < (int)sizeof(line) - 6) n += snprintf(line + n, sizeof(line) - n, " 0x%02X", addr);
    }
    M5.Ex_I2C.stop();
  }
  if (!found && n < (int)sizeof(line) - 9) snprintf(line + n, sizeof(line) - n, " (none)");
  Serial.println(line);

  if (showScreen) {
    M5.Display.setTextSize(1);
    M5.Display.setTextColor(found ? TFT_GREEN : TFT_RED, TFT_BLACK);
    M5.Display.setCursor(10, 44);
    // نسخة مدمجة للشاشة: «I2C: 1C» أو «I2C: (none)»
    M5.Display.print("I2C: ");
    if (found) {
      for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (M5.Ex_I2C.start(addr, false, I2C_SCAN_FREQ)) M5.Display.printf("%02X ", addr);
        M5.Ex_I2C.stop();
      }
    } else {
      M5.Display.print("(none)");
    }
    delay(1500);                            // وقت كافٍ للقراءة على شاشة الإقلاع
  }
  return found;
}

// تُستدعى من setup بعد M5.begin وقبل magInit:
// begin صريح للناقل الخارجي → فحص → استرداد بتبديل الأطراف إن كان فارغاً.
static void i2cPortABootDiagnostics() {
  // M5Unified I2C_Class: bool begin(void) — يهيّئ منفذ PORT.A الافتراضي للوحة
  bool begun = M5.Ex_I2C.begin();
  Serial.printf("Ex_I2C begin: %s\n", begun ? "OK" : "FAIL");

  bool magSeen = false;
  if (i2cScanPortA(false) > 0) {
    magSeen = M5.Ex_I2C.start(MAG_ADDR_PRIMARY,  false, I2C_SCAN_FREQ); M5.Ex_I2C.stop();
    if (!magSeen) { magSeen = M5.Ex_I2C.start(MAG_ADDR_FALLBACK, false, I2C_SCAN_FREQ); M5.Ex_I2C.stop(); }
  }

  if (!magSeen) {
    // لا 0x1C ولا 0x1E — يُحتمل SDA/SCL معكوسان في الكابل الملحوم
#if I2C_SWAP_TRY
    Serial.println("I2C empty — trying swapped SDA/SCL (SDA=G1, SCL=G2)");
    // M5Unified I2C_Class: bool begin(int pin_sda, int pin_scl, uint32_t freq)
    // (لا توجد end() في I2C_Class — begin يعيد التهيئة مباشرة)
    if (M5.Ex_I2C.begin(PIN_PORTA_SCL, PIN_PORTA_SDA, I2C_SCAN_FREQ)) {
      if (i2cScanPortA(true) > 0) {
        g_i2cSwapped = true;
        Serial.println("I2C swapped OK — cable has SDA/SCL crossed");
        return;
      }
    }
    // لم ينفع التبديل — استعد الافتراضي حتى يبقى السلوك كما كان
    Serial.println("I2C still empty — restoring default pins (SDA=G2, SCL=G1)");
    M5.Ex_I2C.begin();
    g_i2cSwapped = false;
#endif
    i2cScanPortA(true);                     // أظهر نتيجة الفحص (الفارغة غالباً) على الشاشة
  } else {
    i2cScanPortA(true);                     // ناقل سليم — أظهر العناوين على شاشة الإقلاع
  }
}

// =============================================================
// مقياس المغناطيسية LIS3MDL (PORT.A، I2C 0x1C/0x1E) — v8.5
// مشغل ريجسترات مباشر عبر M5.Ex_I2C @100kHz — بلا مكتبات خارجية.
// الجهاز الوحيد على الناقل الخارجي في v9.0 — يُستطلع من loop فقط،
// أبداً من sensorTask (يبقى مسار ADS1256 دون تغيير).
// =============================================================
static bool     magPresent = false;       // تُضبط في magInit (setup) وتُقرأ من loop فقط
static uint8_t  magAddr = 0;              // العنوان المكتشف (0x1C أو 0x1E)
static uint32_t magLastPollMs = 0;
// معايرة hard-iron: min/max متكيّفان ببطء لكل محور + أوفست (max+min)/2
static float magMin[3] = { 1e9f,  1e9f,  1e9f};
static float magMax[3] = {-1e9f, -1e9f, -1e9f};
static float magOff[3] = {0, 0, 0};
// حالة أمر MAGCAL (loop فقط)
static bool       magCalActive = false;
static uint32_t   magCalStartMs = 0;
// نتائج مرئية للواجهة/BLE — g_headingDeg/g_magYawFused/g_magOK مُعرّفة في قسم IMU أعلاه
// (تُكتب من loop فقط، float ذرّية على Xtensa)

// قراءة XYZ خام (int16 LE) مع زيادة تلقائية للعنوان (bit7)
static bool magReadRaw(int16_t& mx, int16_t& my, int16_t& mz) {
  uint8_t b[6];
  if (!M5.Ex_I2C.readRegister(magAddr, MAG_REG_OUT_X_L | 0x80, b, 6, MAG_I2C_FREQ))
    return false;
  mx = (int16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
  my = (int16_t)((uint16_t)b[2] | ((uint16_t)b[3] << 8));
  mz = (int16_t)((uint16_t)b[4] | ((uint16_t)b[5] << 8));
  return true;
}

// كشف + تهيئة LIS3MDL — استدعاء واحد من setup بعد i2cPortABootDiagnostics
static void magInit() {
  // تحقّق WHO_AM_I على العنوانين
  uint8_t who = 0, who2 = 0;
  bool rd1 = M5.Ex_I2C.readRegister(MAG_ADDR_PRIMARY, MAG_REG_WHO_AM_I, &who, 1, MAG_I2C_FREQ);
  bool rd2 = M5.Ex_I2C.readRegister(MAG_ADDR_FALLBACK, MAG_REG_WHO_AM_I, &who2, 1, MAG_I2C_FREQ);
  // v8.5.2: تشخيص خام — هل العنوان يجيب أصلاً؟ وماذا يقرأ WHO_AM_I؟
  Serial.printf("MAG probe: 0x1C rd=%d who=0x%02X | 0x1E rd=%d who=0x%02X\n",
                (int)rd1, (unsigned)who, (int)rd2, (unsigned)who2);
  if (rd1 && who == MAG_WHO_AM_I_VAL) {
    magAddr = MAG_ADDR_PRIMARY;
  } else if (rd2 && who2 == MAG_WHO_AM_I_VAL) {
    magAddr = MAG_ADDR_FALLBACK;
  } else {
    magPresent = false;                   // غياب صامت — لا شيء جديد يظهر في الواجهة
    Serial.println("MAG absent (LIS3MDL) — gyro-only yaw");
    return;
  }
  // تسلسل التهيئة
  uint8_t v;
  bool ok = true;
  v = MAG_CTRL1_VAL; ok &= M5.Ex_I2C.writeRegister(magAddr, MAG_REG_CTRL1, &v, 1, MAG_I2C_FREQ);
  v = MAG_CTRL2_VAL; ok &= M5.Ex_I2C.writeRegister(magAddr, MAG_REG_CTRL2, &v, 1, MAG_I2C_FREQ);
  v = MAG_CTRL3_VAL; ok &= M5.Ex_I2C.writeRegister(magAddr, MAG_REG_CTRL3, &v, 1, MAG_I2C_FREQ);
  v = MAG_CTRL4_VAL; ok &= M5.Ex_I2C.writeRegister(magAddr, MAG_REG_CTRL4, &v, 1, MAG_I2C_FREQ);
  delay(10);                              // استقرار بعد الكتابات
  // تحقق بالقراءة الراجعة: CTRL_REG3 يجب أن يكون 0x00 (وضع التحويل المستمر)
  uint8_t rb = 0xFF;
  ok &= M5.Ex_I2C.readRegister(magAddr, MAG_REG_CTRL3, &rb, 1, MAG_I2C_FREQ);
  if (!ok || rb != MAG_CTRL3_VAL) {
    magPresent = false;
    Serial.printf("MAG init FAIL (writeback rb=0x%02X) — gyro-only yaw\n", (unsigned)rb);
    return;
  }
  magPresent = true;
  g_magOK = true;
  Serial.printf("MAG OK LIS3MDL @0x%02X (PORT.A)\n", (unsigned)magAddr);
  M5.Display.setTextSize(1);
  M5.Display.setTextColor(TFT_GREEN, TFT_BLACK);
  M5.Display.setCursor(80, 32);
  M5.Display.print("MAG OK");
}

// تطبيع -180..180
static float wrap180(float a) {
  while (a >  180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

// استطلاع ~20Hz غير حاجب — يُستدعى من loop فقط
static void pollMag() {
  if (!magPresent) return;
  uint32_t now = millis();
  if (now - magLastPollMs < MAG_POLL_MS) return;
  magLastPollMs = now;

  int16_t rx, ry, rz;
  if (!magReadRaw(rx, ry, rz)) return;    // خطأ ناقل عابر → تخطَّ العينة بصمت
  float fx = (float)rx, fy = (float)ry, fz = (float)rz;

  // ---- معايرة hard-iron تلقائية: min/max يتسعان باستمرار، الأوفست = (max+min)/2 ----
  float v[3] = {fx, fy, fz};
  for (int i = 0; i < 3; i++) {
    if (v[i] < magMin[i]) magMin[i] = v[i];
    if (v[i] > magMax[i]) magMax[i] = v[i];
    if (magMax[i] > magMin[i]) magOff[i] = 0.5f * (magMax[i] + magMin[i]);
  }

  // خصم الأوفست ثم التحويل إلى µT (6842 LSB/gauss عند ±4G)
  float mx = (fx - magOff[0]) * 100.0f / MAG_LSB_PER_GAUSS;
  float my = (fy - magOff[1]) * 100.0f / MAG_LSB_PER_GAUSS;
  float mz = (fz - magOff[2]) * 100.0f / MAG_LSB_PER_GAUSS;

  // نسخ pitch/roll (BMI270، درجات) تحت القفل — مكتوبة من sensorTask
  float pitchDeg, rollDeg, gyroYaw;
  xSemaphoreTake(xMutex, portMAX_DELAY);
  pitchDeg = g_sensor.pitch;
  rollDeg  = g_sensor.roll;
  xSemaphoreGive(xMutex);
  gyroYaw  = g_yawDeg;                     // float — قراءة ذرّية (sensorTask فقط يكتبها)

  // ---- تعويض الميل ----
  float pitch = pitchDeg * DEG_TO_RAD;
  float roll  = rollDeg  * DEG_TO_RAD;
  float Xh = mx * cosf(pitch) + mz * sinf(pitch);
  float Yh = mx * sinf(roll) * sinf(pitch) + my * cosf(roll) - mz * sinf(roll) * cosf(pitch);
  float magYaw = atan2f(-Yh, Xh) * RAD_TO_DEG;   // 0..360
  while (magYaw < 0.0f)     magYaw += 360.0f;
  while (magYaw >= 360.0f)  magYaw -= 360.0f;
  g_headingDeg = magYaw;

  // ---- دمج تكميلي مع yaw الجيروسكوبي (معالجة التفاف 0/360) ----
  float diff = wrap180(magYaw - gyroYaw);
  g_magYawFused = wrap180(gyroYaw + MAG_FUSE_ALPHA * diff);

  // ---- نافذة MAGCAL (15s من دوران المستخدم) ----
  if (magCalActive && now - magCalStartMs >= MAG_CAL_MS) {
    magCalActive = false;
    Serial.printf("MAGCAL DONE off=(%.0f,%.0f,%.0f)\n",
                  (double)magOff[0], (double)magOff[1], (double)magOff[2]);
    if (g_bleConnected && !g_bleSending && g_bleTxChar) {
      const char* done = "OK MAGCAL DONE\n";
      g_bleSending = true;
      g_bleTxChar->setValue((uint8_t*)done, strlen(done));
      g_bleTxChar->notify();
      g_bleSending = false;
    }
  }
}

// =============================================================
// الواجهة (SPEC §10) — CoreS3 بلا أزرار فيزيائية: لمس فقط
// =============================================================

const char* modeStr(uint8_t m) {
  return m == MODE_SCAN ? "SCAN" : (m == MODE_REVIEW ? "REVIEW" : "LIVE");
}

// منحدر لوني: أزرق→سماوي→أخضر→أصفر→أحمر حسب t ∈ [0,1]
uint16_t heatColor(float t) {
  t = constrain(t, 0.0f, 1.0f);
  float r, g, b;
  if      (t < 0.25f) { r = 0;               g = 4 * t;             b = 1; }
  else if (t < 0.50f) { r = 0;               g = 1;                 b = 1 - 4 * (t - 0.25f); }
  else if (t < 0.75f) { r = 4 * (t - 0.50f); g = 1;                 b = 0; }
  else                { r = 1;               g = 1 - 4 * (t - 0.75f); b = 0; }
  return M5.Display.color565((uint8_t)(r * 255), (uint8_t)(g * 255), (uint8_t)(b * 255));
}

// حساب هندسة الخريطة الحرارية (مشترك بين الرسم واللمس)
static void heatGeom(int gw, int gh, int& cs, int& ox) {
  cs = min(320 / gw, UI_HEAT_H / gh);
  if (cs < 1) cs = 1;
  ox = (320 - cs * gw) / 2;
}

// =============================================================
// مساعدات لوحة الكشف الاحترافية (LIVE/SCAN/REVIEW)
// كل الإحداثيات مضبوطة ضمن 320×240 والتذييل (y>=208) لا يُغطى
// =============================================================

// لون إطار رمادي غامق موحّد للوحة الأجهزة
#define UI_FRAME_COL   0x4208
// لون خلفية المقاييس الفارغة (رمادي داكن جداً)
#define UI_TRACK_COL   0x1082
// عمق العرض الأقصى لعمود DEPTH (سم)
#define UI_DEPTH_FULL_CM  300.0f
// ارتفاع الخريطة الحرارية في وضع SCAN (مصغّرة لإفساح مجال للمقاييس)
#define UI_SCAN_HEAT_H    132

// نسبة الثقة 0..100: مزيج من قوة الشذوذ وجودة الإشارة
// conf = clamp( 0.5*(|anomaly|/(4*thresh)) + 0.5*(quality/100), 0, 1 ) * 100
static float confPercent(float anomaly, float quality) {
  float c = 0.5f * (fabsf(anomaly) / (4.0f * thresh))
          + 0.5f * (quality / 100.0f);
  return constrain(c, 0.0f, 1.0f) * 100.0f;
}

// منحدر أخضر→أصفر→أحمر حسب t ∈ [0,1] (لمقياس الإشارة)
static uint16_t gyrColor(float t) {
  t = constrain(t, 0.0f, 1.0f);
  uint8_t r, g;
  if (t < 0.5f) { r = (uint8_t)(510.0f * t);        g = 255; }
  else          { r = 255;  g = (uint8_t)(510.0f * (1.0f - t)); }
  return M5.Display.color565(r, g, 0);
}

// لون قيمة GRAD الكبيرة: رمادي تحت العتبة، برتقالي عند الاقتراب،
// أحمر وامض (toggle كل 300ms) عند detect
static uint16_t gradValueColor(float absAnom, uint8_t detect) {
  if (detect) return ((millis() / 300) & 1) ? TFT_RED : 0x8000; // أحمر/أحمر داكن
  if (absAnom >= thresh * 0.6f) return TFT_ORANGE;
  return TFT_LIGHTGREY;
}

// مقياس أفقي متدرج (أخضر→أصفر→أحمر) مع مؤشر متحرك عند frac
static void drawHGauge(int x, int y, int w, int h, float frac) {
  frac = constrain(frac, 0.0f, 1.0f);
  M5.Display.drawRect(x - 1, y - 1, w + 2, h + 2, UI_FRAME_COL);
  for (int i = 0; i < w; i++)
    M5.Display.drawFastVLine(x + i, y, h, gyrColor((float)i / (float)(w - 1)));
  // تعتيم الجزء غير المملوء بدل مسحه — بلا وميض
  int fill = (int)(w * frac);
  if (fill < w) M5.Display.fillRect(x + fill, y, w - fill, h, UI_TRACK_COL);
  // المؤشر المتحرك (خطان أبيضان)
  int mx = x + fill;
  if (mx > x)     M5.Display.drawFastVLine(mx - 1, y - 2, h + 4, TFT_WHITE);
  M5.Display.drawFastVLine(mx, y - 2, h + 4, TFT_WHITE);
}

// عمود عمودي بمنحدر لوني من (r0,g0,b0) أسفل إلى (r1,g1,b1) أعلى الملء
static void drawVBarGrad(int x, int y, int w, int h, float frac,
                         uint8_t r0, uint8_t g0, uint8_t b0,
                         uint8_t r1, uint8_t g1, uint8_t b1) {
  frac = constrain(frac, 0.0f, 1.0f);
  M5.Display.drawRect(x - 1, y - 1, w + 2, h + 2, UI_FRAME_COL);
  M5.Display.fillRect(x, y, w, h, UI_TRACK_COL);
  int fh = (int)(h * frac);
  for (int i = 0; i < fh; i++) {           // i=0 عند القاع
    float t = (h > 1) ? (float)i / (float)(h - 1) : 0.0f;
    uint16_t col = M5.Display.color565(
        (uint8_t)(r0 + (float)(r1 - r0) * t),
        (uint8_t)(g0 + (float)(g1 - g0) * t),
        (uint8_t)(b0 + (float)(b1 - b0) * t));
    M5.Display.drawFastHLine(x, y + h - 1 - i, w, col);
  }
}

// شريط عمق/ثقة أفقي مصغّر (لوضع SCAN) مع عنوان وقيمة رقمية
static void drawMiniHBar(int x, int y, int w, const char* label, float frac,
                         const char* valTxt, bool cyanBlue) {
  M5.Display.setTextSize(1);
  M5.Display.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  M5.Display.setCursor(x, y);
  M5.Display.print(label);
  int bx = x + 44, bw = w - 44 - 52;       // مساحة العنوان يساراً والقيمة يميناً
  frac = constrain(frac, 0.0f, 1.0f);
  M5.Display.drawRect(bx - 1, y - 1, bw + 2, 10, UI_FRAME_COL);
  M5.Display.fillRect(bx, y, bw, 8, UI_TRACK_COL);
  int fw = (int)(bw * frac);
  if (fw > 0) {
    uint16_t col = cyanBlue ? M5.Display.color565(0, 180, 255)
                            : M5.Display.color565(0, 220, 0);
    M5.Display.fillRect(bx, y, fw, 8, col);
  }
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setCursor(bx + bw + 6, y);
  M5.Display.print(valTxt);
}

// ---- لوحة LIVE: شاشة الكشف الرئيسية ----
// ---- لوحة LIVE v9.3: مخطط تدرج حي متدحرج في النصف العلوي + قراءات مدمجة ----
static void drawLivePanel(const SensorData& s) {
  float absAnom = fabsf(s.anomaly);
  float conf = confPercent(s.anomaly, (float)s.quality);
  uint16_t gc = gradValueColor(absAnom, s.detect);

  // (0) مخطط التدرج الحي — buffer دائري 316 عينة (~30s بمعدل رسم ~10Hz)
  static float hist[316];
  static int   hN = 0;
  hist[hN % 316] = s.anomaly; hN++;
  int cnt = (hN < 316) ? hN : 316;

  const int cx0 = 2, cy0 = 18, cw = 316, chh = 72;
  M5.Display.fillRect(cx0, cy0, cw, chh, TFT_BLACK);
  M5.Display.drawRect(cx0, cy0, cw, chh, UI_FRAME_COL);
  float scale = 2.0f * thresh;                          // مقياس تلقائي
  for (int i = 0; i < cnt; i++) {
    float a = fabsf(hist[(hN - cnt + i) % 316]);
    if (a > scale) scale = a;
  }
  const int midY = cy0 + chh / 2;
  M5.Display.drawFastHLine(cx0 + 1, midY, cw - 2, TFT_DARKGREY);   // خط الصفر
  int thPx = (int)(thresh / scale * (chh / 2 - 2));     // خطا ±العتبة (برتقالي متقطع)
  for (int x = cx0 + 1; x < cx0 + cw - 1; x += 6) {
    M5.Display.drawPixel(x, midY - thPx, TFT_ORANGE);
    M5.Display.drawPixel(x, midY + thPx, TFT_ORANGE);
  }
  for (int i = 0; i < cnt - 1; i++) {                    // المنحنى الأخضر
    float v0 = hist[(hN - cnt + i) % 316];
    float v1 = hist[(hN - cnt + i + 1) % 316];
    int y0 = midY - (int)(v0 / scale * (chh / 2 - 2));
    int y1 = midY - (int)(v1 / scale * (chh / 2 - 2));
    M5.Display.drawLine(cx0 + 1 + i, y0, cx0 + 2 + i, y1, TFT_GREEN);
  }

  // (1) قراءة GRAD كبيرة أسفل المخطط مباشرة
  M5.Display.setTextSize(1);
  M5.Display.setTextColor(TFT_DARKGREY, TFT_BLACK);
  M5.Display.setCursor(4, 94);
  M5.Display.print("GRAD uT");
  M5.Display.fillRect(4, 103, 172, 24, TFT_BLACK);
  M5.Display.setTextSize(3);
  M5.Display.setTextColor(gc, TFT_BLACK);
  M5.Display.setCursor(4, 103);
  M5.Display.printf("%8.2f", (double)s.anomaly);

  // (2) HDG يمين القراءة (عند توفر LIS3MDL) + فقاعة التسوية بأقصى اليمين
  if (g_magOK) {
    M5.Display.fillRect(192, 94, 80, 30, TFT_BLACK);
    M5.Display.setTextSize(1);
    M5.Display.setTextColor(TFT_DARKGREY, TFT_BLACK);
    M5.Display.setCursor(194, 95);
    M5.Display.print("HDG");
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(TFT_CYAN, TFT_BLACK);
    M5.Display.setCursor(194, 105);
    M5.Display.printf("%3d", (int)lroundf(g_headingDeg) % 360);
  }
  if (s.imuOK) {
    const int bcx = 294, bcy = 108, br = 12;
    M5.Display.fillRect(bcx - br - 1, bcy - br - 1, 2 * br + 2, 2 * br + 12, TFT_BLACK);
    uint16_t bc = (s.tiltDeg < 5.0f) ? TFT_GREEN : (s.tiltDeg < 10.0f ? TFT_YELLOW : TFT_RED);
    M5.Display.drawCircle(bcx, bcy, br, TFT_LIGHTGREY);
    M5.Display.drawFastHLine(bcx - 3, bcy, 7, TFT_DARKGREY);
    M5.Display.drawFastVLine(bcx, bcy - 3, 7, TFT_DARKGREY);
    float ox = constrain(s.roll,  -(float)(br - 2), (float)(br - 2));
    float oy = constrain(s.pitch, -(float)(br - 2), (float)(br - 2));
    M5.Display.fillCircle(bcx + (int)lroundf(ox), bcy - (int)lroundf(oy), 2, bc);
    M5.Display.setTextSize(1);
    M5.Display.setTextColor(bc, TFT_BLACK);
    M5.Display.setCursor(bcx - 14, bcy + br + 3);
    M5.Display.printf("%4.1f", (double)s.tiltDeg);
  }

  // (3) مقياس الإشارة الأفقي
  M5.Display.setTextSize(1);
  M5.Display.setTextColor(TFT_DARKGREY, TFT_BLACK);
  M5.Display.setCursor(10, 132);
  M5.Display.print("SIGNAL");
  drawHGauge(10, 142, 220, 12, absAnom / (4.0f * thresh));

  // (4) شارة التصنيف الذكي
  const char* cls; uint16_t edge, fill;
  if (absAnom < thresh)         { cls = "NO TARGET";     edge = TFT_DARKGREY; fill = 0x2104; }
  else if (s.anomaly > 0)       { cls = "METAL";         edge = TFT_ORANGE;   fill = 0x4000; }
  else                          { cls = "VOID / CAVITY"; edge = TFT_CYAN;     fill = 0x001F; }
  M5.Display.fillRoundRect(10, 158, 220, 26, 6, fill);
  M5.Display.drawRoundRect(10, 158, 220, 26, 6, edge);
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(edge, fill);
  int tw = (int)strlen(cls) * 12;
  M5.Display.setCursor(10 + (220 - tw) / 2, 164);
  M5.Display.print(cls);

  // (5) سطرا قراءات سفليان مدمجان: S1/S2 ثم DEPTH/CONF/Q/SAT
  M5.Display.fillRect(4, 188, 312, 18, TFT_BLACK);
  M5.Display.setTextSize(1);
  M5.Display.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  M5.Display.setCursor(4, 188);
  M5.Display.printf("S1:%8.2f  S2:%8.2f uT", (double)s.ut1, (double)s.ut2);
  M5.Display.setTextColor(TFT_DARKGREY, TFT_BLACK);
  M5.Display.setCursor(4, 198);
  M5.Display.printf("D:%3.0fcm  C:%3.0f%%  Q:%u  SAT:%s",
                    (double)s.depthCm, (double)conf, (unsigned)s.quality,
                    s.saturated ? "YES" : "no");
}

// ---- لوحة SCAN/REVIEW: خريطة حرارية + مقاييس ----
static void drawScanPanel(uint8_t mode, const SensorData& s, int gw, int gh,
                          int filled, float base, bool cellActive, int cx, int cy) {
  // في SCAN نصغّر ارتفاع الخريطة لإفساح مجال لشريطَي DEPTH/CONF
  int heatH = (mode == MODE_SCAN) ? UI_SCAN_HEAT_H : UI_HEAT_H;
  int cs = min(320 / gw, heatH / gh);
  if (cs < 1) cs = 1;
  int ox = (320 - cs * gw) / 2;

  // مسح منطقة الخريطة (يغطي أيضاً popup منتهي المدة) بلا fillScreen
  M5.Display.fillRect(0, UI_HEAT_Y, 320, 182, TFT_BLACK);

  for (int y = 0; y < gh; y++)
    for (int x = 0; x < gw; x++) {
      ScanCell& c = s_gridCopy[y][x];
      uint16_t col = c.filled
          ? heatColor(fabsf(c.grad - base) / HEAT_FULL_UT)
          : M5.Display.color565(24, 24, 24);
      M5.Display.fillRect(ox + x * cs, UI_HEAT_Y + y * cs, cs - 1, cs - 1, col);
    }
  // مؤشر أبيض على الخلية النشطة أثناء المسح
  if (mode == MODE_SCAN && cellActive && cx >= 0)
    M5.Display.drawRect(ox + cx * cs, UI_HEAT_Y + cy * cs, cs - 1, cs - 1, TFT_WHITE);

  if (mode == MODE_SCAN) {
    // شريطا DEPTH/CONFIDENCE مصغّران للخلية النشطة (قيم حية)
    char val[16];
    float conf = confPercent(s.anomaly, (float)s.quality);
    snprintf(val, sizeof(val), "%3.0fcm", (double)s.depthCm);
    drawMiniHBar(4, 162, 312, "DEPTH", s.depthCm / UI_DEPTH_FULL_CM, val, true);
    snprintf(val, sizeof(val), "%3.0f%%", (double)conf);
    drawMiniHBar(4, 176, 312, "CONF", conf / 100.0f, val, false);
#if SCAN_STEP_MODE
    // v8.3: حالة النبضة الخطوية — شريط «PULSE …» أثناء التجميع أو «ARMED» عند الانتظار
    M5.Display.fillRect(0, 190, 320, 10, TFT_BLACK);
    M5.Display.setTextSize(1);
    if (g_pulseActive) {
      float f = (float)(millis() - g_pulseStartMs) / (float)STEP_PULSE_MS;
      f = constrain(f, 0.0f, 1.0f);
      M5.Display.setTextColor(TFT_YELLOW, TFT_BLACK);
      M5.Display.setCursor(4, 191);
      M5.Display.print("PULSE");
      M5.Display.drawRect(49, 190, 122, 10, UI_FRAME_COL);
      M5.Display.fillRect(50, 191, (int)(120.0f * f), 8, TFT_YELLOW);
    } else {
      M5.Display.setTextColor(TFT_DARKGREY, TFT_BLACK);
      M5.Display.setCursor(4, 191);
      M5.Display.print("ARMED - PULSE btn/STEP");
    }
#endif
  }

  // شريط التقدم: خلايا مكتملة / الإجمالي
  int total = gw * gh;
  M5.Display.drawRect(0, UI_HEAT_Y + UI_HEAT_H + 1, 320, 5, TFT_DARKGREY);
  M5.Display.fillRect(0, UI_HEAT_Y + UI_HEAT_H + 1,
                      (int)(320.0f * filled / max(1, total)), 5, TFT_GREEN);

  // نافذة قيم الخلية في REVIEW عند النقر (+ الثقة المحسوبة من الخلية)
  if (mode == MODE_REVIEW && millis() < g_popupUntil &&
      g_popupX >= 0 && g_popupX < gw && g_popupY >= 0 && g_popupY < gh) {
    ScanCell& c = s_gridCopy[g_popupY][g_popupX];
    float cellConf = confPercent(c.grad - base, (float)c.quality);
    M5.Display.fillRoundRect(60, 52, 200, 116, 8, TFT_NAVY);
    M5.Display.drawRoundRect(60, 52, 200, 116, 8, TFT_WHITE);
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(TFT_WHITE, TFT_NAVY);
    M5.Display.setCursor(70, 60);  M5.Display.printf("Cell %d,%d", g_popupX, g_popupY);
    M5.Display.setCursor(70, 80);  M5.Display.printf("G:%7.2f uT", (double)c.grad);
    M5.Display.setCursor(70, 100); M5.Display.printf("D:%5.0f cm", (double)c.depthCm);
    M5.Display.setCursor(70, 120); M5.Display.printf("Q:%u", (unsigned)c.quality);
    M5.Display.setTextColor(TFT_GREEN, TFT_NAVY);
    M5.Display.setCursor(70, 140); M5.Display.printf("CONF:%3.0f%%", (double)cellConf);
  }
}

void drawUI() {
  // نسخ الحالة المشتركة دفعة واحدة
  SensorData s; uint8_t mode; int gw, gh, filled; float base;
  int cx = -1, cy = -1; bool cellActive;
  xSemaphoreTake(xMutex, portMAX_DELAY);
  s = g_sensor; mode = g_mode; gw = g_gridW; gh = g_gridH;
  filled = g_filledCount; base = g_baseline; cellActive = g_cellActive;
  if (cellActive) cellCoord(g_cellIndex, cx, cy);
  memcpy(s_gridCopy, g_grid, sizeof(g_grid));
  xSemaphoreGive(xMutex);

  // مسح كامل عند تبديل الوضع فقط — بقية الإطارات مسح موضعي (بلا وميض)
  static uint8_t s_lastMode = 0xFF;
  M5.Display.startWrite();
  if (mode != s_lastMode) { M5.Display.fillScreen(TFT_BLACK); s_lastMode = mode; }

  // ---- الترويسة: mode / storage / BLE / cell / heap (محفوظة) ----
  M5.Display.fillRect(0, 0, 320, 14, TFT_BLACK);
  M5.Display.setTextSize(1);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setCursor(4, 4);
  if (s.imuOK)   // v8.2: عرض yaw المختصر H:### — نُسقط Heap لإفساح العرض
    M5.Display.printf("%s SD:%s BLE:%s C:%d,%d H:%d",
                      modeStr(mode), storageName(), g_bleConnected ? "ON" : "off",
                      cx, cy, (int)lroundf(s.yawDeg));
  else
    M5.Display.printf("%s SD:%s BLE:%s Cell:%d,%d Heap:%dK",
                      modeStr(mode), storageName(), g_bleConnected ? "ON" : "off",
                      cx, cy, (int)(ESP.getFreeHeap() / 1024));
  M5.Display.drawFastHLine(0, 15, 320, UI_FRAME_COL);

  if (mode == MODE_LIVE) {
    drawLivePanel(s);
  } else {
    drawScanPanel(mode, s, gw, gh, filled, base, cellActive, cx, cy);
  }

  // v9.0: تلميح «THRESH: x.x uT» لمدة 1.5s بعد أمر BLE «THR» —
  // يُرسم داخل نفس الإطار المُخنّق (لا كسر للتخنيق). عند انتهائه نمسح
  // الشاشة مرة واحدة لإزالة بقاياه — مسح محلي فقط (بلا وميض أسود للوحة كاملة).
  bool hintNow = (millis() < g_threshHintUntil);
  static bool s_hintWas = false;
  if (s_hintWas && !hintNow) M5.Display.fillRect(58, 184, 204, 24, TFT_BLACK);
  s_hintWas = hintNow;
  if (hintNow) {
    char tb[24];
    snprintf(tb, sizeof(tb), "THRESH: %.1f uT", (double)thresh);
    M5.Display.fillRoundRect(60, 186, 200, 20, 4, TFT_NAVY);
    M5.Display.drawRoundRect(60, 186, 200, 20, 4, TFT_WHITE);
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(TFT_YELLOW, TFT_NAVY);
    M5.Display.setCursor(60 + (200 - (int)strlen(tb) * 12) / 2, 188);
    M5.Display.print(tb);
  }

  // ---- شريط اللمس السفلي v9.3: ثلاثة أزرار [MODE] [CAL] [PULSE] — كل زر ~106px ----
  // MODE : تبديل SCAN↔LIVE. CAL: قصيرة=معايرة شاملة، مطوّلة=تنقّل في العتبات (SENS).
  // PULSE: فعّال في SCAN فقط — قصيرة=نبضة قياس+حزمة قياسية، مطوّلة=تخطٍّ.
  {
    bool scanning = (mode == MODE_SCAN);
    bool calFlash = (millis() < g_calFlashUntil);
    const char* lbls[3] = { scanning ? "LIVE" : "SCAN", "CAL", "PULSE" };
    uint16_t    cl[3]   = { scanning ? TFT_RED : TFT_GREEN,
                            calFlash ? TFT_BLACK : TFT_YELLOW,
                            scanning ? TFT_WHITE : TFT_DARKGREY };
    M5.Display.setTextSize(2);
    for (int i = 0; i < 3; i++) {
      int bx = i * 107;
      bool inv = (i == 1 && calFlash);
      M5.Display.fillRect(bx + 1, UI_FOOTER_Y, 105, 31, inv ? TFT_YELLOW : TFT_BLACK);
      M5.Display.drawRect(bx + 1, UI_FOOTER_Y, 105, 31, inv ? TFT_YELLOW : cl[i]);
      M5.Display.setTextColor(cl[i], inv ? TFT_YELLOW : TFT_BLACK);
      int len = (int)strlen(lbls[i]);
      M5.Display.setCursor(bx + 1 + (105 - len * 12) / 2, UI_FOOTER_Y + 8);
      M5.Display.print(lbls[i]);
    }
  }
  M5.Display.endWrite();
}

// ---- معالجة اللمس (M5.Touch) مع ضغطة مطوّلة يدوية التتبع ----
// v9.3: شريط سفلي بثلاثة أزرار — [MODE] (x<107) [CAL/SENS] (107-213) [PULSE] (x>=214)
static void onTap(int16_t x, int16_t y) {
  uint8_t mode;
  xSemaphoreTake(xMutex, portMAX_DELAY);
  mode = g_mode;
  xSemaphoreGive(xMutex);

  if (y >= UI_FOOTER_Y) {                 // أزرار التذييل الثلاثة (v9.3)
    if (x < 107) {                        // MODE: تبديل SCAN ↔ LIVE بنفس أعلام BLE
      if (mode == MODE_SCAN)       g_bStopRequest = true;   // إيقاف → LIVE
      else if (mode == MODE_LIVE)  g_bStartRequest = true;  // بدء → SCAN
      else {                       // REVIEW: عودة إلى LIVE
        xSemaphoreTake(xMutex, portMAX_DELAY);
        g_mode = MODE_LIVE;
        xSemaphoreGive(xMutex);
      }
    } else if (x < 214) {                 // CAL: ضغطة قصيرة = معايرة شاملة (runCalibration)
      g_bCalRequest = true;
      g_calFlashUntil = millis() + CAL_FLASH_MS;
    } else {                              // PULSE: ضغطة قصيرة = تثبيت الخلية الحالية
      if (mode == MODE_SCAN) g_bStepRequest = true;
    }
    return;
  }
  // نقر على خلية في REVIEW → نافذة القيم
  if (mode == MODE_REVIEW && y >= UI_HEAT_Y && y < UI_HEAT_Y + UI_HEAT_H) {
    int gw, gh, cs, ox;
    xSemaphoreTake(xMutex, portMAX_DELAY);
    gw = g_gridW; gh = g_gridH;
    xSemaphoreGive(xMutex);
    heatGeom(gw, gh, cs, ox);
    // حارس قبل القسمة: إحداثي أصغر من الأصل يعطي بسطاً سالباً يُقسَّم نحو الصفر
    // فيُحسَب خطأً كخلية (0,0) — نتحقق أولاً أن النقرة داخل نطاق الخريطة
    if (x >= ox && y >= UI_HEAT_Y) {
      int gx = (x - ox) / cs;
      int gy = (y - UI_HEAT_Y) / cs;
      if (gx >= 0 && gx < gw && gy >= 0 && gy < gh) {
        g_popupX = gx; g_popupY = gy;
        g_popupUntil = millis() + POPUP_MS;
      }
    }
  }
}

// v9.3: ضغطة مطوّلة على CAL/SENS — تنقّل دائري في عتبات جاهزة + تلميح شاشة
static void cycleThreshold() {
  static const float PRE[] = {1, 2, 3, 5, 8, 12, 20, 30, 50};
  float next = PRE[0];
  for (unsigned i = 0; i < sizeof(PRE) / sizeof(PRE[0]); i++)
    if (PRE[i] > thresh + 0.01f) { next = PRE[i]; break; }
  thresh = next;
  g_threshHintUntil = millis() + THRESH_HINT_MS;
  Serial.printf("THRESH -> %.1f uT\n", (double)thresh);
}

static void onLongPress(int16_t x, int16_t y) {
  if (y < UI_FOOTER_Y) return;
  if (x >= 214) {                         // PULSE مطوّلة: تخطّي الخلية دون تسجيل
    if (g_mode == MODE_SCAN) g_bSkipCell = true;
  } else if (x >= 107) {                  // CAL مطوّلة: ضبط سريع للحساسية/العتبة
    cycleThreshold();
  }
}

void handleTouch() {
  static bool down = false;
  static uint32_t t0 = 0;
  static bool longFired = false;
  static int16_t lx = 0, ly = 0;

  if (M5.Touch.getCount() == 0 && !down) return;
  auto t = M5.Touch.getDetail();

  if (t.wasPressed()) {
    down = true; t0 = millis(); longFired = false;
    lx = t.x; ly = t.y;
  }
  if (down && t.isPressed()) {
    lx = t.x; ly = t.y;
    if (!longFired && millis() - t0 > LONG_PRESS_MS) {
      longFired = true;
      onLongPress(lx, ly);
    }
  }
  if (down && t.wasReleased()) {
    down = false;
    if (!longFired) onTap(lx, ly);
  }
}

// =============================================================
// setup / loop (SPEC §11)
// =============================================================
void setup() {
  Serial.begin(115200);
  // إسكات الضجيج التجميلي: نواة VFS تطبع [E] عند كل اختبار exists() في nextPath
  esp_log_level_set("vfs_api", ESP_LOG_NONE);

  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(1);              // 320×240
  M5.Display.setBrightness(128);
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(TFT_CYAN, TFT_BLACK);
  M5.Display.setCursor(10, 10);
  M5.Display.print("GeoScan3D v9.3");

  // ---- IMU (BMI270 الداخلي عبر M5Unified — v8.2) ----
  // M5.Imu.begin() يتم داخل M5.begin؛ نتحقق فقط من التوفر
  g_imuOK = M5.Imu.isEnabled();
  g_sensor.imuOK = g_imuOK ? 1 : 0;       // قبل بدء المهام — لا حاجة للقفل هنا
  Serial.println(g_imuOK ? "IMU OK" : "IMU MISSING");

  // v8.5.1: begin صريح لـ M5.Ex_I2C + فحص + استرداد بتبديل الأطراف — قبل أي كشف
  i2cPortABootDiagnostics();
  magInit();                             // v8.5: كشف LIS3MDL على PORT.A (اختياري — غياب صامت)

  xMutex = xSemaphoreCreateMutex();
  xLogQueue = xQueueCreate(LOG_QUEUE_LEN, LOG_LINE_LEN);

  // تحميل أوفست المعايرة من NVS
  Preferences prefs;
  prefs.begin(PREFS_NS, true);
  g_calOffsetV[0] = prefs.getFloat("cal0", 0.0f);
  g_calOffsetV[1] = prefs.getFloat("cal1", 0.0f);
  prefs.end();

  bleInit();

  // المهام: loggerTask(core0,prio2) / sensorTask(core1,prio3) — SPEC §11
  xTaskCreatePinnedToCore(loggerTask, "logger", 8192, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(sensorTask, "sensor", 4096, NULL, 3, NULL, 1);
}

void loop() {
  M5.update();

  // طلبات BLE/اللمس — آلة حالات الأوضاع هنا
  if (g_bStartRequest) { g_bStartRequest = false; if (g_mode != MODE_SCAN) startScan(); }
  if (g_bStopRequest)  { g_bStopRequest = false;  if (g_mode != MODE_LIVE) stopScan(); }
  if (g_bSkipCell)     {
    g_bSkipCell = false;
    // v9.0: طباعة إحداثيات الخلية المتخطّاة قبل التقدّم (serpentine)
    xSemaphoreTake(xMutex, portMAX_DELAY);
    int kx = -1, ky = -1;
    if (g_mode == MODE_SCAN && g_cellActive) cellCoord(g_cellIndex, kx, ky);
    xSemaphoreGive(xMutex);
    if (kx >= 0) Serial.printf("SKIP -> %d,%d\n", kx, ky);
    skipCell();
  }

#if SCAN_STEP_MODE
  // v8.3: استهلاك طلب النبضة — تسليح تجميع STEP_PULSE_MS (غير حاجب).
  // يُتجاهل بصمت خارج SCAN أو أثناء نبضة جارية.
  if (g_bStepRequest) {
    g_bStepRequest = false;
    xSemaphoreTake(xMutex, portMAX_DELAY);
    if (g_mode == MODE_SCAN && g_cellActive && !g_pulseActive) {
      resetCellAccum();
      g_pulseActive = true;
      g_pulseStartMs = millis();
      int sx, sy;
      cellCoord(g_cellIndex, sx, sy);
      Serial.printf("STEP -> pulse %d,%d\n", sx, sy);
    }
    xSemaphoreGive(xMutex);
  }
#endif

  // استهلاك طلب المعايرة بنفس النمط الحالي
  if (g_bCalRequest) { g_bCalRequest = false; runCalibration(); }

  handleTouch();
  pollMag();                              // v8.5: LIS3MDL (غير حاجب، ~20Hz) — ناقل PORT.A

  // v8.5: أمر MAGCAL — تصفير min/max ثم نافذة جمع 15s (دوّر الجهاز حول كل المحاور)
  if (g_bMagCalRequest) {
    g_bMagCalRequest = false;
    if (magPresent) {
      magMin[0] = magMin[1] = magMin[2] =  1e9f;
      magMax[0] = magMax[1] = magMax[2] = -1e9f;
      magCalActive = true;
      magCalStartMs = millis();
      Serial.println("MAGCAL START — rotate device 15s");
      if (g_bleConnected && !g_bleSending && g_bleTxChar) {
        const char* ok = "OK MAGCAL START\n";
        g_bleSending = true;
        g_bleTxChar->setValue((uint8_t*)ok, strlen(ok));
        g_bleTxChar->notify();
        g_bleSending = false;
      }
    }
  }

  // بث BLE: 5Hz في LIVE، 2Hz في SCAN/REVIEW — بنفس حارس المستخدم الحالي
  static uint32_t lastBle = 0;
  uint32_t bleInterval = (g_mode == MODE_LIVE) ? BLE_LIVE_MS : BLE_SCAN_MS;
  if (g_blePacketReady && g_bleConnected && !g_bleSending &&
      !g_dumpActive && millis() - lastBle >= bleInterval) {
    lastBle = millis();
    sendBlePacket();
  }
  pumpBleDump();
  serviceFileRequests();
#if TEXT_STREAM_ENABLE
  // v9.4: بث السطر النصي (بديل CH05) — مؤقّت مستقل عن الحزمة الثنائية حتى لا يزدحم مكدس BLE
  static uint32_t lastTxt = 0;
  uint32_t txtInterval = (g_mode == MODE_LIVE) ? TEXT_STREAM_LIVE_MS : TEXT_STREAM_SCAN_MS;
  if (g_textStreamOn && g_bleConnected && !g_bleSending && !g_dumpActive &&
      millis() - lastTxt >= txtInterval) {
    lastTxt = millis();
    sendBleTextLine();
  }
#endif
#if SCAN_STEP_MODE
  pumpStepResult();                     // v8.3: بث سطر نتيجة النبضة عند الجاهزية
#endif

  // رسم الواجهة مُخنّق ~10Hz (النمط الحالي)
  static uint32_t lastDraw = 0;
  if (millis() - lastDraw > UI_DRAW_MS) {
    lastDraw = millis();
    drawUI();
  }

  delay(20);                              // محفوظ من الكود الحالي
}
