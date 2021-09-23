#include "esp_camera.h"
#include "myconfig.h"
#include "camera_pins.h"

#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Stepper.h>

LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

#pragma region // DECLARE FEEDER VARIABLES

// Input Buttons
bool button1 = 0;
bool button2 = 0;
bool toggleButton1 = 0;
bool toggleButton2 = 0;

// Motor
const float STEPS_PER_REV = 32;
const float GEAR_RED = 64;
const float STEPS_PER_OUT_REV = STEPS_PER_REV * GEAR_RED;
int StepsRequired = 32;
Stepper steppermotor(STEPS_PER_REV, 12, 15, 13, 14);

int menuScreen = 0; // 0 = Clock, 1 = edit hour, 2 = edit minute, 3 = Feed time 1, 4 = Feed time 2, 5 = Feed Amount
int currentScreen = -1;

int sleepTime = 30;          // seconds
unsigned long timeSince = 0; // millis
bool lcdOn = false;

// Main Clock
int hour = 12;
int minutes = 0;
int minElapsed = 0;
int timeMod = 1000; // 1000 for 1 sec
bool pm = false;

// Feeder 1 Clock
int fHour1 = 12;
int fMin1 = 1;
bool fPm1 = false;

// Feeder 2 Clock
int fHour2 = 12;
int fMin2 = 1;
bool fPm2 = false;

// Feed Size and Timers
int fSize = 1; // S, M, L
long unsigned lastFeed = 0;
bool f1Timeout = true;
bool f2Timeout = true;
#pragma endregion

#pragma region // DECLARE CAM VARIABLES
char myName[] = CAM_NAME;

// This will be displayed to identify the firmware
char myVer[] PROGMEM = __DATE__ " @ " __TIME__;

// current rotation direction
char myRotation[5];

// Illumination LED's
#ifdef LAMP_DISABLE
int lampVal = -1; // lamp disabled by config
#elif LAMP_PIN
int lampVal = 0; // current lamp value, range 0-100, default off
#else
int lampVal = -1; // no lamp pin assigned
#endif

int lampChannel = 7;         // a free PWM channel (some channels used by camera)
const int pwmfreq = 50000;   // 50K pwm frequency
const int pwmresolution = 9; // duty cycle bit range
// https://diarmuid.ie/blog/pwm-exponential-led-fading-on-arduino-or-other-platforms
const int pwmIntervals = 100; // The number of Steps between the output being on and off
float lampR;                  // The R value in the PWM graph equation (calculated in setup)

#pragma endregion

void startCameraServer();

void setup()
{
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    Serial.println();
    Serial.println("====");
    Serial.print("esp32-cam-webserver: ");
    Serial.println(myName);
    Serial.print("Code Built: ");
    Serial.println(myVer);
    

    // initial rotation
    // can be set in myconfig.h
#define CAM_ROTATION 0

    // set the initialisation for image rotation
    // ToDo; might be better to handle this with an enum?
    int n __attribute__((unused)) = snprintf(myRotation, sizeof(myRotation), "%d", CAM_ROTATION);

#ifdef LED_PIN // If we have a notification LED set it to output
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LED_OFF);
#endif

#ifdef LAMP_PIN
    // ledcXXX functions are esp32 core pwm control functions
    ledcSetup(lampChannel, pwmfreq, pwmresolution); // configure LED PWM channel
    ledcWrite(lampChannel, 0);                      // Off by default
    ledcAttachPin(LAMP_PIN, lampChannel);           // attach the GPIO pin to the channel
    // Calculate the PWM scaling R factor:
    // https://diarmuid.ie/blog/pwm-exponential-led-fading-on-arduino-or-other-platforms
    lampR = (pwmIntervals * log10(2)) / (log10(pow(2, pwmresolution)));
#endif

#pragma region // SETUP PINS (CAM)
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
#pragma endregion

    //init with high specs to pre-allocate larger buffers
    if (psramFound())
    {
        config.frame_size = FRAMESIZE_UXGA;
        config.jpeg_quality = 10;
        config.fb_count = 2;
    }
    else
    {
        config.frame_size = FRAMESIZE_SVGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
    }

    // camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        Serial.printf("Camera init failed with error 0x%x", err);
        return;
    }

    sensor_t *s = esp_camera_sensor_get();

    //drop down frame size for higher initial frame rate
    s->set_framesize(s, FRAMESIZE_VGA);

#pragma region // CONNECT TO WIFI
    // Feedback that hardware init is complete and we are now attempting to connect
    Serial.println("Wifi Initialisation");
    flashLED(400);
    delay(100);

    // Connect ot Wifi Network
    Serial.print("Connecting to Wifi Network: ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    Serial.print("Connecting... ");
    bool usingAP = false;

    if (WiFi.status() != WL_CONNECTED)
    {                                // WiFi Failed
        for (int i = 0; i < 40; i++) // attept to connect for 10 seconds
        {
            delay(250);
            Serial.print("... ");
            if (WiFi.status() == WL_CONNECTED)
                break;
        }
        Serial.println();
        if (WiFi.status() != WL_CONNECTED)
        {
            Serial.println("WiFi Failed to Connect Creating AP...");
            usingAP = true;
            WiFi.softAP(APssid, APpassword);
            Serial.println("Setting up AccessPoint");
            Serial.print("SSID     : ");
            Serial.println(APssid);
            Serial.print("Password : ");
            Serial.println(APpassword);
        }
    }

    // feedback that we are connected
    Serial.println("WiFi connected");
    flashLED(200);
    delay(100);
    flashLED(200);
    delay(100);
    flashLED(200);

    // Start the Stream server, and the handler processes for the Web UI.
    startCameraServer();

    Serial.print("\nCamera Ready!  Use 'http://");
    if (usingAP)
        Serial.print(WiFi.softAPIP());
    else
        Serial.print(WiFi.localIP());

    Serial.println("' to connect");
    Serial.println();

#pragma endregion

#pragma region //  FEEDER SETUP
  Wire.begin(14, 15);
  lcd.init();
  lcd.backlight();

  pinMode(13, INPUT); // button 1
  pinMode(12, INPUT); // button 2
  steppermotor.setSpeed(700);
  #pragma endregion
}

// Notification LED 
void flashLED(int flashtime)
{                  // If we have it; flash it.
  digitalWrite(LED_PIN, LED_ON);  // On at full power.
  delay(flashtime);               // delay
  digitalWrite(LED_PIN, LED_OFF); // turn Off
} 

void loop()
{
  getInputs();
  if (button2)
    lcd.clear();
  if (button1)
    menuScreen++;
  if (menuScreen > 7)
    menuScreen = 0;

  if (lcdOn)
  {

  sleepTimer();

  setScreen();

  updateTime();
  }
  checkFeed();
}

#pragma region // Feeder Functions


void getInputs()
{
  button1 = digitalRead(13);
  button2 = digitalRead(12);

  if (button1 || button2)
    timeSince = millis();
  if (timeSince + (sleepTime * 1000) < millis())
  {
    int menuScreen = 0;
    button1 = false;
    toggleButton1 = true;
    button2 = false;
    toggleButton2 = true;
    return;
  }

  if (button1)
  {
    if (toggleButton1)
      button1 = false;
    else
      toggleButton1 = true;
  }
  else
    toggleButton1 = false;

  if (button2)
  {
    if (toggleButton2)
      button2 = false;
    else
      toggleButton2 = true;
  }
  else
    toggleButton2 = false;
}

void sleepTimer()
{
  if (timeSince + (sleepTime * 1000) > millis())
  {
    lcd.display();
    lcd.backlight();
    lcdOn = true;
  }
  else
  {
    lcd.noDisplay();
    lcd.noBacklight();
    lcdOn = false;
  }
}

void setScreen()
{
  lcd.setCursor(0, 0);
  switch (menuScreen)
  {
  case 0:
    if (menuScreen != currentScreen)
    {
      currentScreen = 0;
      lcd.clear();
    }
    lcd.setCursor(6, 0);
    lcd.print("Time");

    lcd.setCursor(4, 1);
    displayClock();
    break;
  case 1:
    if (menuScreen != currentScreen)
    {
      currentScreen = 1;
      lcd.clear();
    }
    lcd.setCursor(3, 0);
    lcd.print("Edit  Hour");
    editClock(true);
    lcd.setCursor(4, 1);
    displayClock();

    break;
  case 2:
    if (menuScreen != currentScreen)
    {
      currentScreen = 2;
      lcd.clear();
    }
    lcd.setCursor(2, 0);
    lcd.print("Edit  Minute");
    editClock(false);
    lcd.setCursor(4, 1);
    displayClock();
    break;
  case 3:
    if (menuScreen != currentScreen)
    {
      currentScreen = 3;
      lcd.clear();
    }
    lcd.setCursor(2, 0);
    lcd.print("Feed #1 Hour");
    updateFeed(0);
    lcd.setCursor(4, 1);
    displayFeed(1);
    break;
  case 4:
    if (menuScreen != currentScreen)
    {
      currentScreen = 4;
      lcd.clear();
    }
    lcd.setCursor(1, 0);
    lcd.print("Feed #1 Minute");
    updateFeed(1);
    lcd.setCursor(4, 1);
    displayFeed(1);
    break;
  case 5:
    if (menuScreen != currentScreen)
    {
      currentScreen = 5;
      lcd.clear();
    }
    lcd.setCursor(2, 0);
    lcd.print("Feed #2 Hour");
    updateFeed(2);
    lcd.setCursor(4, 1);
    displayFeed(0);
    break;
  case 6:
    if (menuScreen != currentScreen)
    {
      currentScreen = 6;
      lcd.clear();
    }
    lcd.setCursor(1, 0);
    lcd.print("Feed #2 Minute");
    updateFeed(3);
    lcd.setCursor(4, 1);
    displayFeed(0);
    break;
  case 7:
    if (menuScreen != currentScreen)
    {
      currentScreen = 7;
      lcd.clear();
    }
    lcd.setCursor(2, 0);
    lcd.print("Feed  Amount");

    feedSize();
    break;
  }
}

void updateTime()
{
  if ((millis() / timeMod) > 60 * (minElapsed + 1))
  {
    minutes++;
    minElapsed++;
    if (hour == 12)
      pm = !pm;
  }
  if (minutes == 60)
  {
    minutes = 0;
    hour = hour + 1;
  }

  if (hour == 13)
    hour = 1;
}

void displayClock()
{
  if (hour < 10)
  {
    lcd.print(0);
  }
  lcd.print(hour);
  lcd.print(":");
  if (minutes < 10)
  {
    lcd.print(0);
  }
  lcd.print(minutes);

  if (pm)
    lcd.print("-PM");
  else
    lcd.print("-AM");
}

void editClock(bool unit)
{
  if (button2)
    unit ? hour++ : minutes++;
  if (button2 && hour == 12 && unit)
    pm = !pm;
}

void updateFeed(int feedNum)
{
  if (!button2)
    return;
  switch (feedNum)
  {
  case 0:
    fHour1++;
    if (fHour1 == 12)
      fPm1 = !fPm1;
    break;
  case 1:
    fMin1++;
    break;
  case 2:
    fHour2++;
    if (fHour2 == 12)
      fPm2 = !fPm2;
    break;
  case 3:
    fMin2++;
    break;
  }
}

void displayFeed(bool feedNum)
{
  int fHour, fMin, fPm;
  if (feedNum)
  {
    fHour = fHour1;
    fMin = fMin1;
    fPm = fPm1;
  }
  else
  {
    fHour = fHour2;
    fMin = fMin2;
    fPm = fPm2;
  }

  if (fMin == 60)
    fMin = 0;
  if (fHour == 13)
    fHour = 1;

  if (fHour < 10)
  {
    lcd.print(0);
  }
  lcd.print(fHour);
  lcd.print(":");
  if (fMin < 10)
  {
    lcd.print(0);
  }
  lcd.print(fMin);

  if (fPm)
    lcd.print("-PM");
  else
    lcd.print("-AM");

  if (feedNum)
  {
    fHour1 = fHour;
    fMin1 = fMin;
    fPm1 = fPm;
  }
  else
  {
    fHour2 = fHour;
    fMin2 = fMin;
    fPm2 = fPm;
  }
}

void feedSize()
{
  if (button2)
    (fSize == 3) ? fSize = 1 : fSize++;

  lcd.setCursor(5, 1);
  switch (fSize)
  {
  case 1:
    lcd.print("Small");
    break;
  case 2:
    lcd.print("Medium");
    break;
  case 3:
    lcd.print("Large");
    break;
  }
}

void checkFeed()
{
 if (!f1Timeout && hour == fHour1 && minutes == fMin1)
  {
    f1Timeout = true;
    lastFeed = millis();
    StepsRequired  =  - fSize * STEPS_PER_OUT_REV;
    steppermotor.step(StepsRequired);
  }
  if (hour == fHour1 && minutes == (fMin1 + 1))
    f1Timeout = false;

  if (fHour1 == fHour2 && fMin1 == fMin2 && fPm1 == fPm2)
    return; // don't feed again if they're set to the same time

  if (!f2Timeout && hour == fHour2 && minutes == fMin2)
  {
    f2Timeout = true;
    lastFeed = millis();
    StepsRequired  =  - fSize * STEPS_PER_OUT_REV;
    steppermotor.step(StepsRequired);
  }
  if (hour == fHour2 && minutes == (fMin2 + 1))
    f1Timeout = false;
}

#pragma endregion


// TODO I2C for camera and display
