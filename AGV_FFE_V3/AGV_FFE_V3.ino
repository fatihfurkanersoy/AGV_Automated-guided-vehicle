/// AGV CODE FATIHFURKAN VERSION 3.0

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// A MOTOR
#define APWM 12
#define ADIR 10
#define ADIRFW 8
// B MOTOR
#define BPWM 46
#define BDIR 48
#define BDIRFW 30
// C MOTOR
#define CPWM 45
#define CDIR 25
#define CDIRFW 23
// D MOTOR
#define DPWM 13
#define DDIR 11
#define DDIRFW 9
// LINE FOLLOWING SENSOR
#define SENSOR_PIN_RIGHT_1 A6
#define SENSOR_PIN_RIGHT_2 A7
#define SENSOR_PIN_RIGHT_3 A1
#define SENSOR_PIN_RIGHT_4 A3
#define SENSOR_PIN_RIGHT_5 A5
#define SENSOR_PIN_RIGHT_6 A2
#define SENSOR_PIN_RIGHT_7 A4

#define SENSOR_PIN_LEFT_1 A12
#define SENSOR_PIN_LEFT_2 32
#define SENSOR_PIN_LEFT_3 A8
#define SENSOR_PIN_LEFT_4 A9
#define SENSOR_PIN_LEFT_5 A11
#define SENSOR_PIN_LEFT_6 A10
#define SENSOR_PIN_LEFT_7 A13

#define SENSOR_PIN_FRONT_1 16
#define SENSOR_PIN_FRONT_2 15
#define SENSOR_PIN_FRONT_3 50
#define SENSOR_PIN_FRONT_4 52
#define SENSOR_PIN_FRONT_5 51
#define SENSOR_PIN_FRONT_6 53
#define SENSOR_PIN_FRONT_7 14

#define SENSOR_PIN_BACK_1 49
#define SENSOR_PIN_BACK_2 47
#define SENSOR_PIN_BACK_3 34
#define SENSOR_PIN_BACK_4 36
#define SENSOR_PIN_BACK_5 38
#define SENSOR_PIN_BACK_6 40
#define SENSOR_PIN_BACK_7 42
// LINE FOLLOWING SENSOR ENABLE PIN
// CONTROL BUTTON PIN
#define CONTROL_BUTTON_PIN 27
// STANBY
#define STANBY_PIN 6
// CONTROL PANEL BUTTONS
#define ILERI_PIN 43
#define GERI_PIN 41
#define SAGA_PIN 7
#define SOLA_PIN 44
#define SOLA_DON_PIN 29
#define SAGA_DON_PIN 31
// STATİONS BUTTONS
#define ANALOG_PIN_BUTTON A0
// BATTERY
#define VBATTADC 14
#define BATTERY_AVARAGE_READ_COUNT 16
// AMPERAGE PIN
#define AMPERAGE_PIN A15
// RGB
#define RED_PIN 2
#define GREEN_PIN 3
#define BLUE_PIN 4
// BUZZER
#define BUZZER_PIN 5
// PWM VALUE
#define PWM_START 255
#define PWM_STOP 0
// LİDAR WARNING PIOUT
#define WARN_LIDAR11 35
#define WARN_LIDAR21 39

#define SENSOR_CONFIDENCE 5

// esp1 - 39 , esp2 - 35 pin
// MILLIS
unsigned long oldTime = 0;
unsigned long newTime;
unsigned long oldTime1 = 0;
unsigned long newTime1;
unsigned long oldTime2 = 0;
unsigned long newTime2;
unsigned long oldTime3 = 0;
unsigned long newTime3;
unsigned long oldTime4 = 0;
unsigned long newTime4;
unsigned long oldTime5 = 0;
unsigned long newTime5;
unsigned long oldTime6 = 0;
unsigned long newTime6;
int buzzerCount = 0;
#define FRONT_SENSOR 0
#define BACK_SENSOR 1
#define RIGHT_SENSOR 2
#define LEFT_SENSOR 3
int chargeStatus = -1;
long TIME_NOW1 = 0;
long TIME_NOW2 = 0;
long TIME_NOW3 = 0;

unsigned long adc_millis1 = 0;
unsigned long adc_millis2 = 0;
unsigned long adc_millis3 = 0;

LiquidCrystal_I2C lcd(0x27, 20, 4);


// panel buttonarının durumlarını okuyup gösteriyoruz
int ILERI_TRY = -1;
int GERI_TRY = -1;
int SAGA_TRY = -1;
int SOLA_TRY = -1;
int SAGA_DON = -1;
int SOLA_DON = -1;
// istasyon butonunun durumlarını okuyup gösteriyoruz
int ANALOG_GIT_BUTTON = -1;
// Sensor Pinlerini durumlarını okuyup gösteriyoruz
int SensorFront1 = -1;
int SensorFront2 = -1;
int SensorFront3 = -1;
int SensorFront4 = -1;
int SensorFront5 = -1;
int SensorFront6 = -1;
int SensorFront7 = -1;
int SensorBack1 = -1;
int SensorBack2 = -1;
int SensorBack3 = -1;
int SensorBack4 = -1;
int SensorBack5 = -1;
int SensorBack6 = -1;
int SensorBack7 = -1;
int SensorRight1 = -1;
int SensorRight2 = -1;
int SensorRight3 = -1;
int SensorRight4 = -1;
int SensorRight5 = -1;
int SensorRight6 = -1;
int SensorRight7 = -1;
int SensorLeft1 = -1;
int SensorLeft2 = -1;
int SensorLeft3 = -1;
int SensorLeft4 = -1;
int SensorLeft5 = -1;
int SensorLeft6 = -1;
int SensorLeft7 = -1;

// SENSOR GRUP VALUE
int SAG_ON_SENSORLER = -1;
int SAG_ORTA_SENSORLER = -1;
int SAG_ARKA_SENSORLER = -1;
//
int SOL_ON_SENSORLER = -1;
int SOL_ORTA_SENSORLER = -1;
int SOL_ARKA_SENSORLER = -1;
//
int ON_SOL_SENSORLER = -1;
int ON_SAG_SENSORLER = -1;
int ON_ORTA_SENSORLER = -1;
//
int ARKA_SOL_SENSORLER = -1;
int ARKA_SAG_SENSORLER = -1;
int ARKA_ORTA_SENSORLER = -1;

int rgbState = -1;
int buzzerStateLoop = -1;
int Warn_Lidar11 = -1;
int Warn_Lidar21 = -1;
int standbyCounter = 60;
int stationStatus = -1;
int controlButtonState = -1;
int lidarControl = -1;

// ----------------------------------------------------------------------------

// ----------------------------------------SETUP------------------------------------
void setup()
{
  Wire.begin();
  Serial.begin(115200);

  // MOTOR PİNMODE
  pinMode(APWM, OUTPUT);
  pinMode(BPWM, OUTPUT);
  pinMode(CPWM, OUTPUT);
  pinMode(DPWM, OUTPUT);
  analogWrite(APWM, 0);
  analogWrite(BPWM, 0);
  analogWrite(CPWM, 0);
  analogWrite(DPWM, 0);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("      OMNIMAN       ");


  lcd.setCursor(0, 1);
  lcd.print("   12/9/22   v3.0   ");

  lcd.setCursor(0, 3);
  lcd.print("  ORBITRO  ROBOTIC  ");

  delay(1000);
  // RGB PİNMODE
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // VOLTAGE SENSOR PİNMODE
  pinMode(VBATTADC, INPUT);
  // AMPERAGE PİNMODE
  pinMode(AMPERAGE_PIN, INPUT);
  // CONTROL BUTTON PİNMODE
  pinMode(ILERI_PIN, INPUT_PULLUP);
  pinMode(GERI_PIN, INPUT_PULLUP);
  pinMode(SAGA_PIN, INPUT_PULLUP);
  pinMode(SOLA_PIN, INPUT_PULLUP);
  pinMode(SOLA_DON_PIN, INPUT_PULLUP);
  pinMode(SAGA_DON_PIN, INPUT_PULLUP);
  // ANALOG PİNMODE
  pinMode(ANALOG_PIN_BUTTON, INPUT);
  // MOTOR DİRECTİON PİNMODE
  pinMode(ADIR, OUTPUT);
  pinMode(BDIR, OUTPUT);
  pinMode(CDIR, OUTPUT);
  pinMode(DDIR, OUTPUT);
  pinMode(ADIRFW, OUTPUT);
  pinMode(BDIRFW, OUTPUT);
  pinMode(CDIRFW, OUTPUT);
  pinMode(DDIRFW, OUTPUT);
  // LINE FOLLOWING SENSOR PİNMODE
  pinMode(SENSOR_PIN_FRONT_1, INPUT_PULLUP);
  pinMode(SENSOR_PIN_FRONT_2, INPUT_PULLUP);
  pinMode(SENSOR_PIN_FRONT_3, INPUT_PULLUP);
  pinMode(SENSOR_PIN_FRONT_4, INPUT_PULLUP);
  pinMode(SENSOR_PIN_FRONT_5, INPUT_PULLUP);
  pinMode(SENSOR_PIN_FRONT_6, INPUT_PULLUP);
  pinMode(SENSOR_PIN_FRONT_7, INPUT_PULLUP);
  pinMode(SENSOR_PIN_BACK_1, INPUT_PULLUP);
  pinMode(SENSOR_PIN_BACK_2, INPUT_PULLUP);
  pinMode(SENSOR_PIN_BACK_3, INPUT_PULLUP);
  pinMode(SENSOR_PIN_BACK_4, INPUT_PULLUP);
  pinMode(SENSOR_PIN_BACK_5, INPUT_PULLUP);
  pinMode(SENSOR_PIN_BACK_6, INPUT_PULLUP);
  pinMode(SENSOR_PIN_BACK_7, INPUT_PULLUP);
  pinMode(SENSOR_PIN_RIGHT_1, INPUT_PULLUP);
  pinMode(SENSOR_PIN_RIGHT_2, INPUT_PULLUP);
  pinMode(SENSOR_PIN_RIGHT_3, INPUT_PULLUP);
  pinMode(SENSOR_PIN_RIGHT_4, INPUT_PULLUP);
  pinMode(SENSOR_PIN_RIGHT_5, INPUT_PULLUP);
  pinMode(SENSOR_PIN_RIGHT_6, INPUT_PULLUP);
  pinMode(SENSOR_PIN_RIGHT_7, INPUT_PULLUP);
  pinMode(SENSOR_PIN_LEFT_1, INPUT_PULLUP);
  pinMode(SENSOR_PIN_LEFT_2, INPUT_PULLUP);
  pinMode(SENSOR_PIN_LEFT_3, INPUT_PULLUP);
  pinMode(SENSOR_PIN_LEFT_4, INPUT_PULLUP);
  pinMode(SENSOR_PIN_LEFT_5, INPUT_PULLUP);
  pinMode(SENSOR_PIN_LEFT_6, INPUT_PULLUP);
  pinMode(SENSOR_PIN_LEFT_7, INPUT_PULLUP);

  // LIDAR WARNING PİNMODE
  // pinMode(WARN_LIDAR11, INPUT);
  // pinMode(WARN_LIDAR21, INPUT);

  pinMode(WARN_LIDAR11, INPUT_PULLUP);
  pinMode(WARN_LIDAR21, INPUT_PULLUP);

  // MOTOR DİRECTİON SETUP
  digitalWrite(ADIR, HIGH);
  digitalWrite(ADIRFW, LOW);
  digitalWrite(BDIR, HIGH);
  digitalWrite(BDIRFW, LOW);
  digitalWrite(CDIR, HIGH);
  digitalWrite(CDIRFW, LOW);
  digitalWrite(DDIR, HIGH);
  digitalWrite(DDIRFW, LOW);
  // BUTTON ÜZERİNDE  OLAN ANAHTAR
  pinMode(CONTROL_BUTTON_PIN, INPUT_PULLUP);
  // STANBY PİN
  pinMode(STANBY_PIN, OUTPUT);
  // START ROBOT
  standbyCounter = 60;

  delay(1000);

  digitalWrite(STANBY_PIN, HIGH);
}

// ----------------------------------------------------------------------------

// ----------------------------------------LOOP------------------------------------
void loop()
{

  // rgbController-- 1 white, 2 red,  3 green,  4 blue,  5 purple,  6 cyan,  7 yellow, 8 stop)

  rgbStatus(rgbState);
  buzzerFlipFlop(buzzerStateLoop);
  if (ANALOG_GIT_BUTTON >= 1 && ANALOG_GIT_BUTTON <= 20)
  {
    controlButtonState = LOW; // MANUEL
  }

  else if (ANALOG_GIT_BUTTON >= 30 && ANALOG_GIT_BUTTON <= 70)
  {                            // OTONOM CONTROL
    controlButtonState = HIGH; // OTONOM
  }

  TIME_NOW2 = millis();
  if (TIME_NOW2 - adc_millis2 > 10)
  {
     SENSOR_VALUE_PRINT();/////////////////////////////////////////////////////////////////////SENSOR_VALUE_PRINT////////////////////////////
    //Serial.println(vBattRead());
    ILERI_TRY = digitalRead(ILERI_PIN);
    GERI_TRY = digitalRead(GERI_PIN);
    SAGA_TRY = digitalRead(SAGA_PIN);
    SOLA_TRY = digitalRead(SOLA_PIN);
    SAGA_DON = digitalRead(SAGA_DON_PIN);
    SOLA_DON = digitalRead(SOLA_DON_PIN);
    ANALOG_GIT_BUTTON = analogRead(ANALOG_PIN_BUTTON);
    adc_millis2 = TIME_NOW2;
  }

  // STANDBY
  newTime2 = millis();
  if (newTime2 - oldTime2 > 5000)
  {
    if (standbyCounter != 0)
    {
      standbyCounter--;
    }
    if (standbyCounter == 0)
    {
      digitalWrite(STANBY_PIN, LOW);
    }
    else if (standbyCounter > 0)
    {
      digitalWrite(STANBY_PIN, HIGH);
    }
    oldTime2 = newTime2;
  }

  if (amparageRead() > 3)
  {
    chargeStatus = 1;
    rgbState = 9;
    buzzerStateLoop = 1;
    standbyCounter = 60;
  }

  else if (amparageRead() < 3)
  {
    chargeStatus = 0;
  }

  if (chargeStatus == 0)
  {
    if (controlButtonState == HIGH) // OTONOM CODE
    {
      if (digitalRead(WARN_LIDAR11) == HIGH && digitalRead(WARN_LIDAR21) == HIGH)
      {
        if (ANALOG_GIT_BUTTON >= 740 && ANALOG_GIT_BUTTON <= 857) //İleri gitme otonom
        {
          if (!(lineFrontSensorValue() & 0b1111111) == 0)
          {
            RotateWheels(false, true, false, false, false, false);

            rgbState = 5;
            buzzerStateLoop = 3;
            standbyCounter = 60;

            if (((lineFrontSensorValue_ON_ORTA_VALUE() * lineFrontSensorValue_ON_SOL_VALUE()) == 1) || lineFrontSensorValue_ON_SOL_VALUE() == 1) // burda 2 3 2 şeklinde gruplanan sensörlerden aracın onundekı sensorlerın ortası ve solundakılerden gelen verıler 1 e esit mi bakıyoruz.
            {
              PwmStraigtLeft(PWM_START); // arac sollu sekilde one dogru gidiyor
            }
            else if (((lineFrontSensorValue_ON_ORTA_VALUE() * lineFrontSensorValue_ON_SAG_SENSOR()) == 1) || lineFrontSensorValue_ON_SAG_SENSOR() == 1)
            {
              PwmStraigtRight(PWM_START); // arac saglı sekilde one dogru gidiyor
            }
            else if ((lineFrontSensorValue_ON_ORTA_VALUE() == 1))
            {
              Pwm(PWM_START);
            }
          }
          else
          {
            Pwm(PWM_STOP);
            rgbState = 3;
            buzzerStateLoop = 1;
          }
        }

        else if (ANALOG_GIT_BUTTON >= 863 && ANALOG_GIT_BUTTON <= 980) // Geri gitme otonom
        {

          if (!(lineBackSensorValue() & 0b1111111) == 0)
          {
            RotateWheels(true, false, false, false, false, false);
            rgbState = 5;
            buzzerStateLoop = 3;
            standbyCounter = 60;

            if (((lineBackSensorValue_ARKA_ORTA_VALUE() * lineBackSensorValue_ARKA_SOL_VALUE()) == 1) || lineBackSensorValue_ARKA_SOL_VALUE() == 1)
            {
              PwmStraigtRight(PWM_START); //
            }
            else if (((lineBackSensorValue_ARKA_ORTA_VALUE() * lineBackSensorValue_ARKA_SAG_SENSOR()) == 1) || lineBackSensorValue_ARKA_SAG_SENSOR() == 1)
            {
              PwmStraigtLeft(PWM_START); //
            }
            else if ((lineBackSensorValue_ARKA_ORTA_VALUE() == 1))
            {
              Pwm(PWM_START);
            }
          }
          else
          {
            Pwm(PWM_STOP);
            rgbState = 3;
            buzzerStateLoop = 1;
          }
        }

        else if (ANALOG_GIT_BUTTON >= 490 && ANALOG_GIT_BUTTON <= 610) // SAG YENGEC gitme otonom
        {

          if (!(lineRightSensorValue() & 0b1111111) == 0)
          {
            RotateWheels(false, false, false, true, false, false);
            rgbState = 4;
            buzzerStateLoop = 3;
            standbyCounter = 60;

            if (((lineRightSensor_SAG_ORTA_VALUE() * lineRightSensor_SAG_ON_VALUE()) == 1) || lineRightSensor_SAG_ON_VALUE() == 1)
            {
              PwmLateralLeft(PWM_START); // SAG ARKA TARAFI TOPLAMAK LAZIM
              Serial.println("SAG ARKA TARAFI TOPARLA");
            }
            else if (((lineRightSensor_SAG_ORTA_VALUE() * lineRightSensor_SAG_ARKA_SENSOR()) == 1) || lineRightSensor_SAG_ARKA_SENSOR() == 1)
            {
              PwmLateralRight(PWM_START); // SAG ON TARAFI TOPLAMAK LAZIM cunku arka sensorler calismis yani onu toparlıcaz
              Serial.println("SAG ON TARAFI TOPLAMAK LAZIM");
            }
            else if ((lineRightSensor_SAG_ORTA_VALUE() == 1))
            {
              Pwm(PWM_START);
              Serial.println("SAG ORTA GIT");
            }
          }

          else
          {
            Pwm(PWM_STOP);
            rgbState = 3;
            buzzerStateLoop = 1;
          }
        }

        else if (ANALOG_GIT_BUTTON >= 620 && ANALOG_GIT_BUTTON <= 745) // SOL YENGEC gitme otonom
        {

          if (!(lineLeftSensorValue() & 0b1111111) == 0)
          {
            RotateWheels(false, false, true, false, false, false);
            rgbState = 4;
            buzzerStateLoop = 3;
            standbyCounter = 60;

            if (((lineLeftSensor_SOL_ORTA_VALUE() * lineLeftSensor_SOL_ON_VALUE()) == 1) || lineLeftSensor_SOL_ON_VALUE() == 1)
            {
              PwmLateralRight(PWM_START); // SAG ARKA TARAFI TOPLAMAK LAZIM
              Serial.println("sol ARKA TOPARLA");
            }
            else if (((lineLeftSensor_SOL_ORTA_VALUE() * lineLeftSensor_SOL_ARKA_SENSOR()) == 1) || lineLeftSensor_SOL_ARKA_SENSOR() == 1)
            {

              PwmLateralLeft(PWM_START);
              Serial.println("sol ON TOPARLA");
              // SAG ON TARAFI TOPLAMAK LAZIM cunku arka sensorler calismis yani onu toparlıcaz
            }
            else if ((lineLeftSensor_SOL_ORTA_VALUE() == 1))
            {
              Pwm(PWM_START);
              Serial.println("SOL ORTA GIT");
            }
          }

          else
          {
            Pwm(PWM_STOP);
            rgbState = 3;
            buzzerStateLoop = 1;
          }
        }

        else
        {

          Pwm(PWM_STOP);
          if (vBattRead() < 22)
          {
            rgbState = 7;
          }
          else
          {
            rgbState = 3;
          }
          buzzerStateLoop = 1;
        }
      }

      else // LIDAR'lardan biri bir sey algılaması durumunda
      {
        Pwm(PWM_STOP);
        rgbState = 2;
        buzzerStateLoop = 2;
      }
    }
    else if (controlButtonState == LOW) // MANUEL CODE
    {

      if (ANALOG_GIT_BUTTON >= 863 && ANALOG_GIT_BUTTON <= 980) // GERİ
      {
        RotateWheels(true, false, false, false, false, false);
        rgbState = 5;
        Pwm(PWM_START);
        buzzerStateLoop = 3;
        standbyCounter = 60;

        // Serial.println("GERI");
      }
      else if (ANALOG_GIT_BUTTON >= 750 && ANALOG_GIT_BUTTON <= 857) // ILERI
      {
        RotateWheels(false, true, false, false, false, false);
        rgbState = 5;
        Pwm(PWM_START);
        buzzerStateLoop = 3;
        standbyCounter = 60;

        // Serial.println("ILERI");
      }
      else if (ANALOG_GIT_BUTTON >= 300 && ANALOG_GIT_BUTTON <= 410) // SAGA DONUS
      {
        RotateWheels(false, false, false, false, true, false);
        rgbState = 4;
        Pwm(PWM_START / 4);
        buzzerStateLoop = 3;
        standbyCounter = 60;
      }
      else if (ANALOG_GIT_BUTTON >= 175 && ANALOG_GIT_BUTTON <= 280) // SOLA DONUS
      {

        RotateWheels(false, false, false, false, false, true);
        rgbState = 4;
        Pwm(PWM_START / 4);
        buzzerStateLoop = 3;
        standbyCounter = 60;

        // Serial.println("SOLA");
      }
      else if (ANALOG_GIT_BUTTON >= 495 && ANALOG_GIT_BUTTON <= 600) // SAGA YENGEC

      {
        RotateWheels(false, false, false, true, false, false);
        rgbState = 4;
        Pwm(PWM_START);
        buzzerStateLoop = 3;
        standbyCounter = 60;

        // Serial.println("SOLA_DON");
      }
      else if (ANALOG_GIT_BUTTON >= 620 && ANALOG_GIT_BUTTON <= 735) // SOLA YENGEC

      {
        RotateWheels(false, false, true, false, false, false);
        rgbState = 4;
        Pwm(PWM_START);
        buzzerStateLoop = 3;
        standbyCounter = 60;

        // Serial.println("SAGA_DON");
      }
      else if (ILERI_TRY == LOW || GERI_TRY == LOW || SAGA_TRY == LOW || SOLA_TRY == LOW || SAGA_DON == LOW || SOLA_DON == LOW)
      {

        FORWARD_MANUAL_BUTTON();
        BACK_MANUAL_BUTTON();
        RIGHT_MANUAL_BUTTON();
        LEFT_MANUAL_BUTTON();
        TURN_LEFT_MANUAL_BUTTON();
        TURN_RIGHT_MANUAL_BUTTON();
      }

      else
      {
        // RotateWheels(false, false, false, false, false, false);
        // rgbController(true, false, false, false,false,false,false,false);
        if (digitalRead(WARN_LIDAR11) == LOW || digitalRead(WARN_LIDAR21) == LOW)
        {
          rgbState = 2;
          buzzerStateLoop = 2;
        }
        else
        {
          buzzerStateLoop = 1;

          if (vBattRead() < 22)
          {
            rgbState = 7;
          }
          else
          {
            rgbState = 3;
          }
        }

        Pwm(PWM_STOP);
      }
    }

    else
    {
      // Serial.println("CONTROL BUTON STATE ERROR");
    }
  }
}

//-------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------
// FUNCTIONS
// ------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------

void FORWARD_MANUAL_BUTTON()
{
  if (ILERI_TRY == LOW)
  {
    RotateWheels(false, true, false, false, false, false);
    rgbState = 5;
    Pwm(PWM_START);
    buzzerStateLoop = 3;
    standbyCounter = 60;
  }
}

void BACK_MANUAL_BUTTON()
{
  if (GERI_TRY == LOW) // geri
  {
    RotateWheels(true, false, false, false, false, false);
    rgbState = 5;
    Pwm(PWM_START);
    buzzerStateLoop = 3;
    standbyCounter = 60;
  }
}

void TURN_RIGHT_MANUAL_BUTTON()
{
  if (SAGA_TRY == LOW) // saga donus
  {
    RotateWheels(false, false, false, false, true, false);
    rgbState = 4;
    Pwm(PWM_START / 4);
    buzzerStateLoop = 3;
    standbyCounter = 60;
  }
}

void TURN_LEFT_MANUAL_BUTTON()
{
  if (SOLA_TRY == LOW) // SOLA DONUS
  {
    RotateWheels(false, false, false, false, false, true);
    rgbState = 4;
    Pwm(PWM_START / 4);
    buzzerStateLoop = 3;
    standbyCounter = 60;
  }
}

void RIGHT_MANUAL_BUTTON()
{
  if (SAGA_DON == LOW) // SAGA YENGEC
  {
    RotateWheels(false, false, false, true, false, false);
    rgbState = 4;
    Pwm(PWM_START);
    buzzerStateLoop = 3;
    standbyCounter = 60;
  }
}

void LEFT_MANUAL_BUTTON()
{
  if (SOLA_DON == LOW) // SOLA YENGEC
  {
    RotateWheels(false, false, true, false, false, false);
    rgbState = 4;
    Pwm(PWM_START);
    buzzerStateLoop = 3;
    standbyCounter = 60;
  }
}

void SENSOR_VALUE_PRINT()
{

  Serial.print("  FRONT ");
  for (int j = 6; j >= 0; j--)
  {
    if (((lineFrontSensorValue() >> j) & 1) == 1)
    {
      Serial.print("1");
    }
    else
    {
      Serial.print("0");
    }
  }
  Serial.print("  BACK ");
  for (int j = 6; j >= 0; j--)
  {
    if (((lineBackSensorValue() >> j) & 1) == 1)
    {
      Serial.print("1");
    }
    else
    {
      Serial.print("0");
    }
  }
  Serial.print(" RIGHT ");
  for (int j = 6; j >= 0; j--)
  {
    if (((lineRightSensorValue() >> j) & 1) == 1)
    {
      Serial.print("1");
    }
    else
    {
      Serial.print("0");
    }
  }
  Serial.print(" LEFT ");
  for (int j = 6; j >= 0; j--)
  {
    if (((lineLeftSensorValue() >> j) & 1) == 1)
    {
      Serial.print("1");
    }
    else
    {
      Serial.print("0");
    }
  }

  Serial.println();
}

void PwmStraigtRight(int pwm_value)
{
  analogWrite(APWM, pwm_value/2);
  analogWrite(BPWM, pwm_value/2);
  analogWrite(CPWM, pwm_value / 4);
  analogWrite(DPWM, pwm_value / 4);
}

void PwmStraigtLeft(int pwm_value)
{
  analogWrite(APWM, pwm_value / 4);
  analogWrite(BPWM, pwm_value / 4);
  analogWrite(CPWM, pwm_value / 2);
  analogWrite(DPWM, pwm_value / 2);
}

void PwmLateralRight(int pwm_value)
{
  analogWrite(APWM, pwm_value / 4);
  analogWrite(BPWM, pwm_value / 2);
  analogWrite(CPWM, pwm_value / 2);
  analogWrite(DPWM, pwm_value / 4);
}

void PwmLateralLeft(int pwm_value)
{
  analogWrite(APWM, pwm_value / 2);
  analogWrite(BPWM, pwm_value / 4);
  analogWrite(CPWM, pwm_value / 4);
  analogWrite(DPWM, pwm_value / 2);
}

void Pwm(int pwm_value)
{
  analogWrite(APWM, pwm_value);
  analogWrite(BPWM, pwm_value);
  analogWrite(CPWM, pwm_value);
  analogWrite(DPWM, pwm_value);
}

void rgbStatus(int rgbState)
{
  if (rgbState == 1)
  {
    rgbController(true, false, false, false, false, false, false, false);
  }
  else if (rgbState == 2)
  {
    rgbController(false, true, false, false, false, false, false, false);
  }
  else if (rgbState == 3)
  {
    rgbController(false, false, true, false, false, false, false, false);
  }
  else if (rgbState == 4)
  {
    rgbController(false, false, false, true, false, false, false, false);
  }
  else if (rgbState == 5)
  {

    rgbController(false, false, false, false, true, false, false, false);
  }
  else if (rgbState == 6)
  {
    rgbController(false, false, false, false, false, true, false, false);
  }
  else if (rgbState == 7)
  {
    rgbController(false, false, false, false, false, false, true, false);
  }
  else if (rgbState == 8)
  {
    rgbController(false, false, false, false, false, false, false, true);
  }
  else if (rgbState == 9)
  {
    chargerRgbStatus();
  }
  else if (rgbState == 10)
  {
    analogWrite(RED_PIN, 40);
  }
}

void RotateWheels(bool ileri, bool geri, bool sag, bool sol, bool soldon, bool sagdon)
{
  if (ileri)
  {
    digitalWrite(ADIR, HIGH);
    digitalWrite(ADIRFW, LOW);
    digitalWrite(BDIR, LOW);
    digitalWrite(BDIRFW, HIGH);
    digitalWrite(CDIR, LOW);
    digitalWrite(CDIRFW, HIGH);
    digitalWrite(DDIR, HIGH);
    digitalWrite(DDIRFW, LOW);
  }
  else if (geri)
  {
    digitalWrite(ADIR, LOW);
    digitalWrite(ADIRFW, HIGH);
    digitalWrite(BDIR, HIGH);
    digitalWrite(BDIRFW, LOW);
    digitalWrite(CDIR, HIGH);
    digitalWrite(CDIRFW, LOW);
    digitalWrite(DDIR, LOW);
    digitalWrite(DDIRFW, HIGH);
  }
  else if (sag)
  {
    digitalWrite(ADIR, LOW);
    digitalWrite(ADIRFW, HIGH);
    digitalWrite(BDIR, LOW);
    digitalWrite(BDIRFW, HIGH);
    digitalWrite(CDIR, HIGH);
    digitalWrite(CDIRFW, LOW);
    digitalWrite(DDIR, HIGH);
    digitalWrite(DDIRFW, LOW);
  }
  else if (sol)
  {
    digitalWrite(ADIR, HIGH);
    digitalWrite(ADIRFW, LOW);
    digitalWrite(BDIR, HIGH);
    digitalWrite(BDIRFW, LOW);
    digitalWrite(CDIR, LOW);
    digitalWrite(CDIRFW, HIGH);
    digitalWrite(DDIR, LOW);
    digitalWrite(DDIRFW, HIGH);
  }
  else if (soldon)
  {
    digitalWrite(ADIR, HIGH);
    digitalWrite(ADIRFW, LOW);
    digitalWrite(BDIR, LOW);
    digitalWrite(BDIRFW, HIGH);
    digitalWrite(CDIR, HIGH);
    digitalWrite(CDIRFW, LOW);
    digitalWrite(DDIR, LOW);
    digitalWrite(DDIRFW, HIGH);
  }
  else if (sagdon)
  {
    digitalWrite(ADIR, LOW);
    digitalWrite(ADIRFW, HIGH);
    digitalWrite(BDIR, HIGH);
    digitalWrite(BDIRFW, LOW);
    digitalWrite(CDIR, LOW);
    digitalWrite(CDIRFW, HIGH);
    digitalWrite(DDIR, HIGH);
    digitalWrite(DDIRFW, LOW);
  }
  else
  {
  }
}

void lcdMessages()
{
  // en son yapılacak
  lcd.setCursor(0, 0);
  lcd.print("      OMNIMAN       ");
  lcd.setCursor(0, 1);
  lcd.print("RGB MOD : ON");
  lcd.setCursor(0, 2);
  lcd.print("AKU:");
  lcd.setCursor(6, 2);
  lcd.print("vBattRead()");
  lcd.setCursor(11, 2);
  lcd.print("%");
  lcd.setCursor(13, 2);
  lcd.setCursor(15, 2);
  lcd.print(vBattRead());
  lcd.setCursor(0, 3);
  lcd.print(amparageRead());
  lcd.setCursor(6, 3);
  lcd.print("HIZ: ");
  lcd.setCursor(9, 3);
  lcd.print("metre/dk");
}

float vBattRead()
{
  int adc_value = 0;
  for (int i = 0; i < BATTERY_AVARAGE_READ_COUNT; i++)
  {
    adc_value += analogRead(VBATTADC);
  }
  adc_value = adc_value / BATTERY_AVARAGE_READ_COUNT;
  float Vbatt = (adc_value * 0.11809);
  return Vbatt;
}

float amparageRead()
{
  int adc_value = 0;
  for (int i = 0; i < BATTERY_AVARAGE_READ_COUNT; i++)
  {
    adc_value += analogRead(AMPERAGE_PIN);
  }
  adc_value = adc_value / BATTERY_AVARAGE_READ_COUNT;
  float Vbatt = (adc_value * 0.01782);
  return Vbatt;
}

void chargerRgbStatus()
{
  newTime6 = millis();
  static int i = 0;
  static int artirFlag = 0;
  int amp = 30;
  if (amparageRead() >= 20)
  {
    amp = 4;
  }
  else if (amparageRead() >= 15 && amparageRead() < 20)
  {
    amp = 10;
  }
  else if (amparageRead() >= 10 && amparageRead() < 15)
  {
    amp = 25;
  }
  else if (amparageRead() >= 0 && amparageRead() < 10)
  {
    amp = 50;
  }

  if (newTime6 - oldTime6 > amp)
  {

    if (i == 0)
    {
      artirFlag = 1;
    }

    if (i == 20)
    {
      artirFlag = 0;
    }

    if (artirFlag == 1)
    {
      i++;
    }
    else
    {
      i--;
    }
    analogWrite(RED_PIN, i);
    analogWrite(GREEN_PIN, i);
    analogWrite(BLUE_PIN, i);
    oldTime6 = newTime6;
  }
}

void rgbController(bool white, bool red, bool green, bool blue, bool purple, bool cyan, bool yellow, bool stop)
{
  newTime1 = millis();
  static int i = 0;
  static int artirFlag = 0;
  if (newTime1 - oldTime1 > 10)
  {

    if (i == 0)
    {
      artirFlag = 1;
    }

    if (i == 70)
    {
      artirFlag = 0;
    }

    if (artirFlag == 1)
    {
      i++;
    }
    else
    {
      i--;
    }
    if (white)
    {
      analogWrite(RED_PIN, i);
      analogWrite(GREEN_PIN, i);
      analogWrite(BLUE_PIN, i);
    }
    else if (red)
    {
      analogWrite(RED_PIN, i);
      analogWrite(GREEN_PIN, 0);
      analogWrite(BLUE_PIN, 0);
    }
    else if (green)
    {
      analogWrite(RED_PIN, 0);
      analogWrite(GREEN_PIN, i);
      analogWrite(BLUE_PIN, 0);
    }
    else if (blue)
    {
      analogWrite(RED_PIN, 0);
      analogWrite(GREEN_PIN, 0);
      analogWrite(BLUE_PIN, i);
    }
    else if (purple)
    {
      analogWrite(RED_PIN, i);
      analogWrite(GREEN_PIN, 0);
      analogWrite(BLUE_PIN, i);
    }
    else if (cyan)
    {
      analogWrite(RED_PIN, 0);
      analogWrite(GREEN_PIN, i);
      analogWrite(BLUE_PIN, i);
    }
    else if (yellow)
    {
      analogWrite(RED_PIN, i);
      analogWrite(GREEN_PIN, i);
      analogWrite(BLUE_PIN, 0);
    }
    else
    {
      analogWrite(RED_PIN, 0);
      analogWrite(GREEN_PIN, 0);
      analogWrite(BLUE_PIN, 0);
    }
    oldTime1 = newTime1;
  }
}

void buzzerFlipFlop(int buzzerStatus) // 0 = off 2 = on 3 = flipflop
{
  newTime3 = millis();
  static int i = 0;
  static int artirFlag = 0;

  if (newTime3 - oldTime3 > 20)
  {
    if (buzzerStatus == 1)
    {
      analogWrite(BUZZER_PIN, 0);
    }
    else if (buzzerStatus == 2)
    {
      analogWrite(BUZZER_PIN, 10);
    }
    else if (buzzerStatus == 3)
    {
      if (i == 0)
      {
        artirFlag = 1;
      }

      if (i == 15)
      {
        artirFlag = 0;
      }

      if (artirFlag == 1)
      {
        i++;
      }
      else
      {
        i--;
      }

      analogWrite(BUZZER_PIN, i);
    }

    oldTime3 = newTime3;
  }
}

// ------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------
//          LINE SENSOR FRONT VALUE -- 4 FUNCTIONS
//-------------------------------------------------------------------------------------------
uint16_t lineFrontSensorValue()
{

  struct
  {
    union
    {
      struct
      {
        uint16_t sensorAll : 7;
      };
      struct
      {
        uint16_t sensor1 : 1;
        uint16_t sensor2 : 1;
        uint16_t sensor3 : 1;
        uint16_t sensor4 : 1;
        uint16_t sensor5 : 1;
        uint16_t sensor6 : 1;
        uint16_t sensor7 : 1;
      };
    };
  } lineSensor;

  lineSensor.sensorAll = 0;

  static int sayac1_on = 0;
  static int sayac1_off = 0;

  if (digitalRead(SENSOR_PIN_FRONT_1) == 0)
  {
    sayac1_off = 0;
    if (sayac1_on < SENSOR_CONFIDENCE)
    {
      sayac1_on++;
    }
  }
  else
  {
    sayac1_on = 0;
    if (sayac1_off < SENSOR_CONFIDENCE)
    {
      sayac1_off++;
    }
  }

  if (sayac1_on >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor1 = 0;
  }

  if (sayac1_off >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor1 = 1;
  }

  static int sayac2_on = 0;
  static int sayac2_off = 0;

  if (digitalRead(SENSOR_PIN_FRONT_2) == 0)
  {
    sayac2_off = 0;
    if (sayac2_on < SENSOR_CONFIDENCE)
    {
      sayac2_on++;
    }
  }
  else
  {
    sayac2_on = 0;
    if (sayac2_off < SENSOR_CONFIDENCE)
    {
      sayac2_off++;
    }
  }

  if (sayac2_on >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor2 = 0;
  }

  if (sayac2_off >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor2 = 1;
  }

  static int sayac3_on = 0;
  static int sayac3_off = 0;

  if (digitalRead(SENSOR_PIN_FRONT_3) == 0)
  {
    sayac3_off = 0;
    if (sayac3_on < SENSOR_CONFIDENCE)
    {
      sayac3_on++;
    }
  }
  else
  {
    sayac3_on = 0;
    if (sayac3_off < SENSOR_CONFIDENCE)
    {
      sayac3_off++;
    }
  }

  if (sayac3_on >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor3 = 0;
  }

  if (sayac3_off >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor3 = 1;
  }

  static int sayac4_on = 0;
  static int sayac4_off = 0;

  if (digitalRead(SENSOR_PIN_FRONT_4) == 0)
  {
    sayac4_off = 0;
    if (sayac4_on < SENSOR_CONFIDENCE)
    {
      sayac4_on++;
    }
  }
  else
  {
    sayac4_on = 0;
    if (sayac4_off < SENSOR_CONFIDENCE)
    {
      sayac4_off++;
    }
  }

  if (sayac4_on >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor4 = 0;
  }

  if (sayac4_off >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor4 = 1;
  }

  static int sayac5_on = 0;
  static int sayac5_off = 0;

  if (digitalRead(SENSOR_PIN_FRONT_5) == 0)
  {
    sayac5_off = 0;
    if (sayac5_on < SENSOR_CONFIDENCE)
    {
      sayac5_on++;
    }
  }
  else
  {
    sayac5_on = 0;
    if (sayac5_off < SENSOR_CONFIDENCE)
    {
      sayac5_off++;
    }
  }

  if (sayac5_on >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor5 = 0;
  }

  if (sayac5_off >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor5 = 1;
  }

  static int sayac6_on = 0;
  static int sayac6_off = 0;

  if (digitalRead(SENSOR_PIN_FRONT_6) == 0)
  {
    sayac6_off = 0;
    if (sayac6_on < SENSOR_CONFIDENCE)
    {
      sayac6_on++;
    }
  }
  else
  {
    sayac6_on = 0;
    if (sayac6_off < SENSOR_CONFIDENCE)
    {
      sayac6_off++;
    }
  }

  if (sayac6_on >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor6 = 0;
  }

  if (sayac6_off >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor6 = 1;
  }

  static int sayac7_on = 0;
  static int sayac7_off = 0;

  if (digitalRead(SENSOR_PIN_FRONT_7) == 0)
  {
    sayac7_off = 0;
    if (sayac7_on < SENSOR_CONFIDENCE)
    {
      sayac7_on++;
    }
  }
  else
  {
    sayac7_on = 0;
    if (sayac7_off < SENSOR_CONFIDENCE)
    {
      sayac7_off++;
    }
  }

  if (sayac7_on >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor7 = 0;
  }

  if (sayac7_off >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor7 = 1;
  }

  /*
    lineSensor.sensor1 = digitalRead(SENSOR_PIN1);
    lineSensor.sensor2 = digitalRead(SENSOR_PIN2);
    lineSensor.sensor3 = digitalRead(SENSOR_PIN3);
    lineSensor.sensor4 = digitalRead(SENSOR_PIN4);
    lineSensor.sensor5 = digitalRead(SENSOR_PIN5);
    lineSensor.sensor6 = digitalRead(SENSOR_PIN6);
    lineSensor.sensor7 = digitalRead(SENSOR_PIN7);
  */

  return lineSensor.sensorAll;
}

int lineFrontSensorValue_ON_SOL_VALUE()
{

  if ((lineFrontSensorValue() & 0b0000011) > 0) // 1. GRUP -- SONDAN VEYA BAŞTAN ILK 2 SENSOR OLCUMU -- 1*0 = 1 / 0*0 = 0 / 1*1 = 1
  {
    // SENSORLERDEN GELEN DEGERLERDEN SADECE 1 I BILE TRUE ISE TRUE DONER
    ON_SOL_SENSORLER = 1;
  }
  else if ((lineFrontSensorValue() & 0b0000011) == 0)
  {
    // Serial.println(" 0 GONDERIYOR ON_SOL_SENSORLER");
    ON_SOL_SENSORLER = 0;
  }

  return ON_SOL_SENSORLER;
}

int lineFrontSensorValue_ON_ORTA_VALUE()
{

  if ((lineFrontSensorValue() & 0b0011100) > 0) // 1. GRUP -- SONDAN VEYA BAŞTAN ILK 2 SENSOR OLCUMU -- 1*0 = 1 / 0*0 = 0 / 1*1 = 1
  {
    // SENSORLERDEN GELEN DEGERLERDEN SADECE 1 I BILE TRUE ISE TRUE DONER
    ON_ORTA_SENSORLER = 1;
  }
  else if ((lineFrontSensorValue() & 0b0011100) == 0)
  {
    // Serial.println(" 0 GONDERIYOR ON_ORTA_SENSORLER");
    ON_ORTA_SENSORLER = 0;
  }

  return ON_ORTA_SENSORLER;
}

int lineFrontSensorValue_ON_SAG_SENSOR()
{
  if ((lineFrontSensorValue() & 0b1100000) > 0)
  {
    // SENSORLERDEN GELEN DEGERLERDEN SADECE 1 I BILE TRUE ISE TRUE DONER
    ON_SAG_SENSORLER = 1;
  }
  else if ((lineFrontSensorValue() & 0b1100000) == 0)
  {
    // Serial.println(" 0 GONDERIYOR SENSORLERDEN");
    ON_SAG_SENSORLER = 0;
  }
  return ON_SAG_SENSORLER;
}

// ------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------
//          LINE SENSOR BACK VALUE -- 4 FUNCTIONS
//-------------------------------------------------------------------------------------------
uint16_t lineBackSensorValue()
{

  struct
  {
    union
    {
      struct
      {
        uint16_t sensorAll : 7;
      };
      struct
      {
        uint16_t sensor1 : 1;
        uint16_t sensor2 : 1;
        uint16_t sensor3 : 1;
        uint16_t sensor4 : 1;
        uint16_t sensor5 : 1;
        uint16_t sensor6 : 1;
        uint16_t sensor7 : 1;
      };
    };
  } lineSensor;

  lineSensor.sensorAll = 0;

  static int sayac1_on = 0;
  static int sayac1_off = 0;

  if (digitalRead(SENSOR_PIN_BACK_1) == 0)
  {
    sayac1_off = 0;
    if (sayac1_on < SENSOR_CONFIDENCE)
    {
      sayac1_on++;
    }
  }
  else
  {
    sayac1_on = 0;
    if (sayac1_off < SENSOR_CONFIDENCE)
    {
      sayac1_off++;
    }
  }

  if (sayac1_on >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor1 = 0;
  }

  if (sayac1_off >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor1 = 1;
  }

  static int sayac2_on = 0;
  static int sayac2_off = 0;

  if (digitalRead(SENSOR_PIN_BACK_2) == 0)
  {
    sayac2_off = 0;
    if (sayac2_on < SENSOR_CONFIDENCE)
    {
      sayac2_on++;
    }
  }
  else
  {
    sayac2_on = 0;
    if (sayac2_off < SENSOR_CONFIDENCE)
    {
      sayac2_off++;
    }
  }

  if (sayac2_on >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor2 = 0;
  }

  if (sayac2_off >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor2 = 1;
  }

  static int sayac3_on = 0;
  static int sayac3_off = 0;

  if (digitalRead(SENSOR_PIN_BACK_3) == 0)
  {
    sayac3_off = 0;
    if (sayac3_on < SENSOR_CONFIDENCE)
    {
      sayac3_on++;
    }
  }
  else
  {
    sayac3_on = 0;
    if (sayac3_off < SENSOR_CONFIDENCE)
    {
      sayac3_off++;
    }
  }

  if (sayac3_on >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor3 = 0;
  }

  if (sayac3_off >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor3 = 1;
  }

  static int sayac4_on = 0;
  static int sayac4_off = 0;

  if (digitalRead(SENSOR_PIN_BACK_4) == 0)
  {
    sayac4_off = 0;
    if (sayac4_on < SENSOR_CONFIDENCE)
    {
      sayac4_on++;
    }
  }
  else
  {
    sayac4_on = 0;
    if (sayac4_off < SENSOR_CONFIDENCE)
    {
      sayac4_off++;
    }
  }

  if (sayac4_on >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor4 = 0;
  }

  if (sayac4_off >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor4 = 1;
  }

  static int sayac5_on = 0;
  static int sayac5_off = 0;

  if (digitalRead(SENSOR_PIN_BACK_5) == 0)
  {
    sayac5_off = 0;
    if (sayac5_on < SENSOR_CONFIDENCE)
    {
      sayac5_on++;
    }
  }
  else
  {
    sayac5_on = 0;
    if (sayac5_off < SENSOR_CONFIDENCE)
    {
      sayac5_off++;
    }
  }

  if (sayac5_on >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor5 = 0;
  }

  if (sayac5_off >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor5 = 1;
  }

  static int sayac6_on = 0;
  static int sayac6_off = 0;

  if (digitalRead(SENSOR_PIN_BACK_6) == 0)
  {
    sayac6_off = 0;
    if (sayac6_on < SENSOR_CONFIDENCE)
    {
      sayac6_on++;
    }
  }
  else
  {
    sayac6_on = 0;
    if (sayac6_off < SENSOR_CONFIDENCE)
    {
      sayac6_off++;
    }
  }

  if (sayac6_on >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor6 = 0;
  }

  if (sayac6_off >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor6 = 1;
  }

  static int sayac7_on = 0;
  static int sayac7_off = 0;

  if (digitalRead(SENSOR_PIN_BACK_7) == 0)
  {
    sayac7_off = 0;
    if (sayac7_on < SENSOR_CONFIDENCE)
    {
      sayac7_on++;
    }
  }
  else
  {
    sayac7_on = 0;
    if (sayac7_off < SENSOR_CONFIDENCE)
    {
      sayac7_off++;
    }
  }

  if (sayac7_on >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor7 = 0;
  }

  if (sayac7_off >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor7 = 1;
  }

  /*
    lineSensor.sensor1 = digitalRead(SENSOR_PIN1);
    lineSensor.sensor2 = digitalRead(SENSOR_PIN2);
    lineSensor.sensor3 = digitalRead(SENSOR_PIN3);
    lineSensor.sensor4 = digitalRead(SENSOR_PIN4);
    lineSensor.sensor5 = digitalRead(SENSOR_PIN5);
    lineSensor.sensor6 = digitalRead(SENSOR_PIN6);
    lineSensor.sensor7 = digitalRead(SENSOR_PIN7);
  */

  return lineSensor.sensorAll;
}

int lineBackSensorValue_ARKA_SOL_VALUE()
{

  if ((lineBackSensorValue() & 0b0000011) > 0) // 1. GRUP -- SONDAN VEYA BAŞTAN ILK 2 SENSOR OLCUMU -- 1*0 = 1 / 0*0 = 0 / 1*1 = 1
  {
    // SENSORLERDEN GELEN DEGERLERDEN SADECE 1 I BILE TRUE ISE TRUE DONER
    ARKA_SOL_SENSORLER = 1;
  }
  else if ((lineBackSensorValue() & 0b0000011) == 0)
  {
    // Serial.println(" 0 GONDERIYOR ARKA_SOL_SENSORLER");
    ARKA_SOL_SENSORLER = 0;
  }

  return ARKA_SOL_SENSORLER;
}

int lineBackSensorValue_ARKA_ORTA_VALUE()
{

  if ((lineBackSensorValue() & 0b0011100) > 0) // 1. GRUP -- SONDAN VEYA BAŞTAN ILK 2 SENSOR OLCUMU -- 1*0 = 1 / 0*0 = 0 / 1*1 = 1
  {
    // SENSORLERDEN GELEN DEGERLERDEN SADECE 1 I BILE TRUE ISE TRUE DONER
    ARKA_ORTA_SENSORLER = 1;
  }
  else if ((lineBackSensorValue() & 0b0011100) == 0)
  {
    // Serial.println(" 0 GONDERIYOR ARKA_ORTA_SENSORLER");
    ARKA_ORTA_SENSORLER = 0;
  }

  return ARKA_ORTA_SENSORLER;
}

int lineBackSensorValue_ARKA_SAG_SENSOR()
{
  if ((lineBackSensorValue() & 0b1100000) > 0)
  {
    // SENSORLERDEN GELEN DEGERLERDEN SADECE 1 I BILE TRUE ISE TRUE DONER
    ARKA_SAG_SENSORLER = 1;
  }
  else if ((lineBackSensorValue() & 0b1100000) == 0)
  {
    // Serial.println(" 0 GONDERIYOR ARKA_SAG_SENSORLER");
    ARKA_SAG_SENSORLER = 0;
  }
  return ARKA_SAG_SENSORLER;
}

// ------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------
//          LINE SENSOR RIGT VALUE -- 4 FUNCTIONS
//-------------------------------------------------------------------------------------------
uint16_t lineRightSensorValue()
{
  struct
  {
    union
    {
      struct
      {
        uint16_t sensorAll : 7;
      };
      struct
      {
        uint16_t sensor1 : 1;
        uint16_t sensor2 : 1;
        uint16_t sensor3 : 1;
        uint16_t sensor4 : 1;
        uint16_t sensor5 : 1;
        uint16_t sensor6 : 1;
        uint16_t sensor7 : 1;
      };
    };
  } lineSensor;

  lineSensor.sensorAll = 0;

  static int sayac1_on = 0;
  static int sayac1_off = 0;

  if (digitalRead(SENSOR_PIN_RIGHT_1) == 0)
  {
    sayac1_off = 0;
    if (sayac1_on < SENSOR_CONFIDENCE)
    {
      sayac1_on++;
    }
  }
  else
  {
    sayac1_on = 0;
    if (sayac1_off < SENSOR_CONFIDENCE)
    {
      sayac1_off++;
    }
  }

  if (sayac1_on >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor1 = 0;
  }

  if (sayac1_off >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor1 = 1;
  }

  static int sayac2_on = 0;
  static int sayac2_off = 0;

  if (digitalRead(SENSOR_PIN_RIGHT_2) == 0)
  {
    sayac2_off = 0;
    if (sayac2_on < SENSOR_CONFIDENCE)
    {
      sayac2_on++;
    }
  }
  else
  {
    sayac2_on = 0;
    if (sayac2_off < SENSOR_CONFIDENCE)
    {
      sayac2_off++;
    }
  }

  if (sayac2_on >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor2 = 0;
  }

  if (sayac2_off >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor2 = 1;
  }

  static int sayac3_on = 0;
  static int sayac3_off = 0;

  if (digitalRead(SENSOR_PIN_RIGHT_3) == 0)
  {
    sayac3_off = 0;
    if (sayac3_on < SENSOR_CONFIDENCE)
    {
      sayac3_on++;
    }
  }
  else
  {
    sayac3_on = 0;
    if (sayac3_off < SENSOR_CONFIDENCE)
    {
      sayac3_off++;
    }
  }

  if (sayac3_on >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor3 = 0;
  }

  if (sayac3_off >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor3 = 1;
  }

  static int sayac4_on = 0;
  static int sayac4_off = 0;

  if (digitalRead(SENSOR_PIN_RIGHT_4) == 0)
  {
    sayac4_off = 0;
    if (sayac4_on < SENSOR_CONFIDENCE)
    {
      sayac4_on++;
    }
  }
  else
  {
    sayac4_on = 0;
    if (sayac4_off < SENSOR_CONFIDENCE)
    {
      sayac4_off++;
    }
  }

  if (sayac4_on >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor4 = 0;
  }

  if (sayac4_off >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor4 = 1;
  }

  static int sayac5_on = 0;
  static int sayac5_off = 0;

  if (digitalRead(SENSOR_PIN_RIGHT_5) == 0)
  {
    sayac5_off = 0;
    if (sayac5_on < SENSOR_CONFIDENCE)
    {
      sayac5_on++;
    }
  }
  else
  {
    sayac5_on = 0;
    if (sayac5_off < SENSOR_CONFIDENCE)
    {
      sayac5_off++;
    }
  }

  if (sayac5_on >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor5 = 0;
  }

  if (sayac5_off >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor5 = 1;
  }

  static int sayac6_on = 0;
  static int sayac6_off = 0;

  if (digitalRead(SENSOR_PIN_RIGHT_6) == 0)
  {
    sayac6_off = 0;
    if (sayac6_on < SENSOR_CONFIDENCE)
    {
      sayac6_on++;
    }
  }
  else
  {
    sayac6_on = 0;
    if (sayac6_off < SENSOR_CONFIDENCE)
    {
      sayac6_off++;
    }
  }

  if (sayac6_on >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor6 = 0;
  }

  if (sayac6_off >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor6 = 1;
  }

  static int sayac7_on = 0;
  static int sayac7_off = 0;

  if (digitalRead(SENSOR_PIN_RIGHT_7) == 0)
  {
    sayac7_off = 0;
    if (sayac7_on < SENSOR_CONFIDENCE)
    {
      sayac7_on++;
    }
  }
  else
  {
    sayac7_on = 0;
    if (sayac7_off < SENSOR_CONFIDENCE)
    {
      sayac7_off++;
    }
  }

  if (sayac7_on >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor7 = 0;
  }

  if (sayac7_off >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor7 = 1;
  }

  /*
    lineSensor.sensor1
    lineSensor.sensor2
    lineSensor.sensor3
    lineSensor.sensor4
    lineSensor.sensor5
    lineSensor.sensor6
    lineSensor.sensor7
  */

  return lineSensor.sensorAll;
}

int lineRightSensor_SAG_ON_VALUE()
{

  if ((lineRightSensorValue() & 0b0000011) > 0) // 1. GRUP -- SONDAN VEYA BAŞTAN ILK 2 SENSOR OLCUMU -- 1*0 = 1 / 0*0 = 0 / 1*1 = 1
  {
    // SENSORLERDEN GELEN DEGERLERDEN SADECE 1 I BILE TRUE ISE TRUE DONER
    SAG_ON_SENSORLER = 1;
  }
  else if ((lineRightSensorValue() & 0b0000011) == 0)
  {
    // Serial.println(" 0 GONDERIYOR SAG_ON_SENSORLER");
    SAG_ON_SENSORLER = 0;
  }

  return SAG_ON_SENSORLER;
}

int lineRightSensor_SAG_ORTA_VALUE()
{
  if ((lineRightSensorValue() & 0b0011100) > 0)
  {
    // SENSORLERDEN GELEN DEGERLERDEN SADECE 1 I BILE TRUE ISE TRUE DONER
    SAG_ORTA_SENSORLER = 1;
  }
  else if ((lineRightSensorValue() & 0b0011100) == 0)
  {
    // Serial.println(" 0 GONDERIYOR SAG_ORTA_SENSORLER");
    SAG_ORTA_SENSORLER = 0;
  }
  return SAG_ORTA_SENSORLER;
}

int lineRightSensor_SAG_ARKA_SENSOR()
{
  if ((lineRightSensorValue() & 0b1100000) > 0)
  {
    // SENSORLERDEN GELEN DEGERLERDEN SADECE 1 I BILE TRUE ISE TRUE DONER
    SAG_ARKA_SENSORLER = 1;
  }
  else if ((lineRightSensorValue() & 0b1100000) == 0)
  {
    // Serial.println(" 0 GONDERIYOR SAG_ARKA_SENSORLER");
    SAG_ARKA_SENSORLER = 0;
  }
  return SAG_ARKA_SENSORLER;
}

// ------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------
//          LINE SENSOR LEFT VALUE -- 4 FUNCTIONS
//-------------------------------------------------------------------------------------------
uint16_t lineLeftSensorValue()
{

  struct
  {
    union
    {
      struct
      {
        uint16_t sensorAll : 7;
      };
      struct
      {
        uint16_t sensor1 : 1;
        uint16_t sensor2 : 1;
        uint16_t sensor3 : 1;
        uint16_t sensor4 : 1;
        uint16_t sensor5 : 1;
        uint16_t sensor6 : 1;
        uint16_t sensor7 : 1;
      };
    };
  } lineSensor;

  lineSensor.sensorAll = 0;

  static int sayac1_on = 0;
  static int sayac1_off = 0;

  if (digitalRead(SENSOR_PIN_LEFT_1) == 0)
  {
    sayac1_off = 0;
    if (sayac1_on < SENSOR_CONFIDENCE)
    {
      sayac1_on++;
    }
  }
  else
  {
    sayac1_on = 0;
    if (sayac1_off < SENSOR_CONFIDENCE)
    {
      sayac1_off++;
    }
  }

  if (sayac1_on >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor1 = 0;
  }

  if (sayac1_off >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor1 = 1;
  }

  static int sayac2_on = 0;
  static int sayac2_off = 0;

  if (digitalRead(SENSOR_PIN_LEFT_2) == 0)
  {
    sayac2_off = 0;
    if (sayac2_on < SENSOR_CONFIDENCE)
    {
      sayac2_on++;
    }
  }
  else
  {
    sayac2_on = 0;
    if (sayac2_off < SENSOR_CONFIDENCE)
    {
      sayac2_off++;
    }
  }

  if (sayac2_on >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor2 = 0;
  }

  if (sayac2_off >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor2 = 1;
  }

  static int sayac3_on = 0;
  static int sayac3_off = 0;

  if (digitalRead(SENSOR_PIN_LEFT_3) == 0)
  {
    sayac3_off = 0;
    if (sayac3_on < SENSOR_CONFIDENCE)
    {
      sayac3_on++;
    }
  }
  else
  {
    sayac3_on = 0;
    if (sayac3_off < SENSOR_CONFIDENCE)
    {
      sayac3_off++;
    }
  }

  if (sayac3_on >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor3 = 0;
  }

  if (sayac3_off >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor3 = 1;
  }

  static int sayac4_on = 0;
  static int sayac4_off = 0;

  if (digitalRead(SENSOR_PIN_LEFT_4) == 0)
  {
    sayac4_off = 0;
    if (sayac4_on < SENSOR_CONFIDENCE)
    {
      sayac4_on++;
    }
  }
  else
  {
    sayac4_on = 0;
    if (sayac4_off < SENSOR_CONFIDENCE)
    {
      sayac4_off++;
    }
  }

  if (sayac4_on >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor4 = 0;
  }

  if (sayac4_off >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor4 = 1;
  }

  static int sayac5_on = 0;
  static int sayac5_off = 0;

  if (digitalRead(SENSOR_PIN_LEFT_5) == 0)
  {
    sayac5_off = 0;
    if (sayac5_on < SENSOR_CONFIDENCE)
    {
      sayac5_on++;
    }
  }
  else
  {
    sayac5_on = 0;
    if (sayac5_off < SENSOR_CONFIDENCE)
    {
      sayac5_off++;
    }
  }

  if (sayac5_on >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor5 = 0;
  }

  if (sayac5_off >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor5 = 1;
  }

  static int sayac6_on = 0;
  static int sayac6_off = 0;

  if (digitalRead(SENSOR_PIN_LEFT_6) == 0)
  {
    sayac6_off = 0;
    if (sayac6_on < SENSOR_CONFIDENCE)
    {
      sayac6_on++;
    }
  }
  else
  {
    sayac6_on = 0;
    if (sayac6_off < SENSOR_CONFIDENCE)
    {
      sayac6_off++;
    }
  }

  if (sayac6_on >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor6 = 0;
  }

  if (sayac6_off >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor6 = 1;
  }

  static int sayac7_on = 0;
  static int sayac7_off = 0;

  if (digitalRead(SENSOR_PIN_LEFT_7) == 0)
  {
    sayac7_off = 0;
    if (sayac7_on < SENSOR_CONFIDENCE)
    {
      sayac7_on++;
    }
  }
  else
  {
    sayac7_on = 0;
    if (sayac7_off < SENSOR_CONFIDENCE)
    {
      sayac7_off++;
    }
  }

  if (sayac7_on >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor7 = 0;
  }

  if (sayac7_off >= SENSOR_CONFIDENCE)
  {
    lineSensor.sensor7 = 1;
  }

  /*
    951 BOS
    666 ILERI
    320 GERI
    520 SAG
    24 SOL
    790 DUZ
    834 TERS
  */

  return lineSensor.sensorAll;
}

int lineLeftSensor_SOL_ARKA_SENSOR()
{
  if ((lineLeftSensorValue() & 0b1100000) > 0)
  {
    // SENSORLERDEN GELEN DEGERLERDEN SADECE 1 I BILE TRUE ISE TRUE DONER
    SOL_ARKA_SENSORLER = 1;
  }
  else if ((lineLeftSensorValue() & 0b1100000) == 0)
  {
    // Serial.println(" 0 GONDERIYOR SOL_ARKA_SENSORLER");
    SOL_ARKA_SENSORLER = 0;
  }
  return SOL_ARKA_SENSORLER;
}

int lineLeftSensor_SOL_ON_VALUE()
{
  if ((lineLeftSensorValue() & 0b0000011) > 0) // 1. GRUP -- SONDAN VEYA BAŞTAN ILK 2 SENSOR OLCUMU -- 1*0 = 1 / 0*0 = 0 / 1*1 = 1
  {
    // SENSORLERDEN GELEN DEGERLERDEN SADECE 1 I BILE TRUE ISE TRUE DONER
    SOL_ON_SENSORLER = 1;
  }
  else if ((lineLeftSensorValue() & 0b0000011) == 0)
  {
    // Serial.println(" 0 GONDERIYOR SOL_ON_SENSORLER");
    SOL_ON_SENSORLER = 0;
  }

  return SOL_ON_SENSORLER;
}

int lineLeftSensor_SOL_ORTA_VALUE()
{
  if ((lineLeftSensorValue() & 0b0011100) > 0)
  {
    // SENSORLERDEN GELEN DEGERLERDEN SADECE 1 I BILE TRUE ISE TRUE DONER
    SOL_ORTA_SENSORLER = 1;
  }
  else if ((lineLeftSensorValue() & 0b0011100) == 0)
  {
    // Serial.println(" 0 GONDERIYOR SAG_ORTA_SENSORLER");
    SOL_ORTA_SENSORLER = 0;
  }
  return SOL_ORTA_SENSORLER;
}
