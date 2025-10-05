#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/Picopixel.h>

//--------------------CONSTANTS----------------------------------
//-----------------Screen configuration (SSD1306)----------------
#define     SCREEN_WIDTH              (128)
#define     SCREEN_HEIGHT             (64)
#define     OLED_RESET                (-1)
#define     OLED_ADDRESS              (0x3C)
//-----------------Custom I2C Pins (NodeMCU)---------------------
#define     I2C_SDA                   (2)   // D4 (GPIO2)
#define     I2C_SCL                   (0)   // D3 (GPIO0)
//-----------------MQ2 Constants (GAS LP)------------------------
#define     MQ2_PIN                   (A0)
#define     RL_VAL_MQ2                (5)
#define     RO_CLEAN_AIR_FACT_MQ2     (9.83)
#define     GAS_LP                    (0)
//-----------------MQ9 Constants (CO2)---------------------------
#define     MQ9_PIN                   (A0)
#define     RL_VAL_MQ9                (1)
#define     RO_CLEAN_AIR_FACT_MQ9     (9.799)
#define     GAS_CO                    (1)   
//-----------------Calibration constants-------------------------
#define     CALIB_SAMPLE_TIMES        (50) //How many samples to take in calibration phase
#define     CALIB_SAMPLE_INTERVAL     (500) //time interval between each sample (ms)
#define     READ_SAMPLE_TIMES         (30) //How many samples to take in normal operation
#define     READ_SAMPLE_INTERVAL      (50) //Time interval between each normal sample (ms)
//-----------------NodeMCU Pin constants--------------------------
#define     D0                        (16)
#define     D1                        (5)
#define     D2                        (4)


//---------------------GLOBAL VARIABLES----------------------------
float LPCurve[3] = {2.3, 0.21, -0.47};
float COCurve[3] = {2.3, 0.72, -0.34};
float Ro_mq2 = 10;
float Ro_mq9 = 20;
float GAS_S = 0; //Storing variable for MQ2
float CO = 0; //Storing variable for MQ9
bool available = false; //Verify if sensors are calibrated ok

// Display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Starting frame image
const unsigned char kirbyImage [] PROGMEM = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xfc, 0xff, 
	0xff, 0xff, 0xff, 0x79, 0xff, 0x7f, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xbe, 0xff, 0x7f, 0xff, 
	0xfc, 0xff, 0xff, 0xff, 0xfe, 0xba, 0xfe, 0xff, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xfe, 0xb8, 0xff, 
	0x7f, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xfe, 0x38, 0xff, 0xff, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xfe, 
	0x38, 0xff, 0xff, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xfe, 0x38, 0xff, 0xff, 0xff, 0xfc, 0xff, 0xff, 
	0xff, 0xfe, 0x3d, 0xff, 0x3f, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xe7, 0xff, 0x3f, 0xff, 0xfc, 
	0xff, 0xff, 0xff, 0xff, 0x83, 0xff, 0x3f, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xff, 0x83, 0xff, 0x3f, 
	0xff, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x3f, 0xff, 0xfc, 0xff, 0xff, 0xf8, 0xff, 0xef, 
	0xfe, 0x3f, 0xff, 0xfc, 0xff, 0xff, 0xc0, 0x0f, 0xff, 0xfc, 0x3f, 0xff, 0xfc, 0xff, 0xff, 0x80, 
	0x03, 0xff, 0xfc, 0x7f, 0xff, 0xfc, 0xff, 0xff, 0x80, 0x01, 0xff, 0xf8, 0x7f, 0xff, 0xfc, 0xff, 
	0xff, 0x80, 0x00, 0xff, 0xf0, 0xff, 0xff, 0xfc, 0xff, 0xff, 0x80, 0x00, 0x7f, 0xe0, 0xff, 0xff, 
	0xfc, 0xff, 0xff, 0xc0, 0x00, 0x3f, 0x80, 0xff, 0xff, 0xfc, 0xff, 0xff, 0xe0, 0x00, 0x1f, 0x00, 
	0xff, 0xff, 0xfc, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0xff, 0xff, 0xfc, 0xff, 0xff, 0xf8, 0x00, 
	0x00, 0x00, 0x7f, 0xff, 0xfc, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xfc, 0xff, 0xff, 
	0xff, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xfc, 0xff, 0xff, 0xff, 0x80, 0x0f, 0x00, 0x7f, 0xff, 0xfc, 
	0xff, 0xff, 0xff, 0xe0, 0x1f, 0x00, 0x7f, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xf8, 0x7f, 0x80, 0x7f, 
	0xff, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x7f, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xc0, 0x7f, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0xff, 0xff, 0xfc, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xf9, 0xff, 0xff, 0xfc
};


/**
  * NodeMCU setup function
  */
void setup() {
  Serial.begin(9600);

  // Start I2C 
  Wire.begin(I2C_SDA, I2C_SCL);

  delay(6000);
  Serial.println("Starting...");

  // Start screen
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
  display.clearDisplay();
  startingFrames();

  // Configuration
  pinMode(D0, OUTPUT);
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);

  digitalWrite(D0, LOW);
  digitalWrite(D1, LOW);
  digitalWrite(D2, LOW);
  delay(200);
  Ro_mq2 = MQ2Calibration(MQ2_PIN);
  delay(500);

  digitalWrite(D0, HIGH);
  digitalWrite(D1, LOW);
  digitalWrite(D2, LOW);
  delay(200);
  Ro_mq9 = MQ2Calibration(MQ9_PIN);
  delay(500);

  Serial.println("Calibration finished");
  Serial.println("Ro MQ2:");
  Serial.println(Ro_mq2);
  Serial.println("Ro MQ9:");
  Serial.println(Ro_mq9);

  if((!isinf(Ro_mq2)) && (!isinf(Ro_mq9))) {
    //sensorFrame();
    available = true;
  } else {
    available = false;
  }

}

/**
  * Loop method.
  */
void loop() {
  if (available) {
    sensorFrame();
    Serial.println("Reading process");

    //reading MQ2
    digitalWrite(D0, LOW);
    digitalWrite(D1, LOW);
    digitalWrite(D2, LOW);
    delay(200);
    GAS_S = MQGetGasPercentage(readMQ2(MQ2_PIN)/Ro_mq2, GAS_LP);

    //reading MQ9
    digitalWrite(D0, HIGH);
    digitalWrite(D1, LOW);
    digitalWrite(D2, LOW);
    delay(200);
    CO = MQGetGasPercentage(readMQ9(MQ9_PIN)/Ro_mq9, GAS_CO);

    Serial.print("GAS_LP: "); Serial.println(GAS_S);
    Serial.print("CO: "); Serial.println(CO);
  } else {
    Serial.println("ERROR");
  }
  
}

/**
  * Calculating MQ sensors percentage
  */
int MQGetPercetage(float rs_ro_ratio, float *lpCurve) {
  return (pow(10, (((log(rs_ro_ratio) - lpCurve[1]) / lpCurve[2]) + lpCurve[0])));
}

/**
  * Calculating the MQ sensors gas percentage
  */
int MQGetGasPercentage(float rs_ro_ratio, int gas_id) {
  if(gas_id == GAS_LP) {
    return MQGetPercetage(rs_ro_ratio, LPCurve);
  } else if (gas_id == GAS_CO) {
    return MQGetPercetage(rs_ro_ratio, COCurve);
  } else {
    return 0;
  }
}

/**
  * Calculating MQ9 Gas percentage (Only monoxide)
  */
//int MG9GasPercentage(float rs_ro_ratio) {
//  return (pow(10, ((-2.199 * (log10(rs_ro_ratio))) + 2.766)));
//}

/**
  * Calculating the MQ2 Resistance
  */
float MQ2ResistanceCalc(int raw_adc) {
  return (((float)RL_VAL_MQ2 * (1023 - raw_adc) / raw_adc));
}

/**
  * Calculating the MQ9 Resistance
  */
float MQ9ResistanceCalc(int raw_adc) {
  return (((float)RL_VAL_MQ9 * (1023 - raw_adc) / raw_adc));
}

/**
  * Calibrating the MQ2 sensor
  */
float MQ2Calibration(int mq_pin) {
  int i;
  float val = 0;

  for (i = 0; i < CALIB_SAMPLE_TIMES; i++) {
    val += MQ2ResistanceCalc(analogRead(mq_pin));
    delay(CALIB_SAMPLE_INTERVAL);
  }

  val = val / CALIB_SAMPLE_TIMES;
  val = val / RO_CLEAN_AIR_FACT_MQ2;
  
  return val;
}

/**
  * Calibrating the MQ9 sensor
  */
float MQ9Calibration(int mq_pin) {
  int i;
  float rs_air_val = 0;
  float r0;

  for(i = 0; i < CALIB_SAMPLE_TIMES; i++) {
    rs_air_val += MQ9ResistanceCalc(analogRead(mq_pin));
    delay(CALIB_SAMPLE_INTERVAL);
  }

  rs_air_val = rs_air_val / CALIB_SAMPLE_TIMES;
  r0 = rs_air_val / RO_CLEAN_AIR_FACT_MQ9;

  return r0;
}

/**
  * Read the MQ2 input data
  */
float readMQ2(int mq_pin) {
  int i;
  float rs = 0;

  for (i = 0; i < READ_SAMPLE_TIMES; i++) {
    rs += MQ2ResistanceCalc(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }

  rs = rs / READ_SAMPLE_TIMES;
  return rs;
}

/**
  * Read the MQ9 input data
  */
float readMQ9(int mq_pin) {
  int i;
  float rs = 0;

  for(i = 0; i < READ_SAMPLE_TIMES; i++) {
    rs += MQ9ResistanceCalc(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }

  rs = rs / READ_SAMPLE_TIMES;
  return rs;
}

/**
  * It show the starting frames (when circuit is energized)
  */
void startingFrames() {
  //-------------------WELCOME SCREEN ----------------------------
  // Clear display and draw content
  display.clearDisplay();

  // Show text
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setFont(&Picopixel);
  display.setCursor(25, 8);
  display.println("GAS AND CO2");
  display.setCursor(30, 22);
  display.println("SENSOR IoT");

  //Show image
  display.drawBitmap(30, 36, kirbyImage, 70, 38, SSD1306_WHITE);

  //Show content in the screen
  display.display();

  //wait five seconds
  delay(6000);

  //-------------------BY SCREEN -------------------------------
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setFont(&Picopixel);
  display.setCursor(25, 30);
  display.println("By Andy Val");
  display.setCursor(5, 44);
  display.println("Github: Kazuhira97");
  display.display();
  delay(4000);

  //-------------------CALIBRATING SCREEN --------------------------
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setFont(&Picopixel);
  display.setCursor(30, 30);
  display.println("Calibrating");
  display.setCursor(35, 44);
  display.println("sensors...");
  display.display();
}

/**
  * It shows the sensor frame (Measurements)
  */
void sensorFrame() {
  int GAS_LP_dummy = 1234; // Dummy data
  int CO_2_dummy = 1234;
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setFont(&Picopixel);

  //Measurements headers
  display.setCursor(15, 25);
  display.println("GAS LP");
  display.setCursor(85, 25);
  display.println("CO2");

  //Unit headers
  display.setCursor(17, 39);
  display.println("(ppm)");
  display.setCursor(78, 39);
  display.println("(ppm)");

  //Measurements data
  display.setCursor(20, 60);
  display.println(String(GAS_LP_dummy));
  display.setCursor(81, 60);
  display.println(String(CO_2_dummy));

  display.display();
}