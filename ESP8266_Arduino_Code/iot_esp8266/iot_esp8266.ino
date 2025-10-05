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
#define     MQ2_CHANNEL               (0)
//-----------------MQ9 Constants (CO2)---------------------------
#define     MQ9_PIN                   (A0)
#define     RL_VAL_MQ9                (10)
#define     RO_CLEAN_AIR_FACT_MQ9     (9.799)
#define     GAS_CO                    (1) 
#define     MQ9_CHANNEL               (1)  
//-----------------Calibration constants-------------------------
#define     CALIB_SAMPLE_TIMES        (50) //How many samples to take in calibration phase
#define     CALIB_SAMPLE_INTERVAL     (500) //time interval between each sample (ms)
#define     READ_SAMPLE_TIMES         (5) //How many samples to take in normal operation
#define     READ_SAMPLE_INTERVAL      (20) //Time interval between each normal sample (ms)
//-----------------NodeMCU Pin constants--------------------------
#define     D0                        (16)
#define     D1                        (5)
#define     D2                        (4)


//---------------------GLOBAL VARIABLES----------------------------
float LPCurve[3] = {2.3, 0.21, -0.47};
float COCurve[3] = {2.3, 0.72, -0.34};
float Ro_mq2 = 10;
float Ro_mq9 = 20;
long GAS_S = 0; //Storing variable for MQ2
long CO = 0; //Storing variable for MQ9
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

  selectChannel(MQ2_CHANNEL);
  Ro_mq2 = MQCalibration(MQ2_PIN, RO_CLEAN_AIR_FACT_MQ2, RL_VAL_MQ2);
  delay(500);

  selectChannel(MQ9_CHANNEL);
  Ro_mq9 = MQCalibration(MQ9_PIN, RO_CLEAN_AIR_FACT_MQ9, RL_VAL_MQ9);
  delay(500);


  Serial.println("Calibration finished");
  Serial.println("Ro MQ2:");
  Serial.println(Ro_mq2);
  Serial.println("Ro MQ9:");
  Serial.println(Ro_mq9);

  if (isValidRo(Ro_mq2) && isValidRo(Ro_mq9)) {
    available = true;
  } else {
    //Print error screen
  }

}

/**
  * Loop method.
  */
void loop() {

  if (available) {
    GAS_S = 0; CO = 0;

    selectChannel(MQ2_CHANNEL);
    Serial.println("MQ2");
    GAS_S = readMQData(MQ2_PIN, RL_VAL_MQ2, Ro_mq2, LPCurve);

    selectChannel(MQ9_CHANNEL);
    Serial.println("MQ9");
    CO = readMQData(MQ9_PIN, RL_VAL_MQ9, Ro_mq9, COCurve);

    sensorFrame(GAS_S, CO);

  }  
}

/**
  * Select the mux channel
  */
void selectChannel(int channel) {
  if (channel == MQ2_CHANNEL) {
    digitalWrite(D0, LOW);
    digitalWrite(D1, LOW);
    digitalWrite(D2, LOW);
  } else if (channel == MQ9_CHANNEL) {
    digitalWrite(D0, HIGH);
    digitalWrite(D1, LOW);
    digitalWrite(D2, LOW);
  }
  delay(200); //Secure time to change channel
}

/**
  * This function validate the ro value
  */
bool isValidRo(float ro) {
  if((isnan(ro)) || (ro <= 0.01) || (ro >= 10000.0)) {
    return false;
  } 

  return true;
}

/**
  * Calibrating the MQ sensor
  */
float MQCalibration(int mq_pin, float RO_CLEAN_AIR_FACT, float RL_VAL) {
  float val = 0;

  for (int i = 0; i < CALIB_SAMPLE_TIMES; ++i) {
    int raw = analogReadAverage(5, 20, mq_pin);
    float rs = MQResistanceCalc(raw, RL_VAL);
    val += rs;
    delay(CALIB_SAMPLE_INTERVAL);
  }

  float rs_avg = val / (float)CALIB_SAMPLE_TIMES;
  float ro = rs_avg / RO_CLEAN_AIR_FACT;

  Serial.print("Calibracion completa. Ro = ");
  Serial.println(ro);
  
  return ro;
}

/**
  * Average of raw input
  */
int analogReadAverage(int samples, int interval, int adc_raw) {
  long avSum = 0;

  //Serial.println("INSIDE ANALOG READ AVERAGE");

  for (int i = 0; i < samples; i++) {
    int lect = analogRead(adc_raw);
    avSum = avSum + (long)lect;
    //Serial.print("Input: ");
    //Serial.print(lect);
    //Serial.print(" - Sum: ");
    //Serial.println(avSum);
    delay(interval);
  }

  int avRaw = (int)(((int)avSum) / samples);
  //Serial.print("Result: ");
  //Serial.println(avRaw);
  return avRaw;
}

/**
  * Calculating the MQ Resistance
  */
float MQResistanceCalc(int raw_adc, float RL_MQ) {
  //return (((float)RL_VAL_MQ2 * (1023 - raw_adc) / raw_adc));
  if (raw_adc <= 0) {
    return 99999.0;
  } 

  float sensorVolt = rawToSensorVolt(raw_adc);

  if (sensorVolt <= 0.00001) {
    return 99999.0;
  } 

  float rs = (RL_MQ * (5.0 - sensorVolt)) / sensorVolt;
  return rs;
}

/**
  * Calculate the sensor voltaje from raw input
  */
float rawToSensorVolt(int raw_adc) {
  float adcVolt = ((float)raw_adc / 1023.0) * 3.3;
  float sensorVolt = adcVolt / (330.0 / (330.0 + 180.0));
  return sensorVolt;
}

/**
  * Convert the Rs/Ro ratio to ppm
  */
long MQGetGasPercentage(float rs_ro_ratio, float *pcurve) {
  if (rs_ro_ratio <= 0.000001) {
    return 0;
  }

  float log_rsro = log10(rs_ro_ratio);
  float exponent = ((log_rsro - pcurve[1]) / pcurve[2]) + pcurve[0];
  float result = pow(10, exponent);

  if(isinf(result)) {
    return 0;
  }

  return (long)result;
}

/**
  * Read the MQ Data
  */
long readMQData(int mq_pin, float RL_MQ, float ro_mq, float *curve) {
  int rawAvg = analogReadAverage(READ_SAMPLE_TIMES, READ_SAMPLE_INTERVAL, mq_pin);
  float sensorVolt = rawToSensorVolt(rawAvg);
  float rs = MQResistanceCalc(rawAvg, RL_MQ);
  float rs_ro_ratio = rs / ro_mq;
  long ppm = MQGetGasPercentage(rs_ro_ratio, curve);

  Serial.print("Sensor data: ");
  Serial.print(" | ADC avg: ");
  Serial.print(rawAvg);
  Serial.print(" | sensorVolt: ");
  Serial.print(sensorVolt);
  Serial.print(" V | Rs: ");
  Serial.print(rs);
  Serial.print(" kΩ | Rs/Ro: ");
  Serial.print(rs_ro_ratio);
  Serial.print(" | ppm(aprox): ");
  Serial.println(ppm);

  return ppm;
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
void sensorFrame(long GAS_D, int CO_D) {
  //int GAS_LP_dummy = 1234; // Dummy data
  //int CO_2_dummy = 1234;
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

  if (GAS_D < 200.0) {
    display.println("LOW");
  } else if ((GAS_D >= 200.0) && (GAS_D < 10000.0)) {
    display.println(String(GAS_D));
  } else if (GAS_D >= 10000.0){
    display.println("HIGH");
  }

  display.setCursor(81, 60);

  if (CO_D < 200.0) {
    display.println("LOW");
  } else if ((CO_D >= 200.0) && (CO_D < 10000.0)) {
    display.println(String(CO_D));
  } else if (CO_D >= 10000.0){
    display.println("HIGH");
  }
  display.display();
}