/* Edge Impulse Arduino examples
 * Copyright (c) 2021 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* Includes ---------------------------------------------------------------- */
#include <accelero-metre_inference.h>
#include <Arduino_LSM9DS1.h>
#include <PDM.h>
#include <Arduino_HTS221.h>
#include <Arduino.h>

#ifndef DEBUG 
//Set to 0 if not in building mode. Set to 1 to see messages in the console
#define DEBUG 0
#endif

#ifndef NBDATA    //          +---------+---------+--------+
#define NBDATA 3  // data  -> | tempLow | humHigh | humLow |   
#endif      

/* Constant defines -------------------------------------------------------- */
#define CONVERT_G_TO_MS2    9.80665f

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal

void blink(){
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);
}

// Fonction nécessaire popur convertir la température et l'humidité en 2*2 entiers 8 bits.
// 2 float -> 8 octets
// 2*2 entiers 8 bits -> 4 octets
// Attendtion, tempHigh peut prendre des valeurs négatives, donc c'est un int8_t et non un uint8_t
void conversionFloatToInteger(float temperature, float humidity, __uint8_t data[], int nbData, __int8_t* tempHigh)
{
  bool negatif = 0;

  if (temperature < 0){
    negatif = 1;
    temperature = -temperature;
  }

  //Conversion float -> 2 * uint8_t
  *tempHigh = (__int8_t)temperature;
  float varTemporaire  = (temperature - *tempHigh) * 100;
  data[0] = (__uint8_t)varTemporaire;
  
  if(negatif){
    *tempHigh = -*tempHigh;
  }

  data[1] = (__uint8_t)humidity;
  varTemporaire  = (humidity - data[1]) * 100;
  data[2] = (__uint8_t)varTemporaire;
}

//Get Sigfox ID
String getID(){
  String id = "";
  char output;

  Serial1.print("AT$I=10\r");
  while (!Serial1.available()){
    blink();
  }

  while(Serial1.available()){
    output = Serial1.read();
    id += output;
    delay(10);
  }

  if(DEBUG){
    Serial.print("Sigfox Device ID: ");
    Serial.println(id);
  }

  return id;
}

//Get PAC number
String getPAC(){
  String pac = "";
  char output;

  Serial1.print("AT$I=11\r");
  while (!Serial1.available()){
     blink();
  }

  while(Serial1.available()){
    output = Serial1.read();
    pac += output;
    delay(10);
  }

  if(DEBUG){
    Serial.print("PAC number: ");
    Serial.println(pac);
  }

  return pac;
}

void testSigfox(){
  String answer = "";
  char output;

  Serial1.print("AT\r");
  while (!Serial1.available()){
    blink();
  }
  while(Serial1.available()){
    output = Serial1.read();
    answer += output;
    delay(10);
  }
  if(DEBUG){
    Serial.print("AT answer: ");
    Serial.println(answer);
  }
}

//Send Sigfox Message
void sendMessage(uint8_t msg[], int size) {
  
  if(DEBUG){
    Serial.println("Inside sendMessage");
  }
  

  String status = "";
  String hexChar = "";
  String sigfoxCommand = "";
  char output;

  sigfoxCommand += "AT$SF=";

  for (int i = 0; i < size; i++)
  {
    hexChar = String(msg[i], HEX);

    //padding
    if (hexChar.length() == 1)
    {
      hexChar = "0" + hexChar;
    }

    sigfoxCommand += hexChar;
  }

  if(DEBUG){
    Serial.println("Sending...");
    Serial.println(sigfoxCommand);
  }
  Serial1.print(sigfoxCommand + "\r");


  while (!Serial1.available())
  {
    if(DEBUG){
      Serial.println("Waiting for response");
    }
    delay(1000);
  }

  while (Serial1.available())
  {
    output = (char)Serial1.read();
    status += output;
    delay(10);
  }

  if(DEBUG){
    Serial.println();
    Serial.print("Status \t");
    Serial.println(status);
  }
}

void sendDonne(float temperature, float humidity, bool mvtAlarm, bool bruitAlarm) {

  if(DEBUG){
    Serial.println("Inside sendMessage");
  }
  
  String     status        = "";
  String     charTempHigh  = "";
  String     sigfoxCommand = "AT$SF=";
  String     hexChar;
  __uint8_t  data[NBDATA];
  __uint8_t  alarm;
  __int8_t   tempHigh;
  char       output;
  bool       tempAlarm = 0;
  bool       humAlarm  = 0;

  conversionFloatToInteger(temperature, humidity, data, NBDATA, &tempHigh);

  if (DEBUG){
    Serial.print("TempHigh = "); Serial.println(tempHigh);
    for(int i=0; i<NBDATA; i++){
      Serial.print("data ["); Serial.print(i); Serial.print("] = "); Serial.println(data[i]);
    }
  }

  charTempHigh = String(tempHigh, HEX);
  //padding
  if (charTempHigh.length() == 1)
  {
    charTempHigh = "0" + charTempHigh;
  }
  sigfoxCommand += charTempHigh;
  

  for (uint8_t i=0; i<NBDATA; i++)
  {
    hexChar = String(data[i], HEX);
    //padding
    if (hexChar.length() == 1)
    {
      hexChar = "0" + hexChar;
    }
    sigfoxCommand += hexChar;
  }

  // alarms handling 
  // Seul les 4 LSB de alarm sont nécessaires car contient 4 bool.
  //          +------+---------+-----------+--------+
  // alarm -> |pleurs|mouvement|température|humidité|
  //          +------+---------+-----------+--------+

  if(temperature < 15.0 || temperature > 28.0){
    tempAlarm = 1;
  }
  if(humidity < 40.0 || humidity > 60.0){
    humAlarm = 1;
  }
  alarm = humAlarm + (tempAlarm << 1) + (mvtAlarm << 2) + (bruitAlarm << 3);
  hexChar = String(alarm, HEX);
  //padding
  hexChar = "0" + hexChar;
  sigfoxCommand += hexChar;
  
  
  if(DEBUG){
    Serial.print("pleurs alarm : ");      Serial.println(bruitAlarm);
    Serial.print("mouvement  alarm : ");  Serial.println(mvtAlarm);
    Serial.print("température alarm : "); Serial.println(tempAlarm);
    Serial.print("humidité alarm : ");    Serial.println(humAlarm);
    Serial.print("alarm : ");             Serial.println(alarm);
    Serial.println("Sending...");
    Serial.println(sigfoxCommand);
  }
  Serial1.print(sigfoxCommand + "\r");


  while (!Serial1.available())
  {
    if(DEBUG){
      Serial.println("Waiting for response");
    }
    delay(1000);
  }

  while (Serial1.available())
  {
    output = (char)Serial1.read();
    status += output;
    delay(10);
  }

  if(DEBUG){
    Serial.println();
    Serial.print("Status \t");
    Serial.println(status);
  }
}
/**
* @brief      Arduino setup function
*/
void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    Serial.println("Edge Impulse Inferencing Demo");

    if (!IMU.begin()) {
        ei_printf("Failed to initialize IMU!\r\n");
    }
    else {
        ei_printf("IMU initialized\r\n");
    }

    if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 3) {
        ei_printf("ERR: EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME should be equal to 3 (the 3 sensor axes)\n");
        return;
    }

     // Configure serial port n°1 uised for SigFox communication
  //Serial1.begin(9600);
  //while(!Serial1){
    //Serial.println("Failed to initialize Sigfox port!");
  //}
  //Set up sigfox communication
  delay(100);
  getID();
  delay(100);
  getPAC();

  if (!HTS.begin()) {
    Serial.println("Failed to initialize humidity temperature sensor!");
    while (1);
  }
}

/**
* @brief      Printf function uses vsnprintf and output using Arduino Serial
*
* @param[in]  format     Variable argument list
*/
void ei_printf(const char *format, ...) {
   static char print_buf[1024] = { 0 };

   va_list args;
   va_start(args, format);
   int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
   va_end(args);

   if (r > 0) {
       Serial.write(print_buf);
   }
}

/**
* @brief      Get data and run inferencing
*
* @param[in]  debug  Get debug info if true
*/
void loop()
{
    ei_printf("\nStarting inferencing in 2 seconds...\n");

    delay(2000);

    ei_printf("Sampling...\n");

    // Allocate a buffer here for the values we'll read from the IMU
    float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

    for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += 3) {
        // Determine the next tick (and then sleep later)
        uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);

        IMU.readAcceleration(buffer[ix], buffer[ix + 1], buffer[ix + 2]);

        buffer[ix + 0] *= CONVERT_G_TO_MS2;
        buffer[ix + 1] *= CONVERT_G_TO_MS2;
        buffer[ix + 2] *= CONVERT_G_TO_MS2;

        delayMicroseconds(next_tick - micros());
    }

    // Turn the raw buffer in a signal which we can the classify
    signal_t signal;
    int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0) {
        ei_printf("Failed to create signal from buffer (%d)\n", err);
        return;
    }

    // Run the classifier
    ei_impulse_result_t result = { 0 };

    err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        return;
    }

    // print the predictions
    ei_printf("Predictions ");
    ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
        result.timing.dsp, result.timing.classification, result.timing.anomaly);
    ei_printf(": \n");
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
    }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("    anomaly score: %.3f\n", result.anomaly);
#endif

 //Variable declaration
  float temperature = HTS.readTemperature(CELSIUS); //CELSIUS macro can be replaced by FAHRENHEIT
  float humidity    = HTS.readHumidity();
  bool mouvement    = 0;
  bool pleurs       = 0;

   for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
      if(strcmp(result.classification[ix].label,"baby moving") == 0 && result.classification[ix].value > 0.7) mouvement = 1;
      else mouvement = 0;
   }
  if(DEBUG){
    // print each of the sensor values
    Serial.print("Temperature = "); Serial.print(temperature); Serial.println(" °C");
    Serial.print("Humidity    = "); Serial.print(humidity);    Serial.println(" %");
    // print an empty line
    Serial.println();
  }

  // SigFox
  sendDonne(temperature, humidity, mouvement, pleurs);

  // In the ETSI zone, due to the reglementation, an object cannot emit more than 1% of the time hourly
  // So, 1 hour = 3600 sec
  // 1% of 3600 sec = 36 sec
  // A Sigfox message takes 6 seconds to emit
  // 36 sec / 6 sec = 6 messages per hours -> 1 every 10 minutes
  delay(600000); //delay est en ms
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_ACCELEROMETER
#error "Invalid model for current sensor"
#endif
