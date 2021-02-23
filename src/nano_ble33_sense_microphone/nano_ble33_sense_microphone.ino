
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

// If your target is limited in memory remove this macro to save 10K RAM
#define EIDSP_QUANTIZE_FILTERBANK   0

/* Includes ---------------------------------------------------------------- */
#include <PDM.h>
#include <tomseitz-project-1_inference.h>
#include <Arduino_HTS221.h>

#ifndef DEBUG 
//Set to 0 if not in building mode. Set to 1 to see messages in the console
#define DEBUG 0
#endif

#ifndef NBDATA    //          +---------+---------+--------+
#define NBDATA 3  // data  -> | tempLow | humHigh | humLow |   
#endif      
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


/** Audio buffers, pointers and selectors */
typedef struct {
    int16_t *buffer;
    uint8_t buf_ready;
    uint32_t buf_count;
    uint32_t n_samples;
} inference_t;

static inference_t inference;
static signed short sampleBuffer[2048];
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal

/**
 * @brief      Arduino setup function
 */
void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);

    Serial.println("Edge Impulse Inferencing Demo");

    // summary of inferencing settings (from model_metadata.h)
    ei_printf("Inferencing settings:\n");
    ei_printf("\tInterval: %.2f ms.\n", (float)EI_CLASSIFIER_INTERVAL_MS);
    ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("\tSample length: %d ms.\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16);
    ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) / sizeof(ei_classifier_inferencing_categories[0]));

    if (microphone_inference_start(EI_CLASSIFIER_RAW_SAMPLE_COUNT) == false) {
        ei_printf("ERR: Failed to setup audio sampling\r\n");
        return;
    }

    
  // Configure serial port n°1 uised for SigFox communication
  Serial1.begin(9600);
  while(!Serial1){
    Serial.println("Failed to initialize Sigfox port!");
  }
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
 * @brief      Arduino main function. Runs the inferencing loop.
 */
void loop()
{
    ei_printf("Starting inferencing in 2 seconds...\n");

    delay(2000);

    ei_printf("Recording...\n");

    bool m = microphone_inference_record();
    if (!m) {
        ei_printf("ERR: Failed to record audio...\n");
        return;
    }

    ei_printf("Recording done\n");

    signal_t signal;
    signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
    signal.get_data = &microphone_audio_signal_get_data;
    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR r = run_classifier(&signal, &result, debug_nn);
    if (r != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", r);
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

//Partie capteur/sigfox
  //Variable declaration
  float temperature = HTS.readTemperature(CELSIUS); //CELSIUS macro can be replaced by FAHRENHEIT
  float humidity    = HTS.readHumidity();
  bool mouvement    = 0;
  bool pleurs       = 0;

   for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
      if(strcmp(result.classification[ix].label,"noise background") == 0 && result.classification[ix].value > 0.7) pleurs = 0;
      else pleurs = 1;
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
 * @brief      PDM buffer full callback
 *             Get data and call audio thread callback
 */
static void pdm_data_ready_inference_callback(void)
{
    int bytesAvailable = PDM.available();

    // read into the sample buffer
    int bytesRead = PDM.read((char *)&sampleBuffer[0], bytesAvailable);

    if (inference.buf_ready == 0) {
        for(int i = 0; i < bytesRead>>1; i++) {
            inference.buffer[inference.buf_count++] = sampleBuffer[i];

            if(inference.buf_count >= inference.n_samples) {
                inference.buf_count = 0;
                inference.buf_ready = 1;
                break;
            }
        }
    }
}

/**
 * @brief      Init inferencing struct and setup/start PDM
 *
 * @param[in]  n_samples  The n samples
 *
 * @return     { description_of_the_return_value }
 */
static bool microphone_inference_start(uint32_t n_samples)
{
    inference.buffer = (int16_t *)malloc(n_samples * sizeof(int16_t));

    if(inference.buffer == NULL) {
        return false;
    }

    inference.buf_count  = 0;
    inference.n_samples  = n_samples;
    inference.buf_ready  = 0;

    // configure the data receive callback
    PDM.onReceive(&pdm_data_ready_inference_callback);

    // optionally set the gain, defaults to 20
    PDM.setGain(80);
    PDM.setBufferSize(4096);

    // initialize PDM with:
    // - one channel (mono mode)
    // - a 16 kHz sample rate
    if (!PDM.begin(1, EI_CLASSIFIER_FREQUENCY)) {
        ei_printf("Failed to start PDM!");
        microphone_inference_end();

        return false;
    }

    return true;
}

/**
 * @brief      Wait on new data
 *
 * @return     True when finished
 */
static bool microphone_inference_record(void)
{
    inference.buf_ready = 0;
    inference.buf_count = 0;

    while(inference.buf_ready == 0) {
        delay(10);
    }

    return true;
}

/**
 * Get raw audio signal data
 */
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)
{
    numpy::int16_to_float(&inference.buffer[offset], out_ptr, length);

    return 0;
}

/**
 * @brief      Stop PDM and release buffers
 */
static void microphone_inference_end(void)
{
    PDM.end();
    free(inference.buffer);
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_MICROPHONE
#error "Invalid model for current sensor."
#endif
