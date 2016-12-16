/*
 * 
 */


extern "C" {
#include "user_interface.h"
#include "sntp.h"
}
#include "FS.h"
#include <Wire.h>
#include "BME280.h"
#include "RX8025.h"
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <TimeLib.h>

#include <ESP8266HTTPClient.h> 

#define SERIAL_BAURATE  115200
#define SDA 14
#define SCL 13
//definition of deep-sleep time
#define PERION_1HOUR 3600000000
#define PERIOD_30MIN 1800000000
#define PERIOD_10MIN 600000000
#define PERIOD_5MIN  300000000

BME280 bme280;
RX8025 rx8025;

#define MaxDataArea (512 - 4 - sizeof(int) -sizeof(time_t))
#define MaxDataBlocks ((MaxDataArea - sizeof(int))/sizeof(AtomInfo))

const char* ssid     = "**** ssid ****";
const char* password = "**** passwd ****";
const char* remote_host = "http://www.biwakobass.org/atominfo.php";
const char content_type = "Content-Type";
const char content_value = "application/json";
const char* fpath = "/f.txt";

int sleepCount = 0;
time_t curtime;

/*
 * @brief mount file system
 */
void initFS() {
  if (!SPIFFS.begin()) {
    Serial.println("Mount Error");
    //turn on error led
    turnOnErrorLed();
  }
  FSInfo fs_info;
  if (SPIFFS.info(fs_info)) {
    Serial.print("totalBytes:");
    Serial.println(fs_info.totalBytes);
    Serial.print("usedBytes:");
    Serial.println(fs_info.usedBytes);
    Serial.println();
  }
}

/*
 * @brief The function initialize serial port with the value of SERIAL_BAURATE
 * variable. 
 * 
 * @note Default value is 115200 bps
 * 
 */
void initSerial() {
  Serial.begin(SERIAL_BAURATE);
  while (!Serial);
  Serial.println();
}

/*
*  @brief This function let the file remove and delete saved data.
*  
*  @retval 0 -> Suceess to remove the file
*  @retval -1 -> Fail to remove the file
*  
*/
int clearFS() {
  int flag = SPIFFS.remove(fpath);
  if (!flag) {
    Serial.println("Failed to Remove File");
    //turn on red-led for error handling
    turnOnErrorLed();
    return -1;
  }
  return 0;
}
/*
 * @brief open the file which contains sets of time, temperature, pressure and humidity.
 * The system need to know when it should connect with wifi and send a httpserver data, 
 * the function let the system know count of a deep-sleep time as well.
 * 
 * @retval row number -> number of saved data
 * @retval -1 ->Error
*/
int getSleepCount() {
  int cnt = 0;
  String tmp;
  File file = SPIFFS.open(fpath, "a+");
  if (!file) {
    //turn on red led for error
    turnOnErrorLed();
    Serial.println("File Open Failed");
    return -1;
  }
  while (file.available()) {
    tmp = file.readStringUntil('\n');
    Serial.println(tmp);
    cnt++;
  }
  file.close();
  return cnt;
}

/*
 * @brief This function is use for Connect with a Wifi Station
 * 
 * @note Retry 10 times if it is not possible to connect. After that 
 * program exit this function with Error.
 * 
 * @retval 0 -> success 
 * @retval -1 -> Error(Timeout)
 *
 */
int ConnectWifi() {
  int conCnt = 0;
  time_t current;
  Serial.println("Connecting with Wifi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(150);
    Serial.print(".");
    //green-led blink, let user know the connecting

    conCnt++;
    //Connection Timeout handling
    if (conCnt > 150) {
        turnOnErrorLed();
      //timer loop which will be turn-off
        return -1;
    }
  }
  Serial.println();
  return 0;
}

time_t setCurrentRTC(){
 
  time_t current;
  //  if(rx8025.needInit()){
      current = getSntpTime();
      rx8025.begin(SDA, SCL);
      rx8025.writeRTC(current);
      Serial.println("connect with ntp server");
      Serial.print(current);
      Serial.println();
//  }
  return current;
}

/*
 * @brief This function send data to the http server with json format
 * 
 * @note After sending the data suceessfully, the system will delete
 * the temporally saved data.
 * 　
 * @retval 0 -> Success to send the data
 * @retval -1 -> Fail to send the data
 * @retval -2 -> Fial to remvoe the file
 */
int sendDataHttpServer() {
    int ret;
    Serial.println("SendData2HttpServer");
    HTTPClient http;
    byte mac[6];
    char mac_addr[64];
    String tmp, result;
    int len;
    int retval;
    WiFi.macAddress(mac);
    Serial.print("macAddress:");
    sprintf(mac_addr, "%x:%x:%x:%x:%x:%x", mac[0], mac[1], mac[2], mac[3],mac[4],mac[5]);
    Serial.println(mac_addr);
    File file = SPIFFS.open(fpath, "a+");    
    if (!file) {
      //turn on red led for error
      turnOnErrorLed();
      Serial.println("File Open Failed");
      return -1;
    }
    tmp = "{\"device\":{\"name\":\"esp8266\", \"id\":\"";
    tmp += ssid;
    tmp += "\"}, \"data\":[ ";
    while (file.available()) {
      tmp += file.readStringUntil('\n');
      tmp +=",";
    }
    len = tmp.length();
    //remove addtional comma
    result = tmp.substring(0, len-1);  
    result += "] }";
    http.begin(remote_host);
    http.addHeader(content_type, content_value); 
    ret = http.POST(result);
    String payload = http.getString();
    Serial.println(payload);
    http.end();
    retval = clearFS();      //remove saved data    
    return retval;
}

/*
 * @brief get temperature, pressure and humid from bme280 module.
 *        get time variable from rtc module as well.
 *  
 * @note after getting the variable, it keep in file system.
 */
int saveAtomInfo() {
  float temp;
  float pressure;
  float humid;
  char buf[512];
  char t[6], p[8], h[6];
  File file = SPIFFS.open(fpath, "a+");
  if (!file) {
    Serial.println("Fail to open File saveAtomInfo");
    turnOnErrorLed();
    return -1;
  }
  temp = bme280.readTemperature();
  pressure = bme280.readPressure();
  humid = bme280.readHumidity();
  Serial.print("time:");
  Serial.println(curtime);
  Serial.print("temperature:");
  Serial.println(temp);
  Serial.print("pressure:");
  Serial.println(pressure);
  Serial.print("humidity:");
  Serial.println(humid);
  Serial.println("--------------------------------");
  dtostrf(pressure, 3, 1, p);
  dtostrf(temp, 3, 1, t);
  dtostrf(humid, 3, 1, h);  
  sprintf(buf, "{\"timestamp\": \"%d\" , \"temp\": \"%s\", \"pre\": \"%s\", \"hum\": \"%s\" }",curtime, t, p, h);
  file.println(buf);
  file.close();
  Serial.println("File Close data writing");
  return 0;
}

/*
 * @brief Initialize I2C and BME280 module.
 */
void initI2C(){
    Wire.begin(SDA, SCL);
    bme280.begin(SDA, SCL);
}

/*
 * @bried Initialize RX8250(RTC) module.
 * @note read time variable from rtc module.
 *       update precise timestamp as estblishing wifi connection.
 */
time_t initRTC(){
  time_t current;
  current = rx8025.readRTC();
  Serial.print("RTC:");
  Serial.println(current);
  return current;
}

uint32_t getSntpTime() {
    configTime(0 * 3600, 0, "ntp.nict.jp", NULL, NULL);

    uint32_t t = 0;
    int cnt = 0;
    while (t == 0) {
        t = sntp_get_current_timestamp();
        delay(10);
        if (++cnt > 100) {
            break;
        }
    }
//    Serial.print("timestamp ntp.nict.jp:");
//    Serial.println(t);
    return t;
}

/*
 * @brief This method describes initial sequence every time this system boot
 * 
 * @note 1.initialize Serial if it is DEBUG
 *       2.initialize FileSystem
 *       3.initialzie I2C module(BME280)
 *       4.read saved data from file system
 *       5.connect with Wifi if sleep count is more than 5
 *         send saved data to httpserver
 *       6.save data to file system
 *       7.enter deep-sleep mode
 */
void setup() {
  initSerial();   //initialize serial port   

  initFS();       //initialize file system
  delay(10);
  initI2C();      //initialize I2C
  delay(10);
  curtime = initRTC();
  //get the count of entering deep-sleep mode
  sleepCount = getSleepCount();
  Serial.print("Sleep count:");
  Serial.println(sleepCount);
  if (sleepCount >= 5) {
    if(!ConnectWifi()) {
      curtime = setCurrentRTC();
      saveAtomInfo();
      sendDataHttpServer();
    }
  } else {
    saveAtomInfo();
  }

  Serial.println("Sleep ESP8266 WAKE_RF_DEFAULT");
  //reset BME280 module for preventing illegal state
  bme280.reset();
  //1min sleep for test
  //1 hour-> PERION_1HOUR 3600000000
  //30min -> PERIOD_30MIN 1800000000
  //10min -> PERIOD_10MIN 600000000
  //5min  -> PERIOD_5MIN  300000000
  ESP.deepSleep(PERIOD_5MIN, WAKE_RF_DEFAULT);
  delay(1000);
}

/*
 * @brief in this system, it does need at all.
 * 
 * @note esp8266 cannot use analogRead method, instead use system_adc_read method.
 */
float getAnalogTemperature() {
  float value;
  int t = system_adc_read();
  value = (float)(t / 1023.0 * 100);
  return value;
}
/*
 * print debug method
 */
void printDebug() {
  int intsize = sizeof(int);
  int infosize = sizeof(AtomInfo);
  int timesize = sizeof(time_t);
  Serial.println("---------------------------------");
  Serial.print("sizeof(int):");
  Serial.println(intsize);
  Serial.print("sizeof(AtomInfo):");
  Serial.println(infosize);
  Serial.print("sizeof(time_t):");
  Serial.println(timesize);

  Serial.print("MaxDataArea:");
  Serial.println(MaxDataArea);
  Serial.print("MaxDataBlocks:");
  Serial.println(MaxDataBlocks);
  Serial.print("Data:");
  Serial.println(sizeof(Data));
}
/*
 * @brief turn on Error LED. just port set for red-led
 */
void turnOnErrorLed(){
  //pinMode(XX, YY);
  Serial.println("turn on error led");
}
void loop() {
  // put your main code here, to run repeatedly:
  // do nothing here
}
