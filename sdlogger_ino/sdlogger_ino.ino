//Libraries
#include <Wire.h>
#include <TimeLib.h>
#include <BMA250.h>
#include <RTCZero.h>            // Can only go into sleep on second intervals
                                // Does not have power down flash or SYSTICK modifications
#include <Adafruit_SleepyDog.h> //Has power down flash correct, but not SYSTICK
                                //Allows you to do millisecond sleep, but limited to WDT limit ~18s
#include <SdFat.h>
#include <SPI.h>
#include <ArduinoLowPower.h>    //Use RTC library for timing, therefore can only sleep in second intervals
                                //Has power down flash and SYSTICK correct
// Parameters to change:
char fileName[] = "logfile.csv";

#define INITIAL_SLEEP_TIME 0.5 //hours; How long to sleep on power, allows to delay start of recording to save power
#define RECORD_INTERVAL 10 //seconds; How long to collect data for during each recording interval
#define SLEEP_BETWEEN_INTERVAL 0 //seconds; How long to sleep between recording intervals
#define SAVE_INTERVAL 5 //seconds; How long to delay between SD card saves. Each save uses a lot of power, so we only save occasionally
#define SLEEP_BETWEEN_SAMPLES 60 //milliseconds; Time between samples, determined by data rate of accelerometer
#define TIME_BETWEEN_SAMPLES 8 //milliseconds; Adjustment factor to keep timestamp accurate after sleep
#define DATA_COUNT 4 //Number of data fields you are collecting. For x,y,z accelerations + temperature -> DATA_COUNT=4

#define CHIP_SELECT 10
#define SECONDS_IN_MIN 60
#define SECONDS_IN_HOUR 3600
#define SECONDS_IN_DAY 86400
#define EPOCH_OFFSET 946684800 //Convert Y2K to epoch time, allows timestamp to start at 0
#define error(msg) sd.errorPrint(&SerialUSB)

int wakeupTime; //Records RTC time when processor wakes up
int prevTime; //Records RTC time (second) at each sample
int lastSave; //Record last time data was stored (synced) on SD card
int rtc_start = 1718374097; //<<<<<<<<<<<-------Manually set the start time of recording using epoch time--------<<<<<<<<<<<<
int iteration = 0;
int itest = 0;

RTCZero rtc; //create RTC object
BMA250 accelSensor; //create BMA250 accelerometer object
SdFat sd; //create file system object
SdFile file; //create file object

void setup() {
  //If SD initialization fails, light LED
  if (!sd.begin(CHIP_SELECT, SD_SCK_MHZ(50))) {
    SPI.end();
    lightLED();
  }
  //If file creation fails light LED
  if (!file.open(fileName, O_WRONLY | O_CREAT | O_APPEND)) {
    SPI.end();
    lightLED();
  }

  detectReset(); //Checks reason for reset, for debugging
  file.sync();

  //Initialize RTC
  rtc.begin();
  rtc.setEpoch(0);

  //Initialize Sensor
  Wire.begin();
  accelSensor.begin(BMA250_range_16g, BMA250_update_time_8ms);

  //Set alarm for initial sleep - allows for to delay recording
  rtc.setAlarmEpoch(rtc.getEpoch() + INITIAL_SLEEP_TIME * SECONDS_IN_HOUR);
  rtc.enableAlarm(rtc.MATCH_YYMMDDHHMMSS);
  LowPower.sleep();

  // Record time at wakeup, record initial timestamp time
  wakeupTime = rtc.getEpoch();
  prevTime = rtc.getEpoch() - EPOCH_OFFSET;

}

void loop() {

  // Data array
  double data[DATA_COUNT];

  //Get data
  getAccel(data);

  //Create timestamp
  String timeStampNow = getTimestamp(&prevTime);

  //Record timestamp and data
  writeData(timeStampNow, data);

  //Save to SD card every 6000 lines of data
  itest += 1;
  if (itest % 6000 == 0) {
    file.sync();
  }
}

void getAccel(double pdata[]) {
  accelSensor.read();
  pdata[0] = accelSensor.X;
  pdata[1] = accelSensor.Y;
  pdata[2] = accelSensor.Z;
  pdata[3] = ((accelSensor.rawTemp * 0.5) + 24.0);
}

String getTimestamp(int *pprevTime) {
  int timeStamp = rtc.getEpoch() - EPOCH_OFFSET;

  //RTC only returns whole seconds. Therefore to get a timestamp at the millisecond
  //level, we estimate it. We know the sampling rate of the accelerometer. Therefore, each time
  //the RTC changes, for example from second 11 to second 12, we start a counter at 0.
  //This counter increments each time the accelerometer collects data. The millisecond
  //portion of the time stamp is then estimated as (iteration * time_between_samples)
  
  //If the RTC has changed, set prevTime to new second, and reset the counter to 0
  if (timeStamp != *pprevTime) {
    iteration = 0;
    *pprevTime = timeStamp;
  }

  //If RTC is stil on the same second, increment the counter
  else {
    iteration += 1;
  }

  //Calculate timestamp values and concatenate
  int t_ms = iteration * TIME_BETWEEN_SAMPLES;
  int t_sec = timeStamp % SECONDS_IN_MIN;
  int t_min = (timeStamp / SECONDS_IN_MIN) % SECONDS_IN_MIN;
  int t_hr =  (timeStamp % SECONDS_IN_DAY) / SECONDS_IN_HOUR;
  int t_day = (timeStamp / SECONDS_IN_DAY);

  //Add starting time information
  int rtc_duration = SECONDS_IN_DAY * t_day + SECONDS_IN_HOUR * t_hr + SECONDS_IN_MIN * t_min + t_sec;
  int rtc_tot = rtc_start + rtc_duration;

  String colon = " : ";
  String dot = ".";
  String dash = "-";
  String timeStampString;

  timeStampString = String(year(rtc_tot)) + dash + print2digits(month(rtc_tot)) + dash + print2digits(day(rtc_tot)) + colon + 
                           print2digits(hour(rtc_tot)) + colon + print2digits(minute(rtc_tot)) + colon + print2digits(second(rtc_tot)) + 
                           dot + print3digits(t_ms);
  return timeStampString;
}

String print2digits(int number){
  String output;
  String onezero = "0";
  if (number < 10){
    output = onezero + number;
  }
  else {
    output = number;
  }
  return output;
}

String print3digits(int number){
  String output, onezero, twozero, num;
  onezero = "0";
  twozero = "00";
  num = String(number);
  if (number < 100){
    if (number < 10){
      output = twozero + num;
    }
    if (number >= 10){
      output = onezero + num;
    }
  }
  else {
    output = num;
  }
  return output;
}

void writeData(String timeStamp, double pdata[]) {
  file.print(timeStamp);

  for (byte i = 0; i < DATA_COUNT; i = i + 1) {
    file.write(",");
    file.print(pdata[i]);
  }

  file.println();
}

void detectReset() {
  if (REG_PM_RCAUSE == PM_RCAUSE_SYST) {
    file.println(F("Reset requested by system"));
  }
  if (REG_PM_RCAUSE == PM_RCAUSE_WDT) {
    file.println(F("Reset requested by Watchdog"));
  }
  if (REG_PM_RCAUSE == PM_RCAUSE_EXT) {
    file.println(F("External reset requested"));
  }
  if (REG_PM_RCAUSE == PM_RCAUSE_BOD33) {
    file.println(F("Reset brown out 3.3V"));
  }
  if (REG_PM_RCAUSE == PM_RCAUSE_BOD12) {
    file.println(F("Reset brown out 1.2v"));
  }
  if (REG_PM_RCAUSE == PM_RCAUSE_POR) {
    file.println(F("Normal power on reset"));
  }
}

void lightLED() {
  pinMode(LED_BUILTIN, OUTPUT);
  while (1) {
    digitalWrite(LED_BUILTIN, HIGH);
  }
}
