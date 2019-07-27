//I_Water_systemtest.ino
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <RTClib.h>
#include <DB.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

#define EEPROM_ADDR 1


DB db;
RTC_DS1307 RTC;
DateTime now;
unsigned int i = 1;
long delayTime = 0;
unsigned long previousTime = 0;
unsigned long curTime = 0;
//const int delayTime = 21600; // 21600 is 6 hours in seconds
const short pwrLed = 13;
const short pwrUp = 12;
const short precision = 175; // 1024 / precision = interval

struct Record {
unsigned long time;
unsigned short sensorB;
unsigned short sensorM;
unsigned short sensorT;
unsigned int   _number;
} record;

void(* resetFunc) (void) = 0;//declare reset function at address 0

void setup() {

pinMode(A1, INPUT);
pinMode(A2, INPUT);
pinMode(A3, INPUT);
pinMode(pwrLed, OUTPUT);
pinMode(0, INPUT);
pinMode(pwrUp, OUTPUT);

Serial.begin(9600);
while (!Serial) {;}
Wire.begin();
//create databases
db.create(EEPROM_ADDR,sizeof(record));
db.open(EEPROM_ADDR);
setupRTC();
setupTimer1();
sleepnow();
attachInterrupt(0, isr_printrecords, LOW);
}


void loop() 
{
if (checkTime())
{
digitalWrite(pwrLed, HIGH);
checkSensors();
}
    sleepnow();
}

ISR(TIMER1_COMPA_vect)
{
sleep_disable();
}

void checkSensors()
{
digitalWrite(pwrUp, HIGH);
delay(3000);
record.time        = now.unixtime();
record.sensorB     = analogRead(A1);
record.sensorM     = analogRead(A2);
record.sensorT     = analogRead(A3);
record._number     = i;

digitalWrite(pwrUp, LOW);
db.append(DB_REC record);
printRecords(i);
delay(500);
i++;

//back off on checks after 20 records
adjusttime();
}

void adjusttime()
{
    //skip if nRecs is zero or one
if(db.nRecs() > 1)
    {
        //this needs to change faster when the value changes faster
    int curTrd = record.sensorT;
    int curMrd = record.sensorM;
    int curBrd = record.sensorB;
    long curTime = record.time;

    db.read(db.nRecs()-1, DB_REC record);
    int preTrd = record.sensorT;
    int preMrd = record.sensorM;
    int preBrd = record.sensorB;
    long preTime = record.time;

    float deltaT = (float) (curTrd-preTrd)/(curTime-preTime);
    float deltaM = (float) (curMrd-preMrd)/(curTime-preTime);
    float deltaB = (float) (curBrd-preBrd)/(curTime-preTime);

    long topExpTime;
    long midExpTime;
    long botExpTime;

    short stablized = 0;

    if (deltaT > 0)
    {
        Serial.print("Top Sensor is Increasing: ");
        Serial.println(deltaT/precision, 10);
        topExpTime = preTime + (curTrd+precision-preTrd)/(curTrd-preTrd)*(curTime-preTime);
    }
    else if (deltaT < 0)
    {
        Serial.print("Top Sensor is Decreasing: ");
        Serial.println(deltaT/precision, 10);
        topExpTime = preTime + (curTrd-precision-preTrd)/(curTrd-preTrd)*(curTime-preTime);
    }
    else 
    {
        Serial.println("Top  Sensor  has  Stabalized");
        topExpTime = curTime + 3600;
        stablized++;
    }

    if (deltaM > 0)
    {
        Serial.print("Middle Sensor is Increasing: ");
        Serial.println(deltaM/precision, 10);
        midExpTime = preTime + (curMrd+precision-preMrd)/(curMrd-preMrd)*(curTime-preTime);
    }
    else if (deltaM < 0)
    {
        Serial.print("Middle Sensor is Decreasing: ");
        Serial.println(deltaM/precision, 10);
        midExpTime = preTime + (curMrd-precision-preMrd)/(curMrd-preMrd)*(curTime-preTime);
    }
    else 
    {
        Serial.println("Middle Sensor has Stabalized");
        midExpTime = curTime + 3600;
        stablized++;
    }
    
    if (deltaB > 0)
    {
        Serial.print("Bottom Sensor is Increasing: ");
        Serial.println(deltaB/precision, 10);
        botExpTime = preTime + (curBrd+precision-preBrd)/(curBrd-preMrd)*(curTime-preTime);
    }
    else if (deltaB < 0)
    {
        Serial.print("Bottom Sensor is Decreasing: ");
        Serial.println(deltaB/precision, 10);
        botExpTime = preTime + (curBrd-precision-preBrd)/(curBrd-preMrd)*(curTime-preTime);
    }
    else 
    {
        Serial.println("Bottom Sensor has Stabalized");
        botExpTime = curTime + 3600;
        stablized++;
    }

    if (stablized == 3)
    {
        db.deleteRec(db.nRecs());
        i--;
    }

    delayTime = min(midExpTime, topExpTime);

    delayTime = abs(min(delayTime, botExpTime) - curTime);
    Serial.print("delayTime = ");
    Serial.print(delayTime);
    Serial.println(" ");
    }

}
void isr_printrecords()
{
detachInterrupt(0);
printRecords();
attachInterrupt(0, isr_printrecords, LOW);
}

bool checkTime()
{
    now = RTC.now();
    curTime = now.unixtime();

    if (delayTime <= (curTime - previousTime))
    {
        previousTime = curTime;

        return true;
    }
    else 
    {
        return false;
    }
}

void sleepnow()
{
digitalWrite(pwrLed, LOW);
set_sleep_mode(SLEEP_MODE_IDLE);
sleep_enable();
sleep_mode();
}

void setupRTC()
{
    int j;

    for(j=0;j<5;j++)
    {
    
    if (RTC.isrunning())
    {
        return;
    }
    else
    {
        delay(500);
        Serial.println("RTC is NOT running!");
        Serial.print("Setting Time to: ");
        RTC.begin();
        delay(500);
        Serial.print(__DATE__);
        Serial.println(__TIME__);
        delay(500);
        RTC.adjust(DateTime(__DATE__, __TIME__));
    }
    }
    Serial.println("Failed to set clock... resetting device");
    delay(1000);
    resetFunc();
}

//this on is overloaded
void printRecords()
{
    int recnum;
Serial.print("\n\r");
Serial.println("_id, top, mid, bot, time");

    for(recnum=1;recnum<=(db.nRecs());recnum++)
    {
    db.read(recnum, DB_REC record);

    Serial.print(record._number);
    Serial.print(",");
    Serial.print(record.sensorT);
    Serial.print(",");
    Serial.print(record.sensorM);
    Serial.print(",");
    Serial.print(record.sensorB);
    Serial.print(",");
    Serial.print(record.time);
    Serial.print("\n\r");
    }
    Serial.print("\n\r");

delay(1000);
}

void printRecords(int i)
{
    db.read(i, DB_REC record);

    Serial.print(record._number);
    Serial.print(",");
    Serial.print(record.sensorT);
    Serial.print(",");
    Serial.print(record.sensorM);
    Serial.print(",");
    Serial.print(record.sensorB);
    Serial.print(",");
    Serial.print(record.time);
    Serial.print("\n");
}

void setupTimer1()
{
//configure timer
/// set entire TCCR1A register to 0 to start fresh
TCCR1A = 0x0;
// same for TCCR1B
TCCR1B = 0x0;
//initialize counter value to 0 for starting point
TCNT1  = 0x0;
// set compare match register **(all registers are 16bit on this chip)**
OCR1A = 0x7FFF;
// turn on CTC mode
TCCR1B |= (1 << WGM12);
// Set CS10 and CS12 bits for 1024 (the largest) prescaler
TCCR1B |= (1 << CS12) | (1 << CS10);  
// enable timer compare interrupt (called at TIMER1_COMPA_vect vector)
TIMSK1 |= (1 << OCIE1A);
}