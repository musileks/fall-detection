#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <mysql.h>


#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_TEAPOT
#define INTERRUPT_PIN 12 
#define LED_PIN 16 
#define time 10

///*** MPU control/status vars ***///

MPU6050 mpu;

bool dmpReady = false;  
uint8_t mpuIntStatus;   
uint8_t devStatus;      
uint16_t packetSize;    
uint16_t fifoCount;     
uint8_t fifoBuffer[64]; 

Quaternion q;           
VectorFloat gravity;    
float ypr[3];         
float tmp_p,tmp_r;
float yaw = 0;
float pitch = 0;
float roll = 0;
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
int16_t ax, ay, az;
int16_t gx, gy, gz;
float Ax=0, Ay=0, Az=0, Gx=0, Gy=0, Gz=0;

///*** Device setting ***///

const int buttonPin = 14;     // the number of the pushbutton pin
const int buzzerPin =  13;      // the number of the buzzer pin
int buttonState = 0;  
int buzzerState = 0; 
bool blinkState = false;


///*** Wifi setting ***///

WiFiClient client; 
const char* ssid = "iPhone";
const char* password = "unlimiteD12345";

///*** for Fall detection and algorithm ***///

String fallstatus;
boolean fall = false; //stores if a fall has occurred
boolean trigger1=false; //stores if first trigger (lower threshold) has occurred
boolean trigger2=false; //stores if second trigger (upper threshold) has occurred
boolean trigger3=false; //stores if third trigger (orientation change) has occurred

byte trigger1count=0; //stores the counts past since trigger 1 was set true
byte trigger2count=0; //stores the counts past since trigger 2 was set true
byte trigger3count=0; //stores the counts past since trigger 3 was set true
int angleChange=0;

unsigned long timer = 0;
unsigned long time_1,time_2,time_s,pre_time,time_out;
int state = 0;


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {

  Serial.begin(115200);
  while (!Serial); 
 
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)  
    {
            delay(500);
            Serial.print(".");
    }
  Serial.println("");
  Serial.println("WiFi connected");   
  Serial.println("IP address: ");  
  Serial.println(WiFi.localIP());  
  
  Wire.begin();

    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

///*** value from calibration sensor ***///
  
    mpu.setXGyroOffset(6);
    mpu.setYGyroOffset(-27);
    mpu.setZGyroOffset(-13);
    mpu.setXAccelOffset(-2092);
    mpu.setYAccelOffset(-563);
    mpu.setZAccelOffset(1165); 

     if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    
  
      //delay(100);
    pinMode(buzzerPin, OUTPUT);
    pinMode(buttonPin, INPUT);
    pinMode(LED_PIN, OUTPUT);
}

void(*resetFunc)(void) = 0;

int value = 0;

void loop() {

    buttonState = digitalRead(buttonPin);
    
   if (!dmpReady) return;
        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();
        fifoCount = mpu.getFIFOCount();
   if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
    {
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));
    }
    else if (mpuIntStatus & 0x02)
    {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        mpu.resetFIFO();
        fifoCount -= packetSize;
      
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        Ax = (ax)/16384.00;
        Ay = (ay)/16384.00;
        Az = (az)/16384.00;
        Gx = (gx)/131.00;
        Gy = (gy)/131.00;
        Gz = (gz)/131.00;
    
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            yaw = ypr[0]* 180/M_PI;
            pitch = ypr[1]* 180/M_PI;
            roll = ypr[2]* 180/M_PI;
        
    ///*** show value of Ax Ay Az and Gx Gy Gz ***///
    
            Serial.print("\t");
            Serial.print("accel\t"); 
            Serial.print(Ax); 
            Serial.print("\t");         
            Serial.print(Ay);
            Serial.print("\t");
            Serial.print(Az); 
            Serial.print("\t");
            Serial.print("gyro\t"); 
            Serial.print(Gx); 
            Serial.print("\t");         
            Serial.print(Gy);
            Serial.print("\t");
            Serial.print(Gz);
            Serial.print("\t"); 
        
    ///*** show value of Yaw Pitch Roll ***///
        
            Serial.print("ypr\t");
            Serial.print(yaw);
            Serial.print("\t");
            Serial.print(pitch);
            Serial.print("\t");
            Serial.println(roll);
            
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);

        ///*** show value of Yaw Pitch Roll ***///
        
        timer = millis();
        buttonState = digitalRead(buttonPin);
          if(state == 0)
          {             
            check_fall(); //Algorithm is used
            
            if(buttonState == LOW)
            {
              unsigned long time_ack = ((timer - pre_time) / 1000);
              if ( time_s >= 0.0) 
                 {
                  Serial.println("I got some problem at state 0");
                  delay(50);
                  state = 2; //Go to State 2
                  }
              }
              else 
              {
                pre_time = timer;
              }
            }
           else if (state == 1)
           {
            Serial.println("Get into State 1");
             buttonState = digitalRead(buttonPin);
             if(((timer - time_out)/1000) < 5)
             {
              digitalWrite(LED_PIN,HIGH);
              delay(500);
              digitalWrite(LED_PIN,LOW);
              delay(500);
             
             if(buttonState == LOW)
             {
              unsigned long time_ack = ((timer - pre_time) / 1000);
               if(time_ack >= 1)
               Serial.println("no problem");
               analogWrite(buzzerPin,0);
               delay(50);
               state = 4; //Go to State 4
             }
             else
             {
              pre_time = timer;
             }
           }
           else
           {
             Serial.println("FALL !! Send notification!");
             fallstatus = "true";
             send_data();
             sound();
             state = 2;
           }
           }
           else if (state == 2)
           {
             Serial.println("state2");
            buttonState = digitalRead(buttonPin);
           
            if(buttonState == LOW)
            {
              unsigned long time_ack = ((timer - pre_time) / 1000);
               if(time_ack >=1)
               {
                analogWrite(buzzerPin,0);
                //fallstatus = "FALSE";
                state = 3; //Go to State 3
               }
               else
             {
              pre_time = timer;
             }
            }
           }
        else if (state == 3)
        {
          buttonState = digitalRead(buttonPin);
      if (buttonState == LOW) {

      } else {
        delay(50);
        //fallstatus = "FALSE";
        state = 0;
        }
        }
         else if (state == 4) {
      buttonState = digitalRead(buttonPin);
      if (buttonState == LOW) {

      } else {
        delay(50);
        //fallstatus = "FALSE";
        state = 0;
      }
    }
    //send_2();
    //send_data(); //Send value to database
    //delay(50);
  }
}


void check_fall()
{
            if (trigger3==true)
            {
               trigger3count++;
               Serial.println(trigger3count);
               if (trigger3count>=5)
               { 
                ///*** condition of Az after fall (lying) ***/// 4th
                  if (Az >= -0.2 && Az <= 0.2)
                  { 
                      fall=true;  trigger3=false; trigger3count=0;
                      Serial.println(angleChange);
                  }
                  else
                  { 
                    ///*** user regained normal orientation ***///
                     trigger3=false; trigger3count=0;
                     Serial.println("TRIGGER 3 DEACTIVATED");
                  }
               }
             }
             
            if (fall==true)
            { 
              ///*** event of fall detection ***///
              Serial.println("FALL DETECTED");
              state = 1;
              fall=false;
            }
              
            if (trigger2count>=5)
            { 
              trigger2=false; trigger2count=0;
              Serial.println("TRIGGER 2 DECACTIVATED");
            }
              
            if (trigger1count>=5)
            {
              trigger1=false; trigger1count=0;
              Serial.println("TRIGGER 1 DECACTIVATED");
            }
              
            if (trigger2==true)
            {
              ///*** condition of Pitch and Roll ***/// 3rd
              if ((roll <= -67.3 || roll >= 56.72) || (pitch <= -45.25 || pitch >= 54.18))
                { 
                  trigger3=true; trigger2=false; trigger2count=0;
                  Serial.println(angleChange);
                  Serial.println("TRIGGER 3 ACTIVATED");
                }
             }

              if (trigger1==true)
              {
              trigger1count++;
              
              ///*** condition of Gx Gy Gz ***/// 2nd
              if ((Gx > 4.3 || Gx < -4.82) || (Gy > 5.12 || Gy < -5.4) || (Gz > 7 || Gz < -7.1))
                { 
                  trigger2=true;
                  Serial.println("TRIGGER 2 ACTIVATED");
                  trigger1=false; trigger1count=0;
                }
              }
    
            ///*** contition of Ax Ay Az ***///  1st
            if (((Ax < -0.88 || Ax > 1 )&& (Az < -0.89 || Az > 1)) || ((Ay < -0.98 || Ay > 1) && (Az < -0.89 || Az > 1.2)) && trigger2==false)
              {
                trigger1=true;
                Serial.println("TRIGGER 1 ACTIVATED");
              }
            delay(100);
}

void sound()
{
    int freq;
    for(freq = 500; freq < 2000; freq += 15)
         {
           tone(buzzerPin, freq, time);    
           delay(50);
         }
      for(freq = 2000; freq > 500; freq -= 15) 
         {
           tone(buzzerPin, freq, time);    
           delay(50);
         }
}

void buzzer(unsigned char delayms)
{
  analogWrite(buzzerPin,100);
  delay(delayms);
  analogWrite(buzzerPin,0);
  delay(delayms);
}

void send_data()
{
   HTTPClient http,http2;
    String url = "http://172.20.10.3/add3.php?"; // send "true" when falls happened
    //String url = "http://172.20.10.3/add2.php?"; // send sensor data (Ax Ay Az Gx Gy Gz Yaw Pitch Roll) 
    //url += (String)value;    
  /*  url += "ax=";
    url +=  Ax;
    url += "&ay=";
    url +=  Ay;    
    url += "&az=";
    url +=  Az;  
    url += "&gx=";
    url +=  Gx;
    url += "&gy=";
    url +=  Gy;
    url += "&gz=";
    url +=  Gz;
    url += "&yaw=";
    url +=  yaw;
    url += "&pitch=";
    url +=  pitch;
    url += "&roll=";
    url +=  roll; */ 
    url += "user_name=";
    url += "me";
    url += "&fall_detect=";
    url += fallstatus;
    url += "&phonenum=";
    url += "0955519744";
    
  ///*** Send data to database by using PHP ***///
    http.begin(url);
    int httpCode = http.GET();
    if (httpCode > 0) 
    {
      if (httpCode == HTTP_CODE_OK) 
        {
          String payload = http.getString();
        }
    } 
    else 
    {
      Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
    }
    http.end(); 
}


