/*
 * makesmart_lock.ino
 * 
 *  Created on: 2021-02-05
 *      Author: cooper @ makesmart.net
 *      Thank you for this great library!
 *      
 * This example shows how to:
 * 1. define a lock accessory and its characteristics in my_accessory.c
 * 2. get the target-state sent from iOS Home APP.
 * 3. report the current-state value to HomeKit.
 * 
 * you can use both:
 *    void open_lock(){}
 * and
 *    void close_lock(){}
 *    
 * at the end of this file to let the lock-mechanism do whatever you want. 
 * 
 * 
 * Pairing Code: 123-45-678
 * 
 * 
 * You should:
 * 1. read and use the Example01_TemperatureSensor with detailed comments
 *    to know the basic concept and usage of this library before other examples。
 * 2. erase the full flash or call homekit_storage_reset() in setup()
 *    to remove the previous HomeKit pairing storage and
 *    enable the pairing with the new accessory of this new HomeKit example.
 * 
 */

#include <Arduino.h>
#include <arduino_homekit_server.h>
#include "wifi_info.h"
#include <Keypad.h>
#include <Servo.h>

#define LOG_D(fmt, ...)   printf_P(PSTR(fmt "\n") , ##__VA_ARGS__);
#define insideButton D1 
#define insideLed D7
#define keyB1 D0
#define keyB2 D5
#define keyB3 D3
#define buzzer D6
#define outsideLed D4
#define servoPin D2
#define doorPosition A0
Servo lockServo;

int lockDeg=0, unlockDeg=180, keyChar, lockState, passCount=0, CorrectPassCount=0, CorrectLockCount=0, doorState, doorPos;
int enteredPasscode[6], Password[6]={1,2,2,1,2,1}, lockCode[4]={2,2,2,2}, resetCode[10]={1,1,2,2,3,3,3,2,2,1};
unsigned long Time=0, buzTime=0;
 
void setup() 
{
  Serial.begin(115200);
  wifi_connect();
 // homekit_storage_reset();
  my_homekit_setup();
  pinMode(insideButton, INPUT_PULLUP);
  pinMode(insideLed, OUTPUT);
  pinMode(keyB1, INPUT_PULLUP);
  pinMode(keyB2, INPUT_PULLUP);
  pinMode(keyB3, INPUT_PULLUP);
  pinMode(doorPosition , INPUT_PULLUP);
  pinMode(buzzer, OUTPUT);
  pinMode(outsideLed, OUTPUT);

  close_lock(0);

  buzzerTone(3);
}











void loop() {
  my_homekit_loop();
  delay(10);
}













//==============================
// HomeKit setup and loop
//==============================

// access lock-mechanism HomeKit characteristics defined in my_accessory.c
extern "C" homekit_server_config_t config;
extern "C" homekit_characteristic_t cha_lock_current_state;
extern "C" homekit_characteristic_t cha_lock_target_state;
static uint32_t next_heap_millis = 0;












// called when the lock-mechanism target-set is changed by iOS Home APP
void set_lock(const homekit_value_t value) 
{
  
  uint8_t state = value.int_value;
  cha_lock_current_state.value.int_value = state;
  cha_lock_target_state.value.int_value = state;
  doorState=state;
  
  if(state == 0)
  {
    open_lock(1);
    report_to_homekit();
  }
  if(state == 1)
  {
    if(doorPos==0)
    {
      close_lock(1);
      report_to_homekit();
    }
    else if(doorPos==1)
    {
      Serial.println("Tried to lock with phone but door is open");
      state = 0;
      doorState=state;
      cha_lock_current_state.value.int_value = state;
      cha_lock_target_state.value.int_value = state;
      report_to_homekit();
    }
    
  }
  
  //report the lock-mechanism current-sate to HomeKit
 // report_to_homekit();
}











void my_homekit_setup() {
  
  cha_lock_target_state.setter = set_lock;
  arduino_homekit_setup(&config);

  
}












void my_homekit_loop() {

  if((analogRead(doorPosition))<500)
  {
    doorPos=1; //open
  }
  else if((analogRead(doorPosition))>500)
  {
    doorPos=0; //closed
  }

  
  //Serial.println(doorPos);
  arduino_homekit_loop();
  inButtonPress();
  if(doorState==1)
  {
    passCode(1);
  }
  else if(doorState==0)
  {
    passCode(0);
  }




  
  const uint32_t t = millis();
  if (t > next_heap_millis) {
    // show heap info every 30 seconds
    next_heap_millis = t + 30 * 1000;
    LOG_D("Free heap: %d, HomeKit clients: %d",
        ESP.getFreeHeap(), arduino_homekit_connected_clients_count());

  }
}











void passCode(int st)
{
  keyChar=keyPress();
  if((millis()-Time)>=10000 && Time!=0)
  {
    passCount=0;
    Time=0;
    buzzerTone(2);
    digitalWrite(outsideLed, LOW);
  }
  if(keyChar!=0)
  {
    Time=millis();
    digitalWrite(outsideLed, HIGH);
    enteredPasscode[passCount]=keyChar;
    passCount++;
    
    if(passCount==6 && st==1)
    {
        passCount=0;
        for(int i=0;i<6;i++)
        {
            if(enteredPasscode[i]==Password[i])
            {
              CorrectPassCount++;
            }
        }
        
        Serial.print("you entered: ");
        for(int i=0;i<6;i++)
        {
            Serial.print(enteredPasscode[i]);
        }
          
        if(CorrectPassCount==6)
        {
            CorrectPassCount=0;
            Serial.println("\nCorrect password!");
            Serial.print("locked via keypad");
            open_lock(0);
            buzzerTone(1);
            Time=0;
            digitalWrite(outsideLed, LOW);
        }
        else
        {
          CorrectPassCount=0;
          Serial.println("\nWrongggggg fucking password!!");
          buzzerTone(0);
          Time=0;
        }
    }
    else if(st==0 && passCount==4)
    {
        passCount=0;
        for(int i=0;i<4;i++)
        {
            if(enteredPasscode[i]==lockCode[i])
            {
                CorrectLockCount++;
            }
        }
        
        Serial.print("you entered: ");
            for(int i=0;i<4;i++)
            {
                Serial.print(enteredPasscode[i]);
            }
          
        if(CorrectLockCount==4)
        {
            if(doorPos==0)
            {
              CorrectLockCount=0;
              Serial.println("\nCorrect password!");
              Serial.print("unlocked via keypad");
              close_lock(0);
              buzzerTone(1);
              Time=0;
              digitalWrite(outsideLed, LOW);
            }
            else if(doorPos==1)
            {
              CorrectLockCount=0;
              Serial.println("keypad attempted to lock but door is open");
              buzzerTone(4);
              Time=0;
              digitalWrite(outsideLed, LOW);
            }
          
        }
        else
        {
          CorrectLockCount=0;
          Serial.println("\nWrongggggg CODE!!");
          buzzerTone(0);
          Time=0;
          
        }
    }

  } 

}






int keyPress()
{
  int kee=0, keyPressStopper=0;
  while(digitalRead(keyB1)==0 || digitalRead(keyB2)==0 || digitalRead(keyB3)==0)
  { 
      tone(buzzer,5000);
      keyPressStopper=10;
      if(digitalRead(keyB1)==0 && digitalRead(keyB2)==1 && digitalRead(keyB3)==1)
      {
        kee=1;
      }
      else if(digitalRead(keyB1)==1 && digitalRead(keyB2)==0 && digitalRead(keyB3)==1)
      {
        kee=2;
      }
      else if(digitalRead(keyB1)==1 && digitalRead(keyB2)==1 && digitalRead(keyB3)==0)
      {
        kee=3;
      }
    
  }
  noTone(buzzer);
  
  if(keyPressStopper==10)
  {
    Serial.println(kee);
    return kee;
  }
    

  return kee;
}





void inButtonPress()
{
  int bt=0, count=0;
  while(digitalRead(insideButton)==0)
  {
    count=1;
  }
  
  if(count==1)
  {
    Serial.println("inside button poressed");
    if(doorState==1)
    {
      open_lock(0);
      Serial.println("unlocked via inside button");
    }
    else if(doorState==0)
    {
      if(doorPos==0)
      {
        close_lock(0);
        Serial.println("locked via inside button");
      }
      else if(doorPos==1)
      {
        Serial.println("attempted to lock by inside button but door is open");
        buzzerTone(4);
      }
      
    }
  }
}






/* use this functions to let your lock mechanism do whatever yoi want */
void open_lock(int fromPhone){
  Serial.println("unsecure"); 
  lockServo.attach(servoPin);
  lockServo.write(unlockDeg);
  delay(700);
  lockServo.detach();
  doorState=0;
  digitalWrite(outsideLed, LOW);
  digitalWrite(insideLed, HIGH);
  if(fromPhone==0)
  {
    cha_lock_current_state.value.int_value = doorState;
    cha_lock_target_state.value.int_value = doorState;
    report_to_homekit();
  }
}

void close_lock(int fromPhone){
  Serial.println("secure");  
  lockServo.attach(servoPin);
  lockServo.write(lockDeg);
  delay(700);
  lockServo.detach();
  doorState=1;
  digitalWrite(outsideLed, LOW);
  digitalWrite(insideLed, LOW);
  if(fromPhone==0)
  {
    cha_lock_current_state.value.int_value = doorState;
    cha_lock_target_state.value.int_value = doorState;
    report_to_homekit();
  }
}

void buzzerTone(int ton)
{
  if(ton==1) //unlocked or locked
  {
    tone(buzzer, 1000);
    delay(100);
    tone(buzzer, 1500);
    delay(100);  
    tone(buzzer, 2000);
    delay(100);
    tone(buzzer, 2500);
    delay(100); 
    tone(buzzer, 3000);
    delay(100);
    tone(buzzer, 3500);
    delay(100);
    tone(buzzer, 4000);
    delay(100); 
    noTone(buzzer);
}
  else if(ton==0) // wrong password or error
  {
    tone(buzzer, 3000);
    delay(100); 
     tone(buzzer, 1000);
    delay(100);  
    tone(buzzer, 3000);
    delay(100); 
     tone(buzzer, 1000);
    delay(100);  
    tone(buzzer, 3000);
    delay(100); 
     tone(buzzer, 1000);
    delay(100);  
    noTone(buzzer);
  }
  else if(ton==2) //inactivity
  {
    tone(buzzer, 2000);
    delay(100);
    noTone(buzzer);
    delay(50);
    tone(buzzer, 2000);
    delay(100);
    noTone(buzzer);
    delay(50);
    tone(buzzer, 2000);
    delay(100);
    noTone(buzzer);
  }
  else if(ton==3) //startup
  {
    digitalWrite(insideLed, HIGH);
    tone(buzzer, 1000);
    delay(100);
    noTone(buzzer);
    delay(100);
    tone(buzzer, 1000);
    delay(100);
    noTone(buzzer);
    delay(100);
    tone(buzzer, 1000);
    delay(50);
    tone(buzzer, 1500);
    delay(50);
    tone(buzzer, 2000);
    delay(50);
    digitalWrite(insideLed, LOW);
    noTone(buzzer);
    
  }
  else if(ton==4) //lock attempt with open door
  {
    
    tone(buzzer, 2000);
    delay(100);
    tone(buzzer, 1500);
    delay(100);
    tone(buzzer, 1000);
    delay(100);
    
   
    noTone(buzzer);
    
  }
}

void report_to_homekit()
{
  homekit_characteristic_notify(&cha_lock_current_state, cha_lock_current_state.value);
  homekit_characteristic_notify(&cha_lock_target_state, cha_lock_target_state.value);
}
