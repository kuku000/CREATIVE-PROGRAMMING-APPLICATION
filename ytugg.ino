//*******************************************************************************
//    Arduino 小車 Multi-Sensors Control 
//   
//    開發環境：Arduino IDE 1.6.5 以上
//    測試使用開發板型號：Arduino UNO R3 / Arduino Genuino
//*******************************************************************************
#include <boarddefs.h>
#include <IRremoteInt.h>
#include <ir_Lego_PF_BitStreamEncoder.h>

#include <IRremote.h>
//#include <Servo.h>

// L298N 馬達驅動板
// 宣告 MotorA 為右邊
// 宣告 MotorB 為左邊

#define MotorA_I1    11 //宣告 I1 接腳
#define MotorA_I2    10  //宣告 I2 接腳
#define MotorB_I3    9 //宣告 I3 接腳
#define MotorB_I4    8  //宣告 I4 接腳
#define MotorA_PWMA    5  //宣告 ENA (PWM調速) 接腳
#define MotorB_PWMB    6  //宣告 ENB (PWM調速) 接腳

// IRremote 紅外線
#define IR_Recv      3   // 宣告紅外線接收接腳
IRrecv irrecv(IR_Recv);  // 宣告 IRrecv 物件來接收紅外線訊號　
decode_results results;  // 宣告解碼變數

// 抓取套件中紅外線遙控器所對應的 IR Code 請勿直接使用本範例中抓到的 IR Code
// 請務必先使用範例資料夾中 IRremote_Test 的範例碼燒入 Arduino 開發板後，使用套件內的紅外線遙控器實際去按下您所
// 要宣告的各按鈕，並透過序列埠監控視窗 Serial Monitor 所顯示對應的數值，實際抓出對應的 IR Code　再宣告下列指令參數

#define IR_Forwards    0xFF629D  // 遙控器方向鍵 上, 前進
#define IR_Back        0xFFA857  // 遙控器方向鍵 下, 後退
#define IR_Stop        0xFF02FD  // 遙控器 OK 鍵, 停止
#define IR_Left        0xFF22DD  // 遙控器方向鍵 左, 左轉
#define IR_Right       0xFFC23D  // 遙控器方向鍵 右, 右轉

#define IR_Tracking    0xFF42BD  // 遙控器 * 鍵, 循跡模式
#define IR_Ultrasonic  0xFF52AD  // 遙控器 # 鍵, 超音波避障模式

// 循線模組
#define SensorLeft    A0  //宣告 左側感測器 輸入腳
#define SensorMiddle  A1  //宣告 中間感測器 輸入腳
#define SensorRight   A2  //宣告 右側感測器 輸入腳
int off_track = 0;        //宣告變數

//sonar HC-SR04 超音波測距模組 HC-SR04 
//sonar pins and datas
int US_Trig = A3;  //宣告超音波模組 Trig 腳位
int US_Echo = A4;  //宣告超音波模組 Echo 腳位
unsigned long sonar_previous_time = 0;
unsigned long sonar_current_time, sonar_pass_time;
float sonar_duty, sonar_distance = 0;

//Laser Module
int lasertrig = A5;
unsigned long laser_on_time = 0;
unsigned long laser_last_time;
bool laserOn= false;

//IR Obstacle pins
const int isObstaclePin = 2;            //紅外線模組 isObstaclePin(OUT) 腳位 連接至腳位2 will use interrupt
volatile bool isObstacle = false;      // HIGH MEANS NO OBSTACLE
const int ledPin = 13;     

void pinFalled()  //ISR for IR Obstacle Module
{
 isObstacle = true;
}  // end of ISR

//Buzzer
const int buzzerPin = 4;

float sonar(unsigned long delayTime){ 
  sonar_current_time = micros();
  sonar_pass_time = sonar_current_time - sonar_previous_time;
  digitalWrite(US_Trig, HIGH);
  if(sonar_pass_time >=delayTime*1000) {
    digitalWrite(US_Trig, LOW);
    sonar_previous_time = micros();
    sonar_duty = pulseIn(US_Echo,HIGH);
    sonar_distance = (sonar_duty/2)/29.4;
    }
  return sonar_distance;
}
void ringTone(int pin) {
  for (int i=0; i<2; i++) { //repeat 10 times
    tone(pin, 523);
     noTone(pin);delay(50);
    tone(pin, 587);
     noTone(pin);delay(50);
    tone(pin, 659);
     noTone(pin);delay(50);
    tone(pin, 698);
     noTone(pin);delay(50);
    tone(pin, 784);
    noTone(pin);delay(50);
    tone(pin, 880);
    noTone(pin);delay(50);
     tone(pin, 698);
     noTone(pin);delay(50);
      tone(pin, 659);
     noTone(pin);delay(50);
      tone(pin, 587);
     noTone(pin);delay(50);
     tone(pin, 523);
     noTone(pin);delay(50);
    
    
    
      
    
    }

  } 
void ringTone2(int pin) {
    tone(pin, 349);  delay(150); //4
    noTone(pin); delay(40);
    tone(pin, 392);  delay(150); //5
    noTone(pin); delay(40); 
    tone(pin, 440);  delay(150); //6
    noTone(pin); delay(40);
    tone(pin, 349);  delay(150); //4
    noTone(pin); delay(40);
    tone(pin,523 );  delay(400); //1
    noTone(pin); delay(60);
    tone(pin, 440);  delay(150); //6
    noTone(pin); delay(40);
    tone(pin, 392);  delay(150); //5
    noTone(pin); delay(40);
    tone(pin, 523);  delay(150); //1
    noTone(pin); delay(40);
    tone(pin, 392);  delay(300); //5
    noTone(pin); delay(40);
    tone(pin, 349);  delay(150); //4
    noTone(pin); delay(40);
    tone(pin, 294);  delay(150); //2
    noTone(pin); delay(40);
    tone(pin, 440);  delay(300); //6
    noTone(pin); delay(40);
     tone(pin, 349);  delay(150);//4
    noTone(pin); delay(40);
    tone(pin, 330);  delay(300); //3
    noTone(pin); delay(60);
    
     tone(pin, 330);  delay(150); //3
    noTone(pin); delay(40);
    tone(pin,294 );  delay(150);//2
    noTone(pin); delay(40);
    tone(pin, 300);  delay(150);//3
    noTone(pin); delay(40);
    tone(pin, 349);  delay(150);//4
    noTone(pin); delay(40);
    tone(pin, 392);  delay(150);//5
    noTone(pin); delay(40);
    tone(pin, 262);  delay(150);//1
    noTone(pin); delay(40);
    tone(pin, 349);  delay(150);//4
    noTone(pin); delay(40);
    tone(pin, 392);  delay(150);//5
    noTone(pin); delay(40);
    
    tone(pin, 440);  delay(150);//6
    noTone(pin); delay(40);
    tone(pin, 494);  delay(150);//B7
    noTone(pin); delay(40);
    tone(pin, 494);  delay(150);//7
    noTone(pin); delay(40);
    tone(pin, 440);  delay(150);//6
    noTone(pin); delay(40);
    tone(pin, 392);  delay(150);//5
    noTone(pin); delay(40);
     tone(pin, 494);  delay(150);//4
    noTone(pin); delay(40);
    tone(pin, 392);  delay(300);//5
    noTone(pin); delay(40);
    
    tone(pin, 587);  delay(150);//4
    noTone(pin); delay(40);

    tone(pin, 392);  delay(150); //5
    noTone(pin); delay(40);
    tone(pin, 440);  delay(150); //6
    noTone(pin); delay(40); 
    tone(pin, 349);  delay(150); //4
    noTone(pin); delay(40);
    tone(pin, 523);  delay(150); //1
    noTone(pin); delay(40);
    tone(pin, 440);  delay(150); //6
    noTone(pin); delay(40);
    tone(pin, 392);  delay(150); //5
    noTone(pin); delay(40);
    tone(pin, 523);  delay(150); //1
    noTone(pin); delay(40);
    
      tone(pin, 392);  delay(150); //5
    noTone(pin); delay(40);
    tone(pin,349 );  delay(150); //4
    noTone(pin); delay(40);
    tone(pin, 294);  delay(150); //2
    noTone(pin); delay(40);
    tone(pin, 330);  delay(150); //3
    noTone(pin); delay(40);
    tone(pin, 349);  delay(150); //4
    noTone(pin); delay(40);
     tone(pin, 262);  delay(300);//1
    noTone(pin); delay(40);
    
    tone(pin, 262);  delay(150); //1
    noTone(pin); delay(40);
    
   tone(pin, 294);  delay(150); //2
    noTone(pin); delay(40);
    tone(pin, 330);  delay(150);//3
    noTone(pin); delay(40);
    tone(pin, 349);  delay(150);//4
    noTone(pin); delay(40);
    tone(pin, 392);  delay(150);//5
    noTone(pin); delay(40);
    tone(pin, 262);  delay(150);//1
    noTone(pin); delay(40);
    tone(pin, 349);  delay(150);//4
    noTone(pin); delay(40);
    tone(pin, 392);  delay(150);//5
    noTone(pin); delay(40);
    tone(pin, 440);  delay(150);//6
    noTone(pin); delay(40);
    
    tone(pin, 494);  delay(150);//B7
    noTone(pin); delay(40);
    tone(pin, 494);  delay(150);//7
    noTone(pin); delay(40);
    tone(pin, 440);  delay(150);//6
    noTone(pin); delay(40);
    tone(pin, 392);  delay(150);//5
    noTone(pin); delay(40);
     tone(pin, 349);  delay(150);//4
    noTone(pin); delay(40);
    tone(pin, 349);  delay(300);//4
    noTone(pin); delay(40);




    
    
} 
void laserfired(unsigned long delayTime) {
  if (!laserOn){
      digitalWrite(lasertrig, HIGH);
      digitalWrite(ledPin, HIGH);
      laser_on_time= micros();
      laserOn=true; 
   }    
  laser_last_time = micros()-laser_on_time;
  if(laser_last_time >= delayTime*1000)
   {
    digitalWrite(lasertrig, LOW);
    digitalWrite(ledPin, LOW);
    laserOn=false;
   } 
}
// 伺服馬達(舵機)
//#define Servo_Pin      3  // 宣告伺服馬達輸出腳位(PWM)
//#define servo_delay  250  // 伺服馬達轉向後的穩定時間
//Servo myservo;            // 宣告伺服馬達變數

void advance(int a)    // 小車前進
{
    digitalWrite(MotorA_I1,HIGH);   //馬達（右）順時針轉動
    digitalWrite(MotorA_I2,LOW);
    digitalWrite(MotorB_I3,HIGH);   //馬達（左）逆時針轉動
    digitalWrite(MotorB_I4,LOW);
    delay(a * 100);
}

void turnR(int d)    // 小車右轉
{
    digitalWrite(MotorA_I1,LOW);    //馬達（右）逆時針轉動
    digitalWrite(MotorA_I2,HIGH);
    digitalWrite(MotorB_I3,HIGH);   //馬達（左）逆時針轉動
    digitalWrite(MotorB_I4,LOW);
    delay(d * 100);
}

void turnL(int e)    // 小車左轉
{
    digitalWrite(MotorA_I1,HIGH);   //馬達（右）順時針轉動
    digitalWrite(MotorA_I2,LOW);
    digitalWrite(MotorB_I3,LOW);    //馬達（左）順時針轉動
    digitalWrite(MotorB_I4,HIGH);
    delay(e * 100);
}    

void stopRL(int f)  // 小車停止
{
    digitalWrite(MotorA_I1,HIGH);   //馬達（右）停止轉動
    digitalWrite(MotorA_I2,HIGH);
    digitalWrite(MotorB_I3,HIGH);   //馬達（左）停止轉動
    digitalWrite(MotorB_I4,HIGH);
    delay(f * 100);
}

void back(int g)    // 小車後退
{
    digitalWrite(MotorA_I1,LOW);    //馬達（右）逆時針轉動
    digitalWrite(MotorA_I2,HIGH);
    digitalWrite(MotorB_I3,LOW);    //馬達（左）順時針轉動
    digitalWrite(MotorB_I4,HIGH);
    delay(g * 100);     
}
    

void setup()
{
  Serial.begin(9600); 
  
  pinMode(MotorA_I1,OUTPUT);
  pinMode(MotorA_I2,OUTPUT);
  pinMode(MotorB_I3,OUTPUT);
  pinMode(MotorB_I4,OUTPUT);
  pinMode(MotorA_PWMA,OUTPUT);
  pinMode(MotorB_PWMB,OUTPUT);
  
  pinMode(isObstaclePin, INPUT); //Receive IR Obstacle interrupt Signal
  pinMode(lasertrig, OUTPUT);
  digitalWrite (lasertrig, LOW);

  pinMode(US_Trig, OUTPUT);
  pinMode(US_Echo, INPUT);
 
  pinMode(SensorLeft,   INPUT); 
  pinMode(SensorMiddle, INPUT);
  pinMode(SensorRight,  INPUT);
  
//  myservo.attach(Servo_Pin);
  
  irrecv.enableIRIn();  // 啟動 IR Controller Receiver 解碼讀取
  
  analogWrite(MotorA_PWMA,200);    //設定馬達 (右) 轉速
  analogWrite(MotorB_PWMB,200);    //設定馬達 (左) 轉速
  attachInterrupt (digitalPinToInterrupt (isObstaclePin), pinFalled, FALLING);  // attach interrupt handler
}


void loop()
{
     
   
    if(irrecv.decode(&results)) 
    {
        // 解碼成功，收到一組紅外線訊號
        Serial.print("\r\nIR Code: ");            
        Serial.print(results.value, HEX); 
        switch(results.value)
        {
          case IR_Forwards:
            Serial.print(" Forwards");
            advance(1);
            break;
            
          case IR_Back:
            Serial.print(" Back");
            back(1);
            break;
            
          case IR_Stop:
            Serial.print(" Stop");
            stopRL(1);
            break;
            
          case IR_Left:
            Serial.print(" Left");
            turnR(1);
            break;
            
          case IR_Right:
            Serial.print(" Right");
            turnL(1);
            break;
            
          case IR_Tracking: //Enter Line Tracking mode
            Serial.print(" Tracking Mode");
            irrecv.resume();
            ringTone2(buzzerPin);
            for(int c=0;c<2;c++){
            turnR(1);
            delay(1000);
            turnL(1);
            delay(1000);
            advance(1);
            delay(1000);
            turnR(1);
            delay(1000);
            turnL(1);
            delay(1000);
            back(1);
            delay(1000);
            }
            break;
            
            
            
            
            
            
             
            
             
              
                
            
          case IR_Ultrasonic:
            Serial.print("Begin Ultrasonic detector");
            
            irrecv.resume();
            
            while (true){
            if(sonar(2) < 15.0 || isObstacle)  // 超音波感測器偵測後得到的距離低於10.0公分或IR有訊號
              {
              stopRL(0); // Stop for a while  
              while (sonar(2) < 15.0 || digitalRead(isObstaclePin)==LOW){   //To see is the obatacles are still there?
                  laserfired(500);
                  ringTone2(buzzerPin); 
                  }    
              if (digitalRead(isObstaclePin)!=LOW) isObstacle =false;
              }
            else
              {
                if(laserOn)
                laserfired(500);
                advance(0);                // 正常前進 
              }
               
              if(irrecv.decode(&results))
              {
                irrecv.resume();
                
                if(results.value == IR_Stop)
                {
                  Serial.print("\r\nStop Ultrasonic detector");
                  stopRL(1);
                  break;
                }
              }
            }
            break;
            
          default:
          Serial.print(" Unsupported");
        }
    
        irrecv.resume(); // Receive the next value
    }
 }
