//调包
#include <SPI.h>
#include <ArduPID.h>

//PID
ArduPID myController;

//设置变量
double setpoint = 100;
double input;
double output;
double p = 1;
double i = 0;
double d = 0;

uint8_t FS=8;
uint8_t LOAD=7;
float value=5 ;
float clockspdMhz=8;

const int OutPin1a = 3;
const int OutPin1b = 6;
const int OutPin2a = 5;
const int OutPin2b = 9;
int time_delay;

int T=16;
int PostPone=4;
int vol[16]={0, 97, 180, 235, 255, 235, 180, 97, 0, -97, -180, -235, -255, -235, -180, -97};

void stepab(int OutPina,int OutPinb,int outvalue){  //outvalue range -256~+256
  if(outvalue>0){
    analogWrite(OutPina, outvalue);
    analogWrite(OutPinb, 0);
  }
  else{
    analogWrite(OutPina, 0);
    analogWrite(OutPinb, abs(outvalue));
  }
}

//控制电机的函数
void n_steps(int n_s){
  for(int i=0;i<abs(n_s)*4;i++){
    
//    n_s=Serial.parseInt();
//    if(n_s==888)break;
    
    //time_delay=analogRead(speed_c);
    int postpone,n,n2;
    time_delay=1;
    if(n_s>0){
      postpone=T-PostPone;
    }
    else{
      postpone=PostPone;
    }
    n=i%T;
    n2=(i+postpone)%T;
    stepab(OutPin1a,OutPin1b,vol[n]);
    stepab(OutPin2a,OutPin2b,vol[n2]);
    delay(time_delay);
  }
}

//uint16_t convert(int chapter,float value){
//  uint16_t value16,cha16;
//  value16=(uint16_t)(value*4095/5);
//  cha16=((uint16_t)chapter)<<12;
//  return cha16|value16;
//}

//模拟输出的函数
void write_data(uint8_t cmd, uint16_t value ){
  //digitalWrite( LOAD, 0);  
  digitalWrite( FS, 0 );
  delayMicroseconds( 1 );
  SPI.transfer( ((value & 0x0F00)>>8) | cmd<<4  );
  SPI.transfer(   value & 0xFF );
  delayMicroseconds( 1 );
  digitalWrite( FS,  1 );
  //digitalWrite( LOAD, 0 );
};

//系统初始化
void setup() {
  
    pinMode(FS,OUTPUT);
    digitalWrite(FS,  1);
    pinMode(LOAD,OUTPUT);
    digitalWrite( LOAD,  0 );
    pinMode(OutPin1a, OUTPUT);
    pinMode(OutPin1b, OUTPUT);
    pinMode(OutPin2a, OUTPUT);
    pinMode(OutPin2b, OUTPUT);
    
    Serial.begin(9600);
    
    SPI.begin();
    SPI.beginTransaction(
      SPISettings(clockspdMhz * 1000000 , MSBFIRST, SPI_MODE2)); //速度 重要的排前面 上升或下降沿

    myController.begin(&input, &output, &setpoint, p, i, d);

    write_data(8,0);

}

//主循环
void loop() {
  
  int n,n2,n_s;
  int i,st;
  int postpone,voltage;
  int delaytime;
   
  i=0;

  //无指令时等待输入
  while(1){
    n_s=Serial.parseInt();
    if(n_s!=0)break;
  }

  //电机控制针尖一直下降直至出现隧穿电流的模式，触发代号999
  if(n_s==999){
    Serial.println("start feed ");
    while(1){

//      n_s=Serial.parseInt();
//      if(n_s==888)break;
      
      voltage=analogRead(A0);
      Serial.println(voltage);
      if(voltage>50){
        break;
      }
      else{
        n_steps(-3);
      } 
    }
  }
  //控制压电陶瓷使电流稳定的模式，触发代号111
  else if(n_s==111){
    while(1){
      input = (double)analogRead(A0);
      myController.compute();
      digitalWrite( LOAD,  1 );
      write_data(3,(uint16_t)output);
      digitalWrite( LOAD,  0 );
      Serial.println(output);
    }
  }
  //电机按照输入控制针尖上下移动  
  else{
    Serial.print("a ");
    n_steps(n_s);
    stepab(OutPin1a,OutPin1b,0);
    stepab(OutPin2a,OutPin2b,0);
    Serial.print("step:");
    Serial.println(n_s); 
  }

}
