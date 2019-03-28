// AUTOR: David Castillo Alvarado
// Fecha: 2/12/2019
// TEMA: Control PID
// TEMA_vídeo : 
#include <Ticker.h>  //timers only for ESP8266
Ticker send_ident;  
int SampleTime = 3;  
String data_send; 
/*------------------------------INput/output values--------------------------------*/
float signal = 0.0; // Input
#define N  (2)
#define M  (8)
int list_signal[M]={200,300,400,500,600,900,800,1000};
int list_delay[N] ={1000,1000};
/*------------------------------Variables for LM298N-------------------------------*/
int IN1 = D8; 
int IN2 = D7;
int PWM1 = D6;
int forWARDS  = 1; 
int backWARDS = 0;

/*------------------------------Variables for incremental encoder----------------------------------*/
volatile long contador =  0;                    // Output
byte          ant      =  0;    
byte          act      =  0;
const byte    encA     =  D1;                  // Signal for channel A
const byte    encB     =  D2;                  // Signal for channel B

/*==============================SETUP=================================*/
void setup() { 
  
  Serial.begin(115200);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(PWM1, LOW);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM1, OUTPUT);
  //------Timer----
  send_ident.attach_ms(SampleTime, DC_send_data); 
  // Pin Interruption
  attachInterrupt(digitalPinToInterrupt(encA), encoder, CHANGE); // rising and falling flank
  attachInterrupt(digitalPinToInterrupt(encB), encoder, CHANGE); // rising and falling flank
}
/*==============================LOOP==================================*/
void loop() 
{/*
  for (int i=0; i<10;i++){
    contador = 0;
    RunMotor(list_signal[i]);
    signal = list_signal[i];
    delay(list_delay[0]);
    signal = 0;
    RunMotor(-list_signal[i]);//frena
    delay(1);
    RunMotor(signal);         //cero pwm
    delay(list_delay[1]);;    //espera */
  for (int i=0; i<M;i++){
    RunMotor(list_signal[i]);
    signal = list_signal[i];
    delay(list_delay[0]);
    //signal = 0;
    RunMotor(-list_signal[i]);
    signal = -list_signal[i];
    delay(list_delay[1]);
    //RunMotor(signal);         //cero pwm
    //delay(list_delay[1]);;    //espera

  }

}

/*====================================================================*/

/*=============================ROLL====================================*/
void DC_send_data(){
  data_send = String("{")+String("'input':") + String(signal) + String(",") 
                         +String("'output':") + String(contador) + String("}");
  
  Serial.println(data_send);
  //Serial.println(dt);
}

// Encoder x4. Execute when interruption pin jumps.
void encoder(void){ 
  //Serial.println(ant);
  ant=act;                            // Saved act (current read) in ant (last read)
  act = digitalRead(encA)<<1|digitalRead(encB);
  if(ant==0 && act==1)        contador++;  // Increase the counter for forward movement
  else if(ant==1  && act==3)  contador++;
  else if(ant==3  && act==2)  contador++;
  else if(ant==2  && act==0)  contador++;
  else contador--;                         // Reduce the counter for backward movement
}


// Function for run the motor, backward, forward or stop
void RunMotor(double Usignal){  
  if (Usignal>=0){
    shaftrev(IN1,IN2,PWM1,backWARDS, Usignal);
  }else{
      shaftrev(IN1,IN2,PWM1,forWARDS, -1*Usignal);
  }   
}

// Function that set DC_driver to move the motor
void shaftrev(int in1, int in2, int PWM, int sentido,int Wpulse){  
  if(sentido == 0){ //backWARDS
    digitalWrite(in2, HIGH);
    digitalWrite(in1, LOW);
    analogWrite(PWM,Wpulse);
    }
  if(sentido == 1){ //forWARDS
    digitalWrite(in2, LOW);
    digitalWrite(in1, HIGH);
    analogWrite(PWM,Wpulse);     
    }
}