/********************************************************
Lima - Perú MARZ/2019
AUTOR: Castillo Alvarado, David 
PROYECTO: Control de Motor DC - PID(Z)
Descripción:
  El presente proyecto es una implementación didactica sobre el uso
  del metodo de control de posición por PID en un motor DC, usando encoder incremental. 

CONEXIÓN::::::::::::::::::::::::::
> ENCODER: (5V) 100PPR
  360grados = 400 pulsos
  0.9 grados / Pulso
  For better signal, we used a NOT TTL7404, for every channel.
  Remember that the output signal is 3.3V for the TTL microchip, and that is good for the ESP8266.
> ESP8266: (5V USB) [Warning I/O 3.3v, doesn't matter if the USB is 5V]
  D1  = Channel A
  D2  = Channel B
  D8  = IN1 --L298N
  D7  = IN2 --L298N
  D6 = PWM --L298N [10bit resolution ESP8266]
> L298N (5v & 19v)
  IN1
  IN2
  In PWM 
FUENTES:::::::::::
  https://www.youtube.com/watch?v=c0L4gNKwjRw
  https://playground.arduino.cc/Code/PIDLibrary
***************************************************************************************************/
#include <PID_v1.h>
#include <Ticker.h>  //timers only for ESP8266
/*-------------------------------------Variables para Impresión en consola------------------------*/
byte         cmd       =  0;             // Use for serial comunication.  
byte         flags;                      // Flag for print values in the serial monitor
/*------------------------------Variebles for LM298N-------------------------------*/
int IN1  = D8; 
int IN2  = D7;
int PWM1 = D6;
int forWARDS  = 1; 
int backWARDS = 0;
float start   = 0;

/*------------------------------Variables for incremental encoder----------------------------------*/
volatile long contador =  0;   
byte          ant      =  0;    
byte          act      =  0;
const byte    encA     =  D1;                  // Signal for channel A
const byte    encB     =  D2;                  // Signal for channel B
int   MIN_MAX_POST     =  300;                 // Limit the maximun position
/*-----------------------------We defined variables for PID algorithm------------------------------*/
Ticker PID_compute;                         // Timer
double Setpoint, Input, Output;
double SampleTime = 1;                      // time in mili seconds, RUN at 160MHZ ESP8266
//double Kp=5, Ki=2, Kd=0.001; 
//double Kp=5.5, Ki=3.6, Kd=0.002;
double Kp=5.5, Ki=4.0, Kd=0.002;              // PID gain
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

/*----------------------------- Interruption Function----------------------------------------------*/ 
void PID_DCmotor_interrup(){
  //Serial.print("t: ");Serial.println(millis()-start)   ;
  //start = millis();
  myPID.Compute();  // Calculus for PID algorithm 
  RunMotor(Output); // PWM order to DC driver
}

/*----------------------------SETUP----------------------------------------------------------------*/
void setup(){ 
  Serial.begin(115200); 
  //Iniciando L298N
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(PWM1, LOW);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM1, OUTPUT);
  
  Setpoint = 0.0; // Init in position 0.0 
  //RUN the PID
  myPID.SetMode(AUTOMATIC);
  // Max Min values for PID algorithm
  myPID.SetOutputLimits(-1023,1023); 
  // Sample Time for PID
  myPID.SetSampleTime(SampleTime);     

  // Initializing Interruptions
  PID_compute.attach_ms(SampleTime, PID_DCmotor_interrup); 

  // Pin Interruption
  attachInterrupt(digitalPinToInterrupt(encA), encoder, CHANGE); // rising and falling flank
  attachInterrupt(digitalPinToInterrupt(encB), encoder, CHANGE); // rising and falling flank
}

/*----------------------------LOOP----------------------------------------------------------------*/
void loop(){  
  Serial.print("PWM  :");Serial.print(Output); 
  Serial.print(" |  contador  :");Serial.print(contador);
  Serial.print(" |  Setpoint  :");Serial.println(Setpoint);
  // Protection code
  //limit_post();
  // Ask for input data for change PID gain or setpoint
  input_data();
}
/*------------------------------------------------------------------------------------------------*/

// Function for run the motor, backward, forward or stop
void RunMotor(double Usignal){  
  if (Setpoint-Input==0){
    shaftrev(IN1,IN2,PWM1,backWARDS, 0);
    //Serial.print("cero");
  }else if(Usignal>=0){
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

// Data catched from serial terminal
void input_data(void){
  if (Serial.available() > 0){           // Check if you have received any data through the serial terminal.
  
      cmd = 0;                            // clean CMD
      cmd = Serial.read();                // "cmd" keep the recived byte
      if (cmd > 31){
      
        flags = 0;                                           // Reboot the flag, that decide what have to be printed
        if (cmd >  'Z') cmd -= 32;                           // Change to Uppercase 
        if (cmd == 'W') { Setpoint += 5.0;     flags = 2; }  // For example is you put 'W' moves 5 steps forward. Relative movement
        if (cmd == 'Q') { Setpoint -= 5.0;     flags = 2; }  // Here is 5 steps backward
        if (cmd == 'S') { Setpoint += 400.0;   flags = 2; }  // The same with another values 
        if (cmd == 'A') { Setpoint -= 400.0;   flags = 2; }
        if (cmd == 'X') { Setpoint += 5000.0;  flags = 2; }
        if (cmd == 'Z') { Setpoint -= 5000.0;  flags = 2; }
        if (cmd == '2') { Setpoint += 12000.0; flags = 2; }
        if (cmd == '1') { Setpoint -= 12000.0; flags = 2; }
        if (cmd == '0') { Setpoint = 0.0;      flags = 2; }  // Ir a Inicio.
        
        // Decode for change the PID gains
        switch(cmd)                                                     // for example, we put "P2.5 I0.5 D40" them the gains will take these values kp, ki y kd.
        {                                                               // You can change every gain independently
          case 'P': Kp  = Serial.parseFloat();        flags = 1; break; // Change the PID gains
          case 'I': Ki  = Serial.parseFloat();        flags = 1; break;
          case 'D': Kd  = Serial.parseFloat();        flags = 1; break;
          case 'G': Setpoint   = -1*Serial.parseFloat(); flags = 2; break; // You can change the setpoint with absolute values Ex: G23000
          case 'K':                                   flags = 3; break;
        }       
        imprimir(flags);
      }
    }
}

// Print date in serial terminal
void imprimir(byte flag){ 

  if ((flag == 1) || (flag == 3))
  {
    Serial.print("KP=");     Serial.print(Kp);
    Serial.print(" KI=");    Serial.print(Ki);
    Serial.print(" KD=");    Serial.print(Kd);
    Serial.print(" Time=");  Serial.println(SampleTime);
  }
  if ((flag == 2) || (flag == 3))
  {
    Serial.print("Position:");
    Serial.println((long)Setpoint);
  }
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

  // Enter the counter as input for PID algorith
  Input=contador;
}

void limit_post(void){
  // Protection code for limit the position
  if(contador>=MIN_MAX_POST){
     RunMotor(0);
  }else if(contador<=-MIN_MAX_POST){
     RunMotor(0);
  }
}
