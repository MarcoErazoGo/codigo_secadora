#include<TimerOne.h>
#include <RTClib.h>   
#include <LiquidCrystal_I2C.h> 
#include <PID_v1.h>

#define PIN_INPUT A2
#define SSR A3
#define tiempociclo 100

double Setpoint, Input, Output;
double Kp=0.078735, Ki=0.014271, Kd = 0.05;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

unsigned long RespuestaUltimaTemperatura;
unsigned long lastPIDcalculation = 0;

RTC_DS3231 reloj; 
unsigned long time;

LiquidCrystal_I2C lcd(0x27, 16,2 );

int tiempoactual,tiemposalida,tiemposecado,tiempollegada;


int play=0;
int star=4,bot_ON=0;
int botonA=9,botonB=10, botonC=11, botonmin=0, botonminant=0, botonstart=0,botonstartant=0;
int cantmin=0,eleccion;
int estado=0;
int cambio=0;
int botonnuevo;
int botonanterior=1;
int estadoanterior=0;
int pausa =0, pulsacion=0,tiempo_pulsacion=0,estado_pausa;

int etapa=1;
float temperatura;
int sensor;

int sensorIRentrada= 12;
int sensorIRsalida= 13;

//SENSORES DE TEMPERATURA
float lectura1,dato1,temperatura1;
float lectura2,dato2,temperatura2;

//VENTILADORES
int Direcion_ventiladores_D = 8;    
int Direcion_ventiladores_I = 7;    
int velocidad_ventilacion=5;

//TRANSPORTE
int Direccion_transporte_D = 2;
int Direccion_transporte_I = 3;
int velocidad_transporte = 6; // ENB conectada al pin 3 de Arduino

//RESISTENCIA
int ge=0,pot, disparo=3, grado_dimmer=0;
int potencia_secado=0;

int tempSecado,veltransporte,velsecado,tiempoproceso=0,conteo=0,referencia=0,a=1,  b=2,c=5,ele=0,entra=0,botonauto,botonautoant,estadoauto;
int entrada=0,salida=0;

void setup() {
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();

  Setpoint = 390;
  myPID.SetOutputLimits(0, tiempociclo);
  myPID.SetSampleTime(tiempociclo);
  myPID.SetMode(AUTOMATIC);
  pinMode(SSR,OUTPUT);
  digitalWrite(SSR,LOW);

  pinMode(star,INPUT);
  pinMode(botonA,INPUT);
  pinMode(botonB,INPUT);
  pinMode(botonC,INPUT);
 
  pinMode(sensorIRentrada,INPUT);
  pinMode(sensorIRsalida,INPUT);

  pinMode (Direccion_transporte_D, OUTPUT);
  pinMode (Direccion_transporte_I, OUTPUT);
  pinMode (velocidad_transporte, OUTPUT); 
  
  pinMode (Direcion_ventiladores_D, OUTPUT);
  pinMode (Direcion_ventiladores_I, OUTPUT);
  pinMode(velocidad_ventilacion, OUTPUT);
  
  if (! reloj.begin()) {       // si falla la inicializacion del modulo
  delay(1000);
  while (1);         // bucle infinito que detiene ejecucion del programa
  }
  reloj.adjust(DateTime(__DATE__,__TIME__)); 
  
}

void loop() {
  
  digitalWrite(SSR,LOW);
  bot_ON= digitalRead(star);

  if(bot_ON==1){
    
     DateTime proceso = reloj.now();  
     tiemposecado=proceso.second();
     sensores();
     if(temperatura1 <=65  || temperatura2 <=65)
     {
      switch(etapa){
        case 1:
       
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("BIENVENIDO");
        delay(500);
        etapa=2;
         
        break;
        
        case 2:
        
        digitalWrite(SSR,LOW);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("seleccione la");
        lcd.setCursor(0,1);
        lcd.print("vel. de secado");
        delay(400);  
        
        while(cantmin == 0){
          botones();
        }
        eleccion=cantmin;
        cantmin=0;
        etapa=3;

        break;

        case 3:

        lcd.clear();
        lcd.setCursor(2,0);
        lcd.print("elijio ");
        lcd.setCursor(10,0);
        lcd.print(eleccion);
        lcd.setCursor(13,0);
        lcd.print("MIN");
        lcd.setCursor(0,1);
        lcd.print("vel. de secado");
        delay(400); 

        etapa=4;

        break;

        case 4:
      
        if(estadoauto==1){
          etapa=5;
        }
        else if(estadoauto==0)
        {
            lcd.clear();
            lcd.setCursor(1,0);
            lcd.print("presione start");
            lcd.setCursor(0,1);
            lcd.print("para secar");
            delay(400); 
            play=0;
            while(play==0){
            play=digitalRead(botonB);
          }
          etapa=5;
          
        }

        break;

        case 5:
        
        if(tiemposecado==0){
        DateTime proceso = reloj.now();  
        tiemposecado=proceso.second();
        }

        if(eleccion==1){
          veltransporte=230;
          tiemposecado=proceso.second()+10;
          referencia=proceso.second();
          Setpoint = 390;
          velsecado=255;
        }
        else if(eleccion==2){
          veltransporte=180;
          referencia=proceso.second();
          tiemposecado=proceso.second()+20;
          Setpoint = 300;
          velsecado=180; 
        }
        else if(eleccion==5){
          veltransporte=150;
          referencia=proceso.second();
          tiemposecado=proceso.second()+50;
          Setpoint = 260;
          velsecado=100; 
        }

        while(referencia < tiemposecado){
          
          DateTime proceso = reloj.now();
          tiempoproceso =proceso.second();

          if(tiempoproceso > conteo){
            conteo=tiempoproceso;
            referencia=referencia+1;
          }
          boton_pausa();
          if(estado_pausa==0){
           secado(); 
          }
          else{
               if(tiempoproceso > conteo)
               {
                    conteo=tiempoproceso;
                    tiemposecado=tiemposecado+1;
                }  
          }
          
          Serial.print(referencia);
          Serial.println("referencia");
          Serial.print(tiempoproceso);
          Serial.println("tiempoproceso");
          Serial.print(tiemposecado);
          Serial.println("tiemposecado");
          Serial.println("esta dentro de while");
         
        }
       if(digitalRead(sensorIRsalida)==0)
       {
         Serial.println("objeto detectado");
         digitalWrite (Direcion_ventiladores_D, HIGH);
         digitalWrite (Direcion_ventiladores_I, LOW);
         digitalWrite (Direccion_transporte_D, HIGH);
         digitalWrite (Direccion_transporte_I, LOW);    
         analogWrite(velocidad_ventilacion,velsecado);
         analogWrite(velocidad_transporte,veltransporte);
         
        if (millis() - RespuestaUltimaTemperatura >= tiempociclo)
        {
        Input = analogRead(PIN_INPUT)*0.0048828125 *100;
        myPID.Compute();
        lastPIDcalculation = millis();
        RespuestaUltimaTemperatura = millis();
        }
        control();
        
         lcd.clear();
         lcd.setCursor(2,0);
         lcd.print("secando ");
         lcd.setCursor(3,1);
         lcd.print("radiografia");
         delay(500); 
        }
       else{
         Serial.println("objeto detectado");
         digitalWrite (Direcion_ventiladores_D, LOW);
         digitalWrite (Direcion_ventiladores_I, LOW);
         digitalWrite (Direccion_transporte_D, LOW);
         digitalWrite (Direccion_transporte_I, LOW);    
         analogWrite(velocidad_ventilacion,0);
         analogWrite(velocidad_transporte,0);

         conteo=0;
         tiemposecado=0;
         referencia=0; 
         digitalWrite(SSR,LOW);
         lcd.clear();
         lcd.setCursor(2,0);
         lcd.print("Inserte nueva ");
         lcd.setCursor(3,1);
         lcd.print("radiografia");
         delay(500); 
         etapa=4;
         }
      break;

      }
     }
   else
   {         
    
             digitalWrite(SSR,LOW);
             lcd.clear();
             lcd.setCursor(2,0);
             lcd.print("PELIGRO");
             lcd.setCursor(3,1);
             lcd.print("SOBRECALENTAMIENTO");
             delay(400); 
             etapa=2;
             conteo=0;
             tiemposecado=0;
             referencia=0; 
             
             if(digitalRead(sensorIRentrada)!=0  || digitalRead(sensorIRsalida)!=0)
             {
                  digitalWrite (Direcion_ventiladores_D, HIGH);
                  digitalWrite (Direcion_ventiladores_I, LOW);
                  digitalWrite (Direccion_transporte_D, HIGH);
                  digitalWrite (Direccion_transporte_I, LOW);
                  analogWrite(velocidad_transporte,230);
                  analogWrite(velocidad_ventilacion,255);   
                  lcd.clear();
                  lcd.setCursor(4,0);
                  lcd.print("Evacuando");
                  lcd.setCursor(3,1);
                  lcd.print("Radiografia");
                  delay(400); 
             }
   }
  }
}

void botones(){
  
     //lectura botones  
     botonmin = digitalRead(botonA);
     botonstart=digitalRead(botonB);
     botonauto = digitalRead(botonC);
  
     if(botonmin == 0 && botonminant== 1){

        entra++;
        if(entra == 1)
        {
           ele=a;
        }
        else if(entra ==2)
        {
           ele=b;
        }
       else if(entra ==3)
        {
           ele=c;
        }
      if(entra>=3){
          entra=0;
        }
       lcd.clear();
       lcd.setCursor(2,0);
       lcd.print("VELOCIDAD DE");
       lcd.setCursor(0,1);
       lcd.print("SECADO ");
       lcd.setCursor(7,1);
       lcd.print(ele);
       lcd.setCursor(9,1);
       lcd.print("MIN.");
       delay(400); 
       }
       botonminant=botonmin;
       
      if(botonauto == 0 && botonautoant == 1)
      {
         if(estadoauto==0)
         {
            estadoauto=1;
            digitalWrite(10,HIGH);
         }
         else
         {
            estadoauto=0;
            digitalWrite(10,LOW);
         }
       }
       botonautoant=botonauto;


       if(botonstart == 0 && botonstartant == 1)
       {
           cantmin=ele;
       }
       botonstartant=botonstart;
}

void secado(){

  entrada = digitalRead(sensorIRentrada);
  salida = digitalRead(sensorIRsalida);
  Serial.print(entrada);
  Serial.println("lectura sensor de entrada");
  Serial.print(salida);
  Serial.println("lectura sensor de salida");

  if(digitalRead(sensorIRentrada)==0 && digitalRead(sensorIRsalida)==0){
            
            digitalWrite (Direcion_ventiladores_D, LOW);
            digitalWrite (Direcion_ventiladores_I, LOW);
            digitalWrite (Direccion_transporte_D, LOW);
            digitalWrite (Direccion_transporte_I, LOW);
            analogWrite(velocidad_transporte,0);
            analogWrite(velocidad_ventilacion,0);
            digitalWrite(SSR,LOW);
            
            lcd.clear();
            lcd.setCursor(3,0);
            lcd.print("ingrese la ");
            lcd.setCursor(3,1);
            lcd.print("radiografia");
            delay(500); 

          }
   else if(digitalRead(sensorIRentrada)!=0 && digitalRead(sensorIRsalida)== 0 )
  {
    if (millis() - RespuestaUltimaTemperatura >= tiempociclo)
    {
    Input = analogRead(PIN_INPUT)*0.0048828125 *100;
    myPID.Compute();
    lastPIDcalculation = millis();
    RespuestaUltimaTemperatura = millis();
    }
    control();

    
    digitalWrite (Direcion_ventiladores_D, HIGH);
    digitalWrite (Direcion_ventiladores_I, LOW);
    digitalWrite (Direccion_transporte_D, HIGH);
    digitalWrite (Direccion_transporte_I, LOW);
    analogWrite(velocidad_transporte,veltransporte);
    analogWrite(velocidad_ventilacion,0);
    
 
    lcd.clear();
    lcd.setCursor(3,0);
    lcd.print("ingresando la");
    lcd.setCursor(3,1);
    lcd.print("radiografia");
    delay(500); 
    analogWrite(velocidad_ventilacion,velsecado);
    lcd.clear();
    lcd.setCursor(2,0);
    lcd.print("posicionando ");
    lcd.setCursor(3,1);
    lcd.print("radiografia");
    delay(500); 
  }
  else if(digitalRead(sensorIRentrada)==0 && digitalRead(sensorIRsalida)!=0)
  {
    if (millis() - RespuestaUltimaTemperatura >= tiempociclo)
    {
    Input = analogRead(PIN_INPUT)*0.0048828125 *100;
    myPID.Compute();
    lastPIDcalculation = millis();
    RespuestaUltimaTemperatura = millis();
    }
    control();

    digitalWrite (Direcion_ventiladores_D, HIGH);
    digitalWrite (Direcion_ventiladores_I, LOW);
    digitalWrite (Direccion_transporte_D, LOW);
    digitalWrite (Direccion_transporte_I, LOW);
    analogWrite(velocidad_transporte,0);
    analogWrite(velocidad_ventilacion,velsecado);
    lcd.clear();
    lcd.setCursor(2,0);
    lcd.print("secando ");
    lcd.setCursor(3,1);
    lcd.print("radiografia");
    delay(500); 
  }
  else if(digitalRead(sensorIRentrada)!=0 && digitalRead(sensorIRsalida)!=0)
  {
    digitalWrite (Direcion_ventiladores_D, LOW);
    digitalWrite (Direcion_ventiladores_I, LOW);
    digitalWrite (Direccion_transporte_D, LOW);
    digitalWrite (Direccion_transporte_I, LOW);
    analogWrite(velocidad_transporte,0);
    analogWrite(velocidad_ventilacion,0);
    digitalWrite(SSR,LOW);
    lcd.clear();
    lcd.setCursor(5,0);
    lcd.print("error");
    delay(500); 
  }

}

void cruce_cero(){

 ge=0;
}

void boton_pausa(){
  DateTime pausa_cancelacion = reloj.now(); 
  
  botonstart=digitalRead(botonB);
  if(botonstart == 0 && botonstartant == 1)
  {
    pulsacion++;
    
    DateTime pausa_cancelacion = reloj.now(); 
    tiempo_pulsacion =pausa_cancelacion.second();
    if(tiempo_pulsacion<=referencia){
      if(pulsacion==3){
        etapa = 2;
        conteo=0;
        tiemposecado=0;
        referencia=0; 
        lcd.clear();
        lcd.setCursor(1,0);
        lcd.print("Procesamiento");
        lcd.setCursor(4,1);
        lcd.print("cancelado");
        delay(400);
      }
    }
    else{
      pulsacion=1;
    }
    if(pulsacion == 1){
     DateTime pausa_cancelacion = reloj.now(); 
     referencia =pausa_cancelacion.second()+3;
     if(estado_pausa==0){
        digitalWrite (Direcion_ventiladores_D, LOW);
        digitalWrite (Direcion_ventiladores_I, LOW);
        digitalWrite (Direccion_transporte_D, LOW);
        digitalWrite (Direccion_transporte_I, LOW);
        analogWrite(velocidad_transporte,0);
        analogWrite(velocidad_ventilacion,0);
        grado_dimmer=0;
        estado_pausa=1;
        lcd.clear();
        lcd.setCursor(3,0);
        lcd.print("Proceso de");
        lcd.setCursor(1,1);
        lcd.print("Secado Pausado");
        delay(400);
     }
     else{
        estado_pausa=0;
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Proceso");
        lcd.setCursor(0,1);
        lcd.print("en funcion");
        delay(400);
     }
    }

  }
  botonstartant=botonstart;
}
void sensores() {

  lectura1= analogRead(A0);
  dato1 = ((lectura1*5)/1024);
  temperatura1=((dato1*80)/5);
  lectura2= analogRead(A1);
  dato2 = ((lectura2*5)/1024);
  temperatura2=((dato2*80)/5);
  Serial.print("lectura sensor 1:         ");
  Serial.println(temperatura1);
  Serial.print("lectura sensor 2:         ");
  Serial.println(temperatura2);
  delay(50);

}
void control()
{
  if(millis() <= (lastPIDcalculation + Output) || (Output == tiempociclo))
  {
    digitalWrite(SSR,HIGH);
  }
  else
  {
    digitalWrite(SSR,LOW);  
  }
}
 
