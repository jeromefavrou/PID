#include "PID.hpp"

//comment or uncomment for simulate or not
#define __SIM__

#define delta_time 500 //500 ms

uint8_t const Ai_size{2};

struct Pin{uint8_t const number;float value;};

struct Pin Ai[Ai_size]{{A0,.0},{A1,.0}};
struct Pin Do{3,LOW};


unsigned long Thread_inst{0};


template<uint8_t N> void update_Ai(struct Pin lst[N])
{
  for(auto i=0u;i<N;i++)
    lst[i].value=analogRead(lst[i].number);
}

void calibrate_termistor(struct Pin & lst)
{
    float tempK = log(10000.0 * ((1024.0 / lst.value - 1)));
    tempK = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * tempK * tempK )) * tempK );
    lst.value = tempK - 273.15;
}

void calibrate_potentiometre(struct Pin & lst)
{
  if(lst.value==0)
  {
    lst.value = 15.0;
    return;
  }
    
    float range = lst.value / 25.575f;
    lst.value = 15.0 + range; // reglage de 15.039 a 55 °c
}

template<int periode> void thread_regulator(unsigned long & time_inst, PID & _regulator ,float & do_value)
{
  if(millis()-time_inst>=periode)
  { 
    time_inst=millis();
    
    do_value =_regulator.corrector(-255,0)*-1.0;
  } 
}

PID Reg_Ai;
long int freq_reg{0};

void setup() 
{
    #ifdef __SIM__
    Serial.begin(19200);
    #endif
  
    Reg_Ai.lock_sensor(Ai[0].value);//test sur tmp0
    Reg_Ai.lock_order(Ai[1].value);
    Reg_Ai.parameter(float(delta_time)/1000.0,20,0.6,10);
    Reg_Ai.direction(-1);
  
    Thread_inst=millis();
}

#ifdef __SIM__
void simulate(float & _temp, float _pow)
{
  _temp+=(_pow-255.0/2.75)*(0.003)-(_temp/300.0);
}

void display_sim(float order,float _temp, float _pow)
{
  Serial.print(order);
  Serial.print(",");
  Serial.print(_temp);
  Serial.print(",");
  Serial.print((float(_pow)/255.0)*100.0);
  Serial.println("");
}
#endif

void loop() 
{
  
  #ifdef __SIM__
  simulate(Ai[0].value,Do.value);
  Ai[1].value=55;
  #else
  update_Ai<Ai_size>(Ai); //lecture des analoges
  calibrate_termistor(Ai[0]); //calibrage
  Ai[0].value += -0 ; //etalonage
  calibrate_potentiometre(Ai[1]); //calibrage
  #endif
  
  thread_regulator<delta_time>(Thread_inst, Reg_Ai,Do.value);
  
  analogWrite(Do.number,Do.value);
  
  #ifdef __SIM__
  display_sim(Ai[1].value,Ai[0].value,Do.value);
  #endif

  delay(50);
}
