#include "PID.hpp"

#define Delta_time int(5)

uint8_t const Ai_size{2};

struct Pin{uint8_t const number;float value;};

struct Pin Ai[Ai_size]{{A0,.0f},{A1,.0f}};
struct Pin Do{3,LOW};

PID Reg_Ai;
long int freq_reg{0};

unsigned long Thread_inst{0};

void setup() 
{
    Reg_Ai.lock_sensor(Ai[0].value);//test sur tmp0
    Reg_Ai.lock_order(Ai[1].value);
    Reg_Ai.parameter(Delta_time/1000.0,10,0.05,1);
    Reg_Ai.direction(-1);
  
    Thread_inst=millis();
}

void loop() 
{
  update_Ai<Ai_size>(Ai); //lecture des analoges
  
  calibrate_termistor(Ai[0]); //calibrage
  calibrate_potentiometre(Ai[1]); //calibrage
  
  thread_regulator<Delta_time>(Thread_inst, Reg_Ai,Do.value);
  
  analogWrite(Do.number,Do.value);

  delay(1);
}


///////////////////////////////
//////FUNCTION/////////////////
///////////////////////////////

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
    
    do_value =_regulator.corrector(-255,0)*-1.0; /// for direcly use do_value in analogWrite()
  } 
}
