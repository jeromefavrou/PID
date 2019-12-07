#ifndef PID_HPP
#define PID_HPP

#ifndef nullptr
  #define nullptr 0x00
#endif

#ifndef PID_version
  #define PID_version 0x04
#endif

class PID
{
  public:
  
  PID(void):m_sensor(nullptr),m_order(m_sensor),m_Kp(1.0),m_Ki(.0),m_Kd(.0),m_dir(1),m_el(.0),m_ei(.0),m_dt(1.0),m_t(.0),m_delay(.0)
  {
    
  }

  void lock_sensor(float & _sensor)
  {
    this->m_sensor=&_sensor;
  }

  void lock_order(float & _order)
  {
    this->m_order=&_order;
  }

  void direction(short _dir)
  {
    this->m_dir= _dir>=1?1:_dir<=-1?-1:1;  /// define the direction of system (direct or inverse action)
  }
  
  void delay(float _delay)
  {
    this->m_delay=_delay; /// define the delay of system (may be > 0 )
  }
 
  void parameter(float _dt,float _Kp,float _Ki=.0f, float _Kd=.0f)
  {
    this->m_dt=_dt; /// set delta of time between 2 corrections /// this value may be stable (in second)
    this->m_Kp=_Kp; /// set proportionnal correction ( gain ) /// if < 0 this parametre reverse direction so may be >0
    this->m_Ki=_Ki; /// set intergrate correction /// this value correcte a static error 
    this->m_Kd=_Kd; /// set derivate correction /// this value correcte all pertubation
  }

  ///return a corrector value between min and max values
  ///min and max can be direcly order a pwm pin if min=+-0 and max +-255 for 8 byte
  float corrector(float _min,float _max)
  {
    float _error=*m_order-*m_sensor; //differential error
    
    this->m_t+=this->m_dt; // timeout
    
    float a=(this->m_el-_error)/this->m_dt; //calculate derivate also last correction  f'(t)= F(at +b) =a
    
    float b=*m_sensor-a*this->m_t; //calculate b in f(t)=at+b
    
    //F(f(t))=at*t/2+ bt + k
    //F(order)=order*t + k
    // f(t) dt = F(y)-F(x) = a/2(y*y-x*x) + b(y-x);
    // order dt= F(y)-F(x) = order(y-x);
    //I x to y = f(t) dt - order dt
    
    float x=this->m_t-this->m_dt;
    this->m_ei+=(*m_order*this->m_dt)-((a/2.0)*(this->square(this->m_t)-this->square(x))+b*(this->m_t-x));//intagrate error
    
    //this->m_ei=this->m_ei>_max?_max:this->m_ei<_min?_min:this->m_ei; //limitless of intergate part
    
    float _correct=(m_Kp*_error + m_Ki*this->m_ei + m_Kd * a - this->m_delay)*float(m_dir) ;
    
    this->m_el=_error;
    
    return _correct<_min?_min:_correct>_max?_max:_correct;
  }

  ~PID(void)
  {
    this->m_sensor=nullptr;
    this->m_order=nullptr;
  }

  private:
  
  inline float square(float x)
  {
    return x*x;
  }
  
  float * m_sensor;
  float * m_order;

  float m_Kp,m_Ki,m_Kd,m_dt,m_t, m_el,m_ei, m_delay;
  short m_dir;  
};

#endif
