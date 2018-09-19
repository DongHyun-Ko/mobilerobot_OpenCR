
String motor1_pos_str;

String motor2_pos_str; 


 

String motor1_vel_str; 

String motor2_vel_str; 


 

long motor1_vel; 

long motor2_vel; 


char motor1_dir = '+'; 

char motor2_dir = '+'; 

 

unsigned long motor1 = 5000000; 

unsigned long motor2 = 5000000; 

 

double motor1_meter = 22.12;; 

double motor2_meter = 22.12;; 

 

unsigned long pre_motor1; 
unsigned long pre_motor2; 

 

bool pre_relative_dir; 

bool relative_dir = true; 

 

double pre_relative_pos; 

double relative_pos = 0; 


double relative_pos_motor1 = 0; 

double relative_pos_motor2 = 0; 


int32_t relative_pul_motor1 = 0; 

int32_t relative_pul_motor2 = 0; 
 

char tx_pos_buffer[1024]; 

char tx_vel_buffer[1024]; 

 
int32_t pre_relative_pul;

int32_t relative_pul = 0; 

  

void setup() { 

  

  Serial.begin(115200); 

   

} 

  

void loop() { 


  pre_motor1 = motor1; //이전 모터 위치 설정

  pre_motor2 = motor2; 

   
 String S = "QPFFFFFFFF,FFFFFFFF";
 String SV = "QV-1000000,+10000000";
 
  motor_position_data(S); //모터 위치 받아오기

   

  motor1 = hexatodecimal(motor1_pos_str); // 모터 위치 10진수로 변환

  motor2 = hexatodecimal(motor2_pos_str); 

 //////////////// 1024*4*26

  motor1_meter = pulse_to_meter(motor1); // 펄스 m 단위 변환

  motor2_meter = pulse_to_meter(motor2); 

   

  relative_pul_motor1 = relative_position_pul(motor1, pre_motor1); // 모터 위치 이동거리로 변환 

  relative_pul_motor2 = relative_position_pul(motor2, pre_motor2); 

  

 

  motor_velocity_data(SV); // 모터 속도 받아오기

  motor1_vel = Str_to_Long(motor1_vel_str, motor1_dir); // 모터 속도 str 에서 long으로 변환 

  motor2_vel = Str_to_Long(motor2_vel_str, motor2_dir); 

 

  double w = angle_vel(motor1_vel,motor2_vel); //각도 받아오기.

 

 Serial.print("1 :      "); //////

 Serial.print(relative_pul_motor1); /////

 Serial.print("      "); ////// 
 
 Serial.print("2 :      "); //////

 Serial.println(relative_pul_motor2); /////

 

   

 /* 

  Serial.print("motor1_ pos :   "); 

  Serial.print(motor1); 

  Serial.print("        "); 

  Serial.print("motor2_pos :    "); 

  Serial.println(motor2); 

   

  Serial.print("relative_motor1_ pos :   "); 

  Serial.print(relative_pos_motor1); 

  Serial.print("        "); 

  Serial.print("relative_motor2_pos :    "); 

  Serial.println(relative_pos_motor2); 

   

  Serial.print("motor1_ vel :   "); 

  Serial.print(motor1_vel_str); 

  Serial.print("        "); 

  Serial.print("motor1_ dir :   "); 

  Serial.print(motor1_dir); 

  Serial.print("        "); 

  Serial.print("motor2_vel :    "); 

  Serial.print(motor2_vel_str); 

  Serial.print("        "); 

  Serial.print("motor1_ dir :   "); 

  Serial.println(motor2_dir); 

 */ 

} 

  

  

void motor_position_data(String S) 

{ 

  int length = 20; 

  int i; 

  int start_position; 

  int comma_position; 

  int semicolon_position; 

  
      drv_dxl_tx_enable(TRUE); 

  

      DXL_PORT.write(0x01); 

      DXL_PORT.print("QP?;"); 

     

      drv_dxl_tx_enable(FALSE); 

  
      length = DXL_PORT.available(); 


      if( length > 0 ) 

      { 

         

        motor1_pos_str = ""; 

        motor2_pos_str = ""; 

     

        for(i=0; i<length; i++ ) 

        { 

         tx_pos_buffer[i] = DXL_PORT.read(); 

          if(tx_pos_buffer[i] == 'P') 
          { 
            start_position = i; 
          } 

          if(tx_pos_buffer[i] == ',') 

          { 

            comma_position = i; 

          } 

  

          if(tx_pos_buffer[i] == ';') 

          { 

            semicolon_position = i; 

          } 

        } 

  

        for(int i = start_position + 1 ; i < comma_position ; i++) 

        { 

          motor1_pos_str += tx_pos_buffer[i]; 

        } 

  

        for(int i = comma_position + 1 ; i < semicolon_position ; i++) 

        { 

          motor2_pos_str += tx_pos_buffer[i]; 

        } 

          

  

      } 

       

  } 

   
unsigned long hexatodecimal(String S){ 

 

  unsigned long pos = 1; 

  unsigned long ans = 0; 

 

  int len = S.length(); 

   

  for( int i = 7 ; i >= 0 ; i--) 

  { 

    if(S.charAt(i) < 65 ) 

    { 

      int n = S.charAt(i) - '0'; 

      ans += n*pos; 

    } 

     

    else 

    { 

      int n = S.charAt(i) - 55; 

      ans += n*pos; 

       

    } 

  

    pos *= 16; 

  } 

   return ans; 

    

}   

 

void motor_velocity_data(String SV) 

{ 

  int length = 20; 

  int i; 

  int start_position; 

  int comma_position; 

  int semicolon_position; 


      drv_dxl_tx_enable(TRUE); 

      DXL_PORT.write(0x01); 

      DXL_PORT.print("QV?;"); 

     

      drv_dxl_tx_enable(FALSE); 


      length = DXL_PORT.available(); 

      if( length > 0 ) 

      { 

         

        motor1_vel_str = ""; 

        motor2_vel_str = ""; 

     

        for(i=0; i<length; i++ ) 

        { 

          tx_pos_buffer[i] = DXL_PORT.read(); 

           

          if(tx_vel_buffer[i] == 'V') 

          { 

            start_position = i; 

          } 

           

          if(tx_vel_buffer[i] == ',') 

          { 

            comma_position = i; 

          } 

  

          if(tx_vel_buffer[i] == ';') 

          { 

            semicolon_position = i; 

          } 

        } 

  

        for(int i = start_position + 2 ; i < comma_position ; i++) 

        { 

          motor1_vel_str += tx_vel_buffer[i]; 

        } 

  

        for(int i = comma_position + 2 ; i < semicolon_position ; i++) 

        { 

          motor2_vel_str += tx_vel_buffer[i]; 

        } 

 

        motor1_dir = tx_vel_buffer[start_position + 1]; 

        motor2_dir = tx_vel_buffer[comma_position + 1]; 


      } 

       

  } 

 double relative_position(double current_pos, double previous_pos) 
 { 
  
  pre_relative_dir = relative_dir; 
  pre_relative_pos = relative_pos; 

  relative_pos = current_pos - previous_pos; 

  if(relative_pos > 0) 
  { 
    relative_dir = true; 
  } 

  else if(relative_pos < 0) 
  { 
    relative_dir = false; 
  } 

  if(relative_dir == pre_relative_dir) 

  { 

      return relative_pos; 

  } 

  else 

  { 

      return pre_relative_pos; 

  } 

   

 } 

int32_t relative_position_pul(unsigned long current_pul, unsigned long previous_pul) 
{ 
  
  pre_relative_dir = relative_dir; 
  pre_relative_pul = relative_pul; 

  relative_pul = current_pul - previous_pul; 
  
  if(relative_pul > 0) 
  { 
    relative_dir = true; 
  } 

  else if(relative_pul < 0) 
  { 
    relative_dir = false; 
  } 

  if(relative_dir == pre_relative_dir) 
  { 
      return relative_pul; 
  } 
  
  else 
  { 
      return pre_relative_pul; 
  } 
} 



 

double pulse_to_meter(unsigned long pulse) 

{ 

  double meter = pulse*0.00000442494; // pulse * ( 2*pi*r / (1024*4*26)) 

 

  return meter; 

 

} 

 

long Str_to_Long(String vel_str, char dir) 

{ 

  unsigned long pos = 1; 

  long ans = 0; 

 

  int len = vel_str.length(); 

   

  for( int i = len-1 ; i >= 0 ; i--) 

  { 

    

    int n = vel_str.charAt(i) - '0'; 

         

    ans += n*pos; 

  

    pos *= 10; 

    

  } 

 

  if(dir == '-') 

  { 

    return ans*(-1); 

  } 

  else{ 

    return ans; 

  } 

     

   

} 

 

double angle_vel(long left_vel, long right_vel) 
{
  
   long vel_rpm = right_vel - left_vel; 

 

   double vel_rad = 0.075 * 0.10472 * (vel_rpm);// rpm to m/s 

     

   double angle_w = vel_rad  * 5.2631578947368421052631578947368; 

 

   Serial.println(angle_w); 

     

   return angle_w;//이거 적분해서 세타구하기 

     

 

} 
