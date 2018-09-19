void motor_to_msg(float left_vel, float right_vel)
{
 
  String SV = "SV";

  float left_vel;
  float left_vel_100;
  int Motor1_vel_int;
  String Motor1_vel;
  
  String comma = ",";

  float right_vel;
  float right_vel_100;
  int Motor2_vel_int;
  String Motor2_vel;
  
  String Semicolon = ";";

  left_vel_100 = left_vel * 100;
  right_vel_100 = right_vel * 100;
  
  Motor1_vel_int = (int)left_vel_100; 
  Motor1_vel_int = (int)left_vel_100; 

  Motor1_vel = String(Motor1_vel_int);
  Motor2_vel = String(Motor2_vel_int);
  
    drv_dxl_tx_enable(TRUE);

    DXL_PORT.write(0x01);      
    DXL_PORT.print(SV+Motor1_vel+comma+Motor2_Vel+Semicolon);

    drv_dxl_tx_enable(FALSE);
  
}


