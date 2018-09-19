#include <ros.h> 
#include <std_msgs/String.h> 
#include <std_msgs/Char.h> 

#define DXL_USB_VER           20170915 
#define DXL_PORT              Serial3 
#define DXL_BAUD              115200 
#define DXL_TX_BUFFER_LENGTH  1024 

uint8_t tx_buffer[DXL_TX_BUFFER_LENGTH]; 
static uint32_t rx_data_cnt = 0; 
static uint32_t tx_data_cnt = 0; 

void motorCallback( const std_msgs::String& motor_msg) 
{ 
  drv_dxl_tx_enable(TRUE);  
  DXL_PORT.write(0x01); 
  DXL_PORT.print(motor_msg.data); 
  DXL_PORT.print(";"); 
  drv_dxl_tx_enable(FALSE); 
} 
ros::NodeHandle nh; 
ros::Subscriber<std_msgs::String> motor_sub("motor_out", motorCallback ); 

std_msgs::Char rx_msg; 
ros::Publisher pub_rx("rx", &rx_msg); 
  
char PE[8] = "PE0001;";
char SM[8] = "SM0505;";

void setup() 
{ 

  DXL_PORT.begin(DXL_BAUD); 

  nh.initNode(); 
  nh.subscribe(motor_sub); 
  nh.advertise(pub_rx); 
  drv_dxl_tx_enable(FALSE); 
 
  delay(3000);
  one();
  delay(1000);
  two();
} 

void loop() 
{ 
  rx_dxl(); 
  nh.spinOnce();   
} 

void rx_dxl() 
{ 
  //-- DXL -> USB 
  int length; 
  int i;   
  length = DXL_PORT.available(); 
  if ( length > 0 ) 
  { 
    if( length > DXL_TX_BUFFER_LENGTH ) 
    { 
      length = DXL_TX_BUFFER_LENGTH; 
    } 
    for(i=0; i<length; i++ ) 
    { 
      tx_buffer[i] = DXL_PORT.read(); 
      rx_msg.data=tx_buffer[i]; 
      pub_rx.publish(&rx_msg); 
    } 
    rx_data_cnt += length; 
  } 
} 

void one()
{
   drv_dxl_tx_enable(TRUE);

   DXL_PORT.write(0x01);
   DXL_PORT.print(PE);
    
   drv_dxl_tx_enable(FALSE);
}

void two()
{
  drv_dxl_tx_enable(TRUE);

   DXL_PORT.write(0x01);
   DXL_PORT.print(SM);
    
   drv_dxl_tx_enable(FALSE);
}
