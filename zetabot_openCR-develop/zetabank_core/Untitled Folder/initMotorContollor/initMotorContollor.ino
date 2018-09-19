void init_MotorContollor(){ 
  /***
  Initialize motor parameters for working motor using motor controller(BL2408-DID)
  ***/
  
  DXL_PORT.begin(115200); // Serial3 = DXL_PORT
  delay(3000); 
   
   // To transmit commands, set True.
   drv_dxl_tx_enable(TRUE); // RS485 Tx mode on

   /*
   Serial.write()
   - Writes binary data to the serial port. This data is sent as a byte or series of bytes; to send the characters representing the digits of a number use the print() function instead.

  Serial.print()
  - Prints data to the serial port as human-readable ASCII text.
   */
  
   DXL_PORT.write(0x01); // Setting driver id to 0x01 
   DXL_PORT.print("PE0001;"); // host id 01 set
  
   // To receive messages, set False.
   drv_dxl_tx_enable(FALSE); // RS485 Tx mode off
   delay(1000);
    
   // To transmit commands, set True.
   drv_dxl_tx_enable(TRUE); // RS485 tx mode on

   DXL_PORT.write(0x01); // driver id 01 
   DXL_PORT.print("SM0505;"); // driver control mode 05 => Velocity control mode
    
   drv_dxl_tx_enable(FALSE); // RS485 tx mode off
  
}

