# Pin info


## OpenCR

Dynamicel-RS485 GND Pin

Dynamicel-RS485 + Pin

Dynamicel-RS485 - Pin

## Motor controller

- controller_side_Pin

SCI+      gray

SCI-      white

DGND      blue

U1        blue

V1        yellow

W1        white

U2        blue

V2        yellow

W2        white

+24V      red

PGND      black

- controller_hall_Pin

Pin1(AGND)          black      

Pin2(Hall W)        white

Pin3(Hall V)        yellow

Pin4(Hall U)        blue

Pin5(+5V)           red


- controller_encoder_Pin

Pin1(N.C.)          gray
Pin2(Vcc)           gray
Pin3(GND)           gray
Pin4(N.C.)          gray
Pin5(Channel -A)    gray
Pin6(Channel  A)    gray
Pin7(Channel -B)    gray
Pin8(Channel  B)    gray
Pin9(Channel -Z)    gray
Pin10(Channel  Z)   gray

## MaxonMotor

https://www.maxonmotor.co.kr/medias/sys_master/root/8825435324446/17-EN-270.pdf

- MOTOR

Pin1(U)         red
pin2(V)         black
pin3(W)         white
pin4(N.C.)      NON

- HALLSENSOR

Pin1(Hall U)    gray
Pin2(Hall V)    gray
Pin3(Hall W)    gray
Pin4(GND)       gray
Pin5(V_hall 5V) blue
Pin6(N.C.)      NON


## Encoder

https://www.maxonmotor.co.kr/medias/sys_master/root/8825853771806/17-EN-389.pdf

Pin1(N.C.)          gray
Pin2(Vcc)           gray
Pin3(GND)           gray
Pin4(N.C.)          gray
Pin5(Channel -A)    gray
Pin6(Channel  A)    gray
Pin7(Channel -B)    gray
Pin8(Channel  B)    gray
Pin9(NON)           gray
Pin10(NON)          gray


## Connection of wire

OpenCR                              motor_controller_side_Pin

|OpenCR| |motor_controller_side_Pin|
|:--:|:--:|:--:|
|Dynamicel-RS485 + Pin      |gray   |SCI+|
|Dynamicel-RS485 - Pin      |white  |SCI-|
|Dynamicel-RS485 GND Pin    |blue   |DGND|


Motor controller                          Maxon_Motor

- controller_side_Pin                        MOTOR1

U1 ---------- blue------------ red--------- Pin1(U)    
V1 --------- yellow----------- black ------ pin2(V)      
W1 ---------- white ---------- white ------ pin3(W)  

                                            MOTOR2

U2 ---------- blue------------ red--------- Pin1(U)    
V2 --------- yellow----------- black ------ pin2(V)      
W2 ---------- white ---------- white ------ pin3(W)  

- controller_hall_Pin                        MOTOR1,2_HALLSENSOR

Pin1(AGND) -------- black ------- gray ---- Pin4(GND)      
Pin2(Hall W) ------ white ------- gray ---- Pin3(Hall W)    
Pin3(Hall V) ------ yellow ------ gray ---- Pin2(Hall V)   
Pin4(Hall U) ------ blue -------- gray ---- Pin1(Hall U)   
Pin5(+5V) --------- red --------- gray ---- Pin5(V_hall 5V)


- controller_encoder_Pin                     Encoder1,2

Pin1(N.C.) -------------- gray ------------ Pin1(N.C.) 
Pin2(Vcc) --------------- gray ------------ Pin2(Vcc)
Pin3(GND) --------------- gray ------------ Pin3(GND) 
Pin4(N.C.) -------------- gray ------------ Pin4(N.C.) 
Pin5(Channel -A) -------- gray ------------ Pin5(Channel -A)
Pin6(Channel  A) -------- gray ------------ Pin6(Channel  A)
Pin7(Channel -B) -------- gray ------------ Pin7(Channel -B)
Pin8(Channel  B) -------- gray ------------ Pin8(Channel  B) 
Pin9(Channel -Z) -------- gray              Pin9(Channel -Z)
Pin10(Channel  Z) ------- gray              Pin10(Channel  Z) 

- controller_Side_Pin                        battery   

+24V -------------------- red ------------- Vcc
PGND -------------------- black ----------- GND  
