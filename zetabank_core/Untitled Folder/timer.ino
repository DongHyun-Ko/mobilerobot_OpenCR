#include<MsTimer2.h>

void setup() {

  MsTimer2::set(500, flash); // flash함수를 500ms마다 호출한다
  MsTimer2::start();
    

}

void loop() {
  // put your main code here, to run repeatedly:

}

void flash() {

  //위치받아오기
  //int32_t로 변환
  //속도받아오기
  //long으로 변환
  
}
