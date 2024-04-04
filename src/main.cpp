#include <Arduino.h>
#include "motor_control.h"

void setup(){
  motor_init_i();

}




void loop(){

  motor_run_i();
  static float target_1 = 0;
  static float target_2 = 0;
  change_target_right(target_1);
  target_1 += 0.001;
  change_target_left(target_2);
  target_2 -= 0.001;

}





