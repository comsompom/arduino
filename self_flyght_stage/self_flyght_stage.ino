int time_counter;
int stage_fly;
float up_down_signal;

void setup() {
  time_counter = 1;
  stage_fly = 0;
  up_down_signal = 1000;
  Serial.begin(57600);
}

void loop() {
  check_stage();
  Serial.print("Up - Down: ");
  Serial.println(up_down_signal);

  // Serial.print("Timer: ");
  // Serial.println(time_counter);
}

void check_stage() {
  time_counter += 1;
  if(time_counter == 2000) {
    if(stage_fly == 0){
      stage_fly = 1;
      time_counter = 0;
    }
  }

  if(time_counter == 2000) {
    if(stage_fly == 1){
      stage_fly = 2;
      time_counter = 0;
    }
  }

  if(time_counter == 2000) {
    if(stage_fly == 2){
      stage_fly = 3;
      time_counter = 0;
    }
  }

  if(time_counter == 2000) {
    if(stage_fly == 3){
      stage_fly = 4;
      time_counter = 0;
    }
  }

  if(time_counter == 2000) {
    if(stage_fly == 4){
      stage_fly = 5;
      time_counter = 0;
    }
  }

  if(time_counter == 2000) {
    if(stage_fly == 5){
      stage_fly = 6;
      time_counter = 0;
    }
  }

  if(time_counter == 2000) {
    if(stage_fly == 6){
      stage_fly = 7;
      time_counter = 0;
    }
  }

  if(time_counter == 2000) {
    if(stage_fly == 7){
      stage_fly = 0;
      time_counter = 0;
    }
  }

  if(time_counter < 2000){
    if(stage_fly < 4){
      up_down_signal += (int)((time_counter / 1800));
    }
    if(stage_fly > 3){
      up_down_signal -= (int)((time_counter / 1800));
    }
  }
}