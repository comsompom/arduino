///////////////////////////////////////////////////////////////////////////////////////
//Terms of use
///////////////////////////////////////////////////////////////////////////////////////
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.
///////////////////////////////////////////////////////////////////////////////////////
//Safety note
///////////////////////////////////////////////////////////////////////////////////////
//Always remove the propellers and stay away from the motors unless you 
//are 100% certain of what you are doing.
///////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>                          //Include the Wire.h library so we can communicate with the gyro.
#include <EEPROM.h>                        //Include the EEPROM.h library so we can store information onto the EEPROM

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 1.3;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.04;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 18.0;              //Gain setting for the roll D-controller
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)

// Altitude PID controller settings
float pid_p_gain_altitude = 2.0;           //Gain setting for the altitude P-controller
float pid_i_gain_altitude = 0.05;          //Gain setting for the altitude I-controller
float pid_d_gain_altitude = 15.0;          //Gain setting for the altitude D-controller
int pid_max_altitude = 300;                //Maximum output of the altitude PID-controller (+/-)

boolean auto_level = true;                 //Auto level on (true) or off (false)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte eeprom_data[36];
int cal_int, start, gyro_address;
int temperature;
int acc_axis[4], gyro_axis[4];
float roll_level_adjust, pitch_level_adjust;
long acc_x, acc_y, acc_z, acc_total_vector;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer;
// Add missing ESC timer channels for output pulse timing
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_axis_cal[4];
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
float pid_i_mem_altitude, pid_output_altitude, pid_last_altitude_d_error;
boolean gyro_angles_set;

// Flight control variables
unsigned long flight_start_timer;
byte flight_phase = 0;
float target_altitude = 0;
float current_altitude = 0;
float forward_speed = 0;
int base_throttle = 1500;
int esc_1, esc_2, esc_3, esc_4;
int throttle;

int led_setup;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  for(start = 0; start <= 35; start++)eeprom_data[start] = EEPROM.read(start);
  start = 0;                                                                
  gyro_address = eeprom_data[32];                                           

  Wire.begin();                                                             
  TWBR = 12;                                                                

  //Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs.
  DDRD |= B11110000;                                                        //Configure digital poort 4, 5, 6 and 7 as output.
  DDRB |= B00110000;                                                        //Configure digital poort 12 and 13 as output.

  //Check the EEPROM signature to make sure that the setup program is executed.
  while(eeprom_data[33] != 'J' || eeprom_data[34] != 'M' || eeprom_data[35] != 'B')delay(10);

  //The flight controller needs the MPU-6050 with gyro and accelerometer
  //If setup is completed without MPU-6050 stop the flight controller program  
  if(eeprom_data[31] == 2 || eeprom_data[31] == 3)delay(10);

  set_gyro_registers();                                                     

  // Two-step gyro calibration
  // 1. First calibration: motors off
  calibrate_gyro(2000, 0);
  // 2. Second calibration: motors at 1100 (props OFF for safety!)
  calibrate_gyro(2000, 1100);

  // Initialize flight control variables
  flight_start_timer = millis();
  flight_phase = 0;
  target_altitude = 0;
  current_altitude = 0;
  forward_speed = 0;
  
  // Reset PID controllers
  pid_i_mem_roll = 0;
  pid_last_roll_d_error = 0;
  pid_i_mem_pitch = 0;
  pid_last_pitch_d_error = 0;
  pid_i_mem_yaw = 0;
  pid_last_yaw_d_error = 0;
  pid_i_mem_altitude = 0;
  pid_last_altitude_d_error = 0;

  angle_pitch = angle_pitch_acc;                                          
  angle_roll = angle_roll_acc;                                           
  gyro_angles_set = true;                                                 

  // Setup complete indication
  setup_complete_blink();
  
  loop_timer = micros();                                                    
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){
  // Flight control state machine
  unsigned long flight_time = millis() - flight_start_timer;
  
  // State machine for autonomous flight
  switch(flight_phase) {
    case 0: // Wait 5 seconds before takeoff
      if(flight_time > 5000) {
        flight_phase = 1;
        target_altitude = 100; // Target 1 meter (100 cm)
      }
      break;
      
    case 1: // Rising to 1 meter
      if(current_altitude >= 95) { // Allow 5cm tolerance
        flight_phase = 2;
        forward_speed = 30; // Start moving forward
      }
      break;
      
    case 2: // Moving forward
      if(flight_time > 12000) { // After ~7 seconds of forward movement (5s + 7s = 12s)
        flight_phase = 3;
        forward_speed = 0;
        target_altitude = 0; // Begin descent
      }
      break;
      
    case 3: // Landing
      if(current_altitude < 5) { // Almost at ground level
        flight_phase = 4;
      }
      break;
      
    case 4: // Flight complete - motors off
      target_altitude = 0;
      forward_speed = 0;
      break;
  }

  // Calculate altitude from accelerometer data
  current_altitude += (acc_z - acc_total_vector) * 0.0000611; // Simple integration
  
  //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
  gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_roll / 65.5) * 0.3);   
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_pitch / 65.5) * 0.3);
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_yaw / 65.5) * 0.3);      
  
  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_pitch * 0.0000611;                                    
  angle_roll += gyro_roll * 0.0000611;                                      

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066);                  
  angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066);                  

  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));       
  
  if(abs(acc_y) < acc_total_vector){                                        
    angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;          
  }
  if(abs(acc_x) < acc_total_vector){                                        
    angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;          
  }
  
  angle_pitch_acc -= 0.0;                                                   
  angle_roll_acc -= 0.0;                                                    
  
  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;            
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;               

  pitch_level_adjust = angle_pitch * 15;                                    
  roll_level_adjust = angle_roll * 15;                                      

  if(!auto_level){                                                          
    pitch_level_adjust = 0;                                                 
    roll_level_adjust = 0;                                                  
  }

  // Set control parameters based on flight phase
  if(flight_phase < 4) {
    // Altitude control
    pid_error_temp = target_altitude - current_altitude;
    pid_i_mem_altitude += pid_i_gain_altitude * pid_error_temp;
    if(pid_i_mem_altitude > pid_max_altitude)pid_i_mem_altitude = pid_max_altitude;
    else if(pid_i_mem_altitude < pid_max_altitude * -1)pid_i_mem_altitude = pid_max_altitude * -1;
    
    pid_output_altitude = pid_p_gain_altitude * pid_error_temp + pid_i_mem_altitude + 
                         pid_d_gain_altitude * (pid_error_temp - pid_last_altitude_d_error);
    
    if(pid_output_altitude > pid_max_altitude)pid_output_altitude = pid_max_altitude;
    else if(pid_output_altitude < pid_max_altitude * -1)pid_output_altitude = pid_max_altitude * -1;
    
    pid_last_altitude_d_error = pid_error_temp;
    
    // Base throttle + altitude adjustment
    throttle = base_throttle + pid_output_altitude;
    
    // Forward movement control
    pid_pitch_setpoint = forward_speed;
    pid_roll_setpoint = 0; // Keep roll level
    pid_yaw_setpoint = 0;  // Keep yaw stable
    
    calculate_pid();
    
    //The PID set point in degrees per second is determined by the roll receiver input.
    //The PID set point is limited to 400 degrees per second by the variable pid_max_roll.
    //This is done to prevent the PID controller to be used beyond its limits.
    if(throttle > 1800) throttle = 1800;                   
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

    if(esc_1 < 1100) esc_1 = 1100;                                         
    if(esc_2 < 1100) esc_2 = 1100;                                         
    if(esc_3 < 1100) esc_3 = 1100;                                         
    if(esc_4 < 1100) esc_4 = 1100;                                         

    if(esc_1 > 2000)esc_1 = 2000;                                           
    if(esc_2 > 2000)esc_2 = 2000;                                           
    if(esc_3 > 2000)esc_3 = 2000;                                           
    if(esc_4 > 2000)esc_4 = 2000;                                           
  }
  else {
    esc_1 = 1000;                                                           
    esc_2 = 1000;                                                           
    esc_3 = 1000;                                                           
    esc_4 = 1000;                                                           
  }

  if(micros() - loop_timer > 4050)digitalWrite(12, HIGH);                   
  
  while(micros() - loop_timer < 4000);                                      
  loop_timer = micros();                                                    

  PORTD |= B11110000;                                                       
  timer_channel_1 = esc_1 + loop_timer;                                     
  timer_channel_2 = esc_2 + loop_timer;                                     
  timer_channel_3 = esc_3 + loop_timer;                                     
  timer_channel_4 = esc_4 + loop_timer;                                     
  
  gyro_signalen();

  while(PORTD >= 16){                                                       
    current_time = micros();                                              
    if(timer_channel_1 <= current_time)PORTD &= B11101111;                
    if(timer_channel_2 <= current_time)PORTD &= B11011111;                
    if(timer_channel_3 <= current_time)PORTD &= B10111111;                
    if(timer_channel_4 <= current_time)PORTD &= B01111111;                
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for reading the gyro
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gyro_signalen(){
  //Read the MPU-6050
  if(eeprom_data[31] == 1){
    Wire.beginTransmission(gyro_address);                                   //Start communication with the gyro.
    Wire.write(0x3B);                                                       //Start reading @ register 43h and auto increment with every read.
    Wire.endTransmission();                                                 //End the transmission.
    Wire.requestFrom(gyro_address,14);                                      //Request 14 bytes from the gyro.
    
    while(Wire.available() < 14);                                           //Wait until the 14 bytes are received.
    acc_axis[1] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the acc_x variable.
    acc_axis[2] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the acc_y variable.
    acc_axis[3] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the acc_z variable.
    temperature = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the temperature variable.
    gyro_axis[1] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
    gyro_axis[2] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
    gyro_axis[3] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
  }

  if(cal_int == 2000){
    gyro_axis[1] -= gyro_axis_cal[1];                                       //Only compensate after the calibration.
    gyro_axis[2] -= gyro_axis_cal[2];                                       //Only compensate after the calibration.
    gyro_axis[3] -= gyro_axis_cal[3];                                       //Only compensate after the calibration.
  }
  gyro_roll = gyro_axis[eeprom_data[28] & 0b00000011];                      //Set gyro_roll to the correct axis that was stored in the EEPROM.
  if(eeprom_data[28] & 0b10000000)gyro_roll *= -1;                          //Invert gyro_roll if the MSB of EEPROM bit 28 is set.
  gyro_pitch = gyro_axis[eeprom_data[29] & 0b00000011];                     //Set gyro_pitch to the correct axis that was stored in the EEPROM.
  if(eeprom_data[29] & 0b10000000)gyro_pitch *= -1;                         //Invert gyro_pitch if the MSB of EEPROM bit 29 is set.
  gyro_yaw = gyro_axis[eeprom_data[30] & 0b00000011];                       //Set gyro_yaw to the correct axis that was stored in the EEPROM.
  if(eeprom_data[30] & 0b10000000)gyro_yaw *= -1;                           //Invert gyro_yaw if the MSB of EEPROM bit 30 is set.

  acc_x = acc_axis[eeprom_data[29] & 0b00000011];                           //Set acc_x to the correct axis that was stored in the EEPROM.
  if(eeprom_data[29] & 0b10000000)acc_x *= -1;                              //Invert acc_x if the MSB of EEPROM bit 29 is set.
  acc_y = acc_axis[eeprom_data[28] & 0b00000011];                           //Set acc_y to the correct axis that was stored in the EEPROM.
  if(eeprom_data[28] & 0b10000000)acc_y *= -1;                              //Invert acc_y if the MSB of EEPROM bit 28 is set.
  acc_z = acc_axis[eeprom_data[30] & 0b00000011];                           //Set acc_z to the correct axis that was stored in the EEPROM.
  if(eeprom_data[30] & 0b10000000)acc_z *= -1;                              //Invert acc_z if the MSB of EEPROM bit 30 is set.
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for calculating pid outputs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//The PID controllers 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calculate_pid(){
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}

void set_gyro_registers(){
  //Setup the MPU-6050
  if(eeprom_data[31] == 1){
    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
    Wire.write(0x6B);                                                          //We want to write to the PWR_MGMT_1 register (6B hex)
    Wire.write(0x00);                                                          //Set the register bits as 00000000 to activate the gyro
    Wire.endTransmission();                                                    //End the transmission with the gyro.

    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
    Wire.write(0x1B);                                                          //We want to write to the GYRO_CONFIG register (1B hex)
    Wire.write(0x08);                                                          //Set the register bits as 00001000 (500dps full scale)
    Wire.endTransmission();                                                    //End the transmission with the gyro

    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
    Wire.write(0x1C);                                                          //We want to write to the ACCEL_CONFIG register (1A hex)
    Wire.write(0x10);                                                          //Set the register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission();                                                    //End the transmission with the gyro

    //Let's perform a random register check to see if the values are written correct
    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search
    Wire.write(0x1B);                                                          //Start reading @ register 0x1B
    Wire.endTransmission();                                                    //End the transmission
    Wire.requestFrom(gyro_address, 1);                                         //Request 1 bytes from the gyro
    while(Wire.available() < 1);                                               //Wait until the 6 bytes are received
    if(Wire.read() != 0x08){                                                   //Check if the value is 0x08
      digitalWrite(12,HIGH);                                                   //Turn on the warning led
      while(1)delay(10);                                                       //Stay in this loop for ever
    }

    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search
    Wire.write(0x1A);                                                          //We want to write to the CONFIG register (1A hex)
    Wire.write(0x03);                                                          //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
    Wire.endTransmission();                                                    //End the transmission with the gyro    

  }  
}

void setup_complete_blink(){
  digitalWrite(LED_BUILTIN, LOW);
  delay(2000);
  for (led_setup = 0; led_setup < 3; led_setup ++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }
}

// Add calibration function before setup
void calibrate_gyro(int samples, int motor_speed) {
  // Reset calibration accumulators
  gyro_axis_cal[1] = 0;
  gyro_axis_cal[2] = 0;
  gyro_axis_cal[3] = 0;

  for (cal_int = 0; cal_int < samples; cal_int++) {
    if (cal_int % 15 == 0) digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    gyro_signalen();
    gyro_axis_cal[1] += gyro_axis[1];
    gyro_axis_cal[2] += gyro_axis[2];
    gyro_axis_cal[3] += gyro_axis[3];

    // Run motors at the specified speed (props OFF for safety!)
    if (motor_speed > 0) {
      esc_1 = esc_2 = esc_3 = esc_4 = motor_speed;
      PORTD |= B11110000;
      delayMicroseconds(motor_speed);
      PORTD &= B00001111;
    } else {
      // Just send a short pulse to keep ESCs quiet
      PORTD |= B11110000;
      delayMicroseconds(1000);
      PORTD &= B00001111;
    }
    delay(3);
  }
  gyro_axis_cal[1] /= samples;
  gyro_axis_cal[2] /= samples;
  gyro_axis_cal[3] /= samples;
}
