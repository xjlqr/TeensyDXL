 /*------------------------------------------------------------------------------------------------------------------*/
// Libraries                                                                                                        //
/*------------------------------------------------------------------------------------------------------------------*/
#include "srv.h" // Servo control library

/*------------------------------------------------------------------------------------------------------------------*/
// Program constansts                                                                                               //
/*------------------------------------------------------------------------------------------------------------------*/
// Serial ports and baudrates ////////////////////////////////////////////////////////////////////////////////////////
#define USB    Serial    //USB Port
#define USB_BAUD 9600    //USB Baudrate
//
#define DXL      Serial3 //Servo Port
#define DXL_BAUD 2000000 //Servo Baudrate
,

// Torque Control ////////////////////////////////////////////////////////////////////////////////////////////////////
#define PP_GAIN  30.0     //Proportional-position gain (position based control) 81
#define IP_GAIN  10.0//20.0      //Integral-position gain     (steady state error cancelling) 4
#define DP_GAIN  1.0//1.0      //Derivative-position gain   (resists changes in position, causes instability with values over ~1.0)

#define PV_GAIN 1.0 //Proportional-velocity gain
#define IV_GAIN 0.5 //Integral-velocity gain
#define DV_GAIN 0.05 // Derivative-velocity gain

#define D_PERIOD 5 //Sampling period for error derivatives (in samples; values 3-15 are fine. Longer period means slower reaction)

#define PWM_LIMIT 400

// Torque Control Movement Functions /////////////////////////////////////////////////////////////////////////////////
#define P_FUNC(t)      180.0*sin(0.25*t) //Position function
#define V_FUNC(t) 0.25*180.0*cos(0.25*t) //Derivative of position function

// Timing
double to_seconds( size_t dt ){
  double seconds = 1.0 * dt;
  seconds = seconds / 1000000.0;
  return seconds;
}

/*------------------------------------------------------------------------------------------------------------------*/
// Servo Data Area                                                                                                  //
/*------------------------------------------------------------------------------------------------------------------*/
// Servo Information /////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t servo_ids[] = {1, 2, 3, 4, 5};                          //ID values for servos in chain (Should be constant, but breaks library [todo: fix])
const uint8_t servo_count = sizeof(servo_ids) / sizeof(uint8_t); // How many servos are attached (ie, 5 in this case)
const uint8_t active_servos = 5;//
const double motor_constants[][2] = {                           // Motor constants for torque-to-pwm conversion (linear approx.)
  {2.5577, 0.18100},                                            // datasheet values: 64:(2.5577, 0.18101) 106:{1.9022, 0.25432} 24:{8.1014, 0.22404}
  {1.9022, 0.25000},                                            // mesured values    64:(2.6840  0,17121) 106:                  24:(nan)
  {2.5577, 0.18000},
  {8.1014, 0.21850},
  {8.1014, 0.21950}
};

const double pos_limits[servo_count] = {180.0, 90.0, 90.0, 90.0, 90.0};

double pos_goals[servo_count] = {0.0, 0.0, 0.0, 0.0, 0.0};
double vel_goals[servo_count] = {0, 0, 0, 0, 0};

// Servo data /////////////////////////////////////////////////////////////////////////////////////////////////////////
address_t address_table[ADDRESS_TABLE_SIZE];                    // Address information from servo control table (in library srv.h)
double ang[servo_count]     = {0.0, 0.0, 0.0, 0.0, 0.0};        // Angles read from servo
double ang_vel[servo_count] = {0.0, 0.0, 0.0, 0.0, 0.0};        // Velocities read from servos
int pwm_goals[servo_count]  = {  0,   0,   0,   0,   0};        // PWM goals sent to servos

//
const int flat_data_size = 35;                                  // Size of data array (servo_count*rx_count)
int flat_data[flat_data_size];                                  // Data storage in flattened 2D array

// PID controller data //////////////////////////////////////////////////////////////////////////////////////
double pp_errors[servo_count] = {0.0, 0.0, 0.0, 0.0, 0.0}; //proportional-position errors
double ip_errors[servo_count] = {0.0, 0.0, 0.0, 0.0, 0.0}; //integral-position errors
double dp_errors[D_PERIOD][servo_count];                   //derivative-position errors

double pv_errors[servo_count] = {0.0, 0.0, 0.0, 0.0, 0.0}; //proportional-velocity errors
double iv_errors[servo_count] = {0.0, 0.0, 0.0, 0.0, 0.0}; //integral-velocity errors
double dv_errors[D_PERIOD][servo_count];                   //derivative-velocity errors

int correction[servo_count]   = {  0,   0,   0,   0,   0}; //PWM correction value

// Dynamic Modelling //////////////////////////////////////////////////////////////////////////////////////
// All functions calculate the torque applied by a specific external or internal force on the robot //
double link0gravity( double angles[] ) {
  return 0.0;
}

double link1gravity( double angles[] ) {
  return -0.45 * sin( PI * angles[1] * 0.00555555 ) + -0.30 * sin( PI * ( angles[1] + angles[2] ) * 0.0055555 );
}

double link2gravity( double angles[] ) {
  return -0.30 * sin( PI * ( angles[1] + angles[2] ) * 0.00555555 );
}

double (*gravity[servo_count])(double angles[]) = {
  link0gravity,
  link1gravity,
  link2gravity,
  link0gravity,
  link0gravity
};

/*------------------------------------------------------------------------------------------------------------------*/
// Data Transmission Aread                                                                                          //
/*------------------------------------------------------------------------------------------------------------------*/
// Data labels/count/size for reception //
const char *rx_labels[] = {
  "Torque Enable",
  "LED",
  "Goal PWM",
  "Present PWM",
  "Present Load",
  "Present Velocity",
  "Present Position",
};
const uint8_t rx_count = sizeof(rx_labels) / sizeof(char*); //Amount of strings in the rx array
const uint8_t rx_size = find_address(rx_labels[0]).byte_count + //Total size of data packet received from each servo (can't be calculated in loop because const)
                        find_address(rx_labels[1]).byte_count +
                        find_address(rx_labels[2]).byte_count +
                        find_address(rx_labels[3]).byte_count +
                        find_address(rx_labels[4]).byte_count +
                        find_address(rx_labels[5]).byte_count +
                        find_address(rx_labels[6]).byte_count;

// Data labels/count/size for transmission //
const char *tx_labels[] = {
  "Goal PWM",
};
const uint8_t tx_count = sizeof(tx_labels) / sizeof(char*);

uint8_t controls[] = {0, 0, 0, 0};
uint8_t control_state[] = {1, 0, 0, 0, 0};
/*------------------------------------------------------------------------------------------------------------------*/
// Timing Area                                                                                                      //
/*------------------------------------------------------------------------------------------------------------------*/
size_t step_start =   0;
size_t dt         =   0;
double t          = 0.0;
double pt         = 0.0;
size_t loop_counter = 0;
char inputString[100];
bool stringComplete = false;
bool handshake_complete = false;
void setup() {
  //Load address table
  initialize_address_table( address_table );

  //Dummy calls to torque functions (prevents linker from wrongly removing indirect function calls)
  link0gravity(ang);
  link1gravity(ang);
  link2gravity(ang);

  //Zero out the D errors
  for( int i = 0 ; i < D_PERIOD ; i++ ){
    for( int j = 0; j < servo_count ; j++ ){
        dp_errors[i][j] = 0.0;
        dv_errors[i][j] = 0.0;
    }
  }

  //Enable serial transmission
  USB.begin( USB_BAUD );
  DXL.begin( DXL_BAUD );
  DXL.transmitterEnable( LED_BUILTIN );

  //Delay inserted for opening Serial Monitor/Plotter
  delay(2000);

  //Wake up and configure the servos
  for ( int i = 1 ; i <= active_servos ; i++ ) {
    USB.print("Configuring servo "); USB.print(i); USB.println(":");

    while ( srv_read( &DXL, i, "ID" ) != i ) {
      USB.print("Plug in servo "); USB.print(i); USB.println("!"); delay(2000);
    }

    //Reduce communication time
    USB.print("Return Time "); USB.println(srv_read( &DXL, i, "Return Delay Time" ));
    srv_write( &DXL, i, "Return Delay Time", 5);

    //Enable PWM control
    USB.print("Operating Mode "); USB.println(srv_read( &DXL, i, "Operating Mode") );
    srv_write( &DXL, i, "Operating Mode", 16 );

    //Configure indirect memory access for use with sync read/write
    USB.println("Configuring memory access...");
    srv_assign_indirect_memory( &DXL, i, "Indirect Address 1", rx_labels, rx_count, address_table );
  }

  //Turns on LED and torque for all servos
  srv_write( &DXL, BROADCAST, "Torque Enable", true );
  srv_write( &DXL, BROADCAST, "LED", true );

  //Check that handshake succeeded
  const char* handshake = "HANDSHAKE:164897\n";
  while(!handshake_complete){
    handshake_complete = false; 
    int k = 0; 
    if(stringComplete){
      while(inputString[k] != '\0'){
        if( inputString[k] == handshake[k] ){
          handshake_complete = true;
          USB.print(inputString[k++]);
        }else{
          handshake_complete = false;
          break;
        }
      }
      stringComplete = false;
    }
    USB.println("Waiting for handshake: HANDSHAKE:164897");
    delay(1000);
  }

  controls[0] = 90;
  controls[1] = 90;
  controls[2] = 90;
  controls[3] = 1;

  //Reset timer
  step_start = micros();
  dt = 1000000;
}

int counter = 0;
void loop() {
  bool received = srv_sync_read( &DXL, servo_ids, active_servos, "Indirect Data 1", rx_size, rx_labels, flat_data, rx_count ); //Returns false if transmission fails
  if ( received ) {
    for ( int i = 0; i < active_servos ; i++ ) {
      ang[i]     = pos_2_deg( flat_data[i * rx_count + 6] );
      ang_vel[i] = vel_2_rpm( flat_data[i * rx_count + 5] );

      int sign = 0;
      double magnitude = 0.0;
      double error = 0.0;
      switch(i){
        case 0:
          error = ( (controls[0]*2.0 - 180.0 ) - ang[i] )/360.0;
          ( error > 0 )?(sign = 1):(sign = -1);
          magnitude = abs( error );
          magnitude = pow( magnitude, 0.8 ) ;
          vel_goals[i] = 600.0 * sign * magnitude ;
          pos_goals[i] = pos_goals[i] + vel_goals[i]*to_seconds(dt);
          break;

        case 1:
          error = ( ( 90.0 - 2.0*(controls[1]-45.0) ) - ang[i] )/90.0;
          ( error > 0 )?(sign = 1):(sign = -1);
          magnitude = abs( error );
          magnitude = pow( magnitude, 0.7 ) ;
          vel_goals[i] = 200.0 * sign * magnitude ;
          pos_goals[i] = pos_goals[i] + vel_goals[i]*to_seconds(dt);
          break;

        case 2:
            error = 2.0*( controls[2] - 90 );
            if( abs(error) > 20 ){
              vel_goals[i] = error;
              pos_goals[i] = pos_goals[i] + vel_goals[i]*to_seconds(dt);
            }
          break;
        case 3:
          if(controls[3] == 1){
            pos_goals[i] = -90.0;
            vel_goals[i] = 0;
          }else if(controls[3] == 4){
            pos_goals[i] = -5.0;
            vel_goals[i] = 0;
          }
          break;
        case 4:
          pos_goals[i] = -pos_goals[i-1];
          break;
        
        default:
          pos_goals[i] = 0.0;
          vel_goals[i] = 0.0;
          break;
        }

      

      if( abs(pos_goals[i]) > pos_limits[i] ){
        vel_goals[i] = 0.0;
      }
      pos_goals[i] = max( pos_goals[i], -pos_limits[i] );
      pos_goals[i] = min( pos_goals[i],  pos_limits[i] );

      //Apply force cancelling (currently, only gravity and motor internal forces are applied)
      correction[i] = torque_2_pwm( gravity[i](ang), ang_vel[i], motor_constants[i] );

      //Position error calculation
      double new_p_error = pos_goals[i] - ang[i];
      double p_error_change = ( new_p_error - pp_errors[i] ) / to_seconds( dt );
      dp_errors[loop_counter][i] = p_error_change;
      double p_average = 0;
      for( int j = 0; j < D_PERIOD; j++ ){
        p_average += dp_errors[j][i];
      }
      p_average /= 1.0*D_PERIOD;
      pp_errors[i] = new_p_error;
      ip_errors[i] = ip_errors[i] + pp_errors[i] * to_seconds( dt );

      //Velocity error calculation
      double new_v_error = vel_goals[i] - ang_vel[i];
      double v_error_change = ( new_v_error - pv_errors[i] ) / to_seconds( dt );
      dv_errors[loop_counter][i] = v_error_change;
      double v_average = 0;
      for( int j = 0; j < D_PERIOD; j++ ){
        v_average += dv_errors[j][i];
      }
      v_average /= 1.0*D_PERIOD;
      pv_errors[i] = new_v_error;
      iv_errors[i] = iv_errors[i] + pv_errors[i] * to_seconds( dt );

      //Position correction
      correction[i] += PP_GAIN * pp_errors[i];
      if(!(i == 3 || i == 4)){
        correction[i] += IP_GAIN * ip_errors[i];
      }
      correction[i] += DP_GAIN * p_average   ;
      
      //Velocity correction
      (vel_goals[i] != 0)?(correction[i] += PV_GAIN * pv_errors[i]):(correction[i] += 0);
      (vel_goals[i] != 0)?(correction[i] += IV_GAIN * iv_errors[i]):(correction[i] += 0);
      (vel_goals[i] != 0)?(correction[i] += DV_GAIN * v_average   ):(correction[i] += 0);

      //Limit PWM value to maximum allowable
      correction[i] = max( correction[i], -PWM_LIMIT );
      correction[i] = min( correction[i],  PWM_LIMIT );
      if( t > 0.5 ){
        pwm_goals[i] = correction[i];
      }
    }
  } else {
    for ( int i = 0; i < active_servos ; i++ ) {
      pwm_goals[i] = 0;
    }
  }

  //Write PWM data to servos
  srv_sync_write( &DXL, servo_ids, active_servos, "Indirect Data 3", tx_labels, pwm_goals, tx_count);

  //Update loop counter (for derivative gains)
  loop_counter++;
  loop_counter = loop_counter%D_PERIOD;

  //Update timing system & print serial output
  pt = pt + to_seconds( dt );
  if ( ( pt > 0.01 ) ) {
    //Serial.print(t);Serial.print(",");Serial.print(",");
    for( int i = 0 ; i < servo_count ; i++ ){
      //Serial.print(ang[i]); Serial.print(",");
      //Serial.print(ang_vel[i]);Serial.print(" ");
    }//Serial.println();
    Serial.println(" ");
    pt = 0;
  }

  if( stringComplete ){
    if( t > 0.5){
      for( int i = 0; i < 4 ; i++ ){
        if( inputString[i] == '\0' ){
          break;
        }
        controls[i] = inputString[i];
      }
    }
    stringComplete = false;
  }

  dt = micros() - step_start;
  t = t + to_seconds( dt );
  step_start = micros();
}

//Input handling
void serialEvent() {
  int i = 0;
  while (Serial.available() && !stringComplete) {
    uint8_t inChar = (uint8_t)Serial.read(); 
    inputString[i++] = inChar; 
    if (inChar == '\n') {
      inputString[i] = '\0';
      stringComplete = true;
      break;
    }
  }
}

