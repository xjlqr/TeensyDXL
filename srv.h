#ifndef srv_h
#define srv_h

//This file contains the high level servo interface functions. Use these.

#define MAX_BUFFER_LENGTH 255 //How much data in bytes a single servo is alloted.
#define BROADCAST 254         //ID for addressing all servos instead of a single one.
#define DXL_TIMEOUT 2000      //Timeout for reading data from the servos in microseconds. A rx-tx cycle takes about 1000 microseconds normally.

//This section is copied from the low level library and contains address table information for the specific servo used
//currently configured for the MX28(2.0), should work for all MX(2.0) series servos (disallows current control on MX64 and 106)
#define ADDRESS_TABLE_SIZE 662
#define ADDRESS_INFORMATION_SIZE 162

//Converts integer position to angle in degrees
double pos_2_deg( int pos ){
  double  tmp = (double) (( pos*1.0) -4096.0/2.0) * 1.0;
  return tmp*0.088;
}

//Converts integer velocity to revolutions-per-minute
float vel_2_rpm( int vel ){
  float tmp = (float) vel * 1.0;
  return tmp*0.229f;
}

//Converts load to percentage of max allowed
float lod_2_prc( int lod ){
  float tmp = (float) lod * 1.0;
  if( tmp < 2000 ){
     return tmp*0.1f;
  }else{
    return (tmp - pow(2, 16))*0.1f;
  }
}

//Converts pwm to voltage
float pwm_2_vlt( int pwm ){
  float tmp = (float) pwm * 1.0;
  return tmp * 0.13559322f;
}

int torque_2_pwm ( double torque, double velocity, const double constants[] ){
  return (int) ( (885.0/12.0)*(torque*constants[0] + velocity*constants[1]) );
}

struct address_t{
  uint16_t index;
  uint8_t byte_count;
  bool write_permission;
  bool in_ram;
  const char *label;
};

typedef struct address_t address_t;

const address_t nil_address = { 0, 0, 0, 0, "nil" };

const address_t address_information[ADDRESS_INFORMATION_SIZE] = {
    {  0, 2, 0, 0, "Model Number"           },
    {  2, 4, 0, 0, "Model Information"      },
    {  6, 1, 0, 0, "Firmware Version"       },
    {  7, 1, 1, 0, "ID"                     },
    {  8, 1, 1, 0, "Baud Rate"              },
    {  9, 1, 1, 0, "Return Delay Time"      },
    { 10, 1, 1, 0, "Drive Mode"             },
    { 11, 1, 1, 0, "Operating Mode"         },
    { 12, 1, 1, 0, "Secondary(Shadow) ID"   },
    { 13, 1, 1, 0, "Protocol Version"       },
    { 20, 4, 1, 0, "Homing Offset"          },
    { 24, 4, 1, 0, "Moving Threshold"       },
    { 31, 1, 1, 0, "Temperature Limit"      },
    { 32, 2, 1, 0, "Max Voltage Limit"      },
    { 34, 2, 1, 0, "Min Voltage Limit"      },
    { 36, 2, 1, 0, "PWM Limit"              },
    { 40, 4, 1, 0, "Acceleration Limit"     },
    { 44, 4, 1, 0, "Velocity Limit"         },
    { 48, 4, 1, 0, "Max Position Limit"     },
    { 52, 4, 1, 0, "Min Position Limit"     },
    { 63, 1, 1, 0, "Shutdown"               },
    { 64, 1, 1, 1, "Torque Enable"          },
    { 65, 1, 1, 1, "LED"                    },
    { 68, 1, 1, 1, "Status Return Level"    },
    { 69, 1, 0, 1, "Registered Instruction" },
    { 70, 1, 0, 1, "Hardware Error Status"  },
    { 76, 2, 1, 1, "Velocity I Gain"        },
    { 78, 2, 1, 1, "Velocity P Gain"        },
    { 80, 2, 1, 1, "Position D Gain"        },
    { 82, 2, 1, 1, "Position I Gain"        },
    { 84, 2, 1, 1, "Position P Gain"        },
    { 88, 2, 1, 1, "Feedforward 2nd Gain"   },
    { 90, 2, 1, 1, "Feedforward 1st Gain"   },
    { 98, 1, 1, 1, "BUS Watchdog"           },
    {100, 2, 1, 1, "Goal PWM"               },
    {104, 4, 1, 1, "Goal Velocity"          },
    {108, 4, 1, 1, "Profile Acceleration"   },
    {112, 4, 1, 1, "Profile Velocity"       },
    {116, 4, 1, 1, "Goal Position"          },
    {120, 2, 0, 1, "Realtime Tick"          },
    {122, 1, 0, 1, "Moving"                 },
    {123, 1, 0, 1, "Moving Status"          },
    {124, 2, 0, 1, "Present PWM"            },
    {126, 2, 0, 1, "Present Load"           },
    {128, 4, 0, 1, "Present Velocity"       },
    {132, 4, 0, 1, "Present Position"       },
    {136, 4, 0, 1, "Velocity Trajectory"    },
    {140, 4, 0, 1, "Position Trajectory"    },
    {144, 2, 0, 1, "Present Input Voltage"  },
    {146, 1, 0, 1, "Present Temperature"    },
    {168, 2, 1, 1, "Indirect Address 1"     },
    {170, 2, 1, 1, "Indirect Address 2"     },
    {172, 2, 1, 1, "Indirect Address 3"     },
    {174, 2, 1, 1, "Indirect Address 4"     },
    {176, 2, 1, 1, "Indirect Address 5"     },
    {178, 2, 1, 1, "Indirect Address 6"     },
    {180, 2, 1, 1, "Indirect Address 7"     },
    {182, 2, 1, 1, "Indirect Address 8"     },
    {184, 2, 1, 1, "Indirect Address 9"     },
    {186, 2, 1, 1, "Indirect Address 10"    },
    {188, 2, 1, 1, "Indirect Address 11"    },
    {190, 2, 1, 1, "Indirect Address 12"    },
    {192, 2, 1, 1, "Indirect Address 13"    },
    {194, 2, 1, 1, "Indirect Address 14"    },
    {196, 2, 1, 1, "Indirect Address 15"    },
    {198, 2, 1, 1, "Indirect Address 16"    },
    {200, 2, 1, 1, "Indirect Address 17"    },
    {202, 2, 1, 1, "Indirect Address 18"    },
    {204, 2, 1, 1, "Indirect Address 19"    },
    {206, 2, 1, 1, "Indirect Address 20"    },
    {208, 2, 1, 1, "Indirect Address 21"    },
    {210, 2, 1, 1, "Indirect Address 22"    },
    {212, 2, 1, 1, "Indirect Address 23"    },
    {214, 2, 1, 1, "Indirect Address 24"    },
    {216, 2, 1, 1, "Indirect Address 25"    },
    {218, 2, 1, 1, "Indirect Address 26"    },
    {220, 2, 1, 1, "Indirect Address 27"    },
    {222, 2, 1, 1, "Indirect Address 28"    },
    {224, 1, 1, 1, "Indirect Data 1"        },
    {225, 1, 1, 1, "Indirect Data 2"        },
    {226, 1, 1, 1, "Indirect Data 3"        },
    {227, 1, 1, 1, "Indirect Data 4"        },
    {228, 1, 1, 1, "Indirect Data 5"        },
    {229, 1, 1, 1, "Indirect Data 6"        },
    {230, 1, 1, 1, "Indirect Data 7"        },
    {231, 1, 1, 1, "Indirect Data 8"        },
    {232, 1, 1, 1, "Indirect Data 9"        },
    {233, 1, 1, 1, "Indirect Data 10"       },
    {234, 1, 1, 1, "Indirect Data 11"       },
    {235, 1, 1, 1, "Indirect Data 12"       },
    {236, 1, 1, 1, "Indirect Data 13"       },
    {237, 1, 1, 1, "Indirect Data 14"       },
    {238, 1, 1, 1, "Indirect Data 15"       },
    {239, 1, 1, 1, "Indirect Data 16"       },
    {240, 1, 1, 1, "Indirect Data 17"       },
    {241, 1, 1, 1, "Indirect Data 18"       },
    {242, 1, 1, 1, "Indirect Data 19"       },
    {243, 1, 1, 1, "Indirect Data 20"       },
    {244, 1, 1, 1, "Indirect Data 21"       },
    {245, 1, 1, 1, "Indirect Data 22"       },
    {246, 1, 1, 1, "Indirect Data 23"       },
    {247, 1, 1, 1, "Indirect Data 24"       },
    {248, 1, 1, 1, "Indirect Data 25"       },
    {249, 1, 1, 1, "Indirect Data 26"       },
    {250, 1, 1, 1, "Indirect Data 27"       },
    {251, 1, 1, 1, "Indirect Data 28"       },
    {578, 2, 1, 1, "Indirect Address 29"    },
    {580, 2, 1, 1, "Indirect Address 30"    },
    {582, 2, 1, 1, "Indirect Address 31"    },
    {584, 2, 1, 1, "Indirect Address 32"    },
    {586, 2, 1, 1, "Indirect Address 33"    },
    {588, 2, 1, 1, "Indirect Address 34"    },
    {590, 2, 1, 1, "Indirect Address 35"    },
    {592, 2, 1, 1, "Indirect Address 36"    },
    {594, 2, 1, 1, "Indirect Address 37"    },
    {596, 2, 1, 1, "Indirect Address 38"    },
    {598, 2, 1, 1, "Indirect Address 39"    },
    {600, 2, 1, 1, "Indirect Address 40"    },
    {602, 2, 1, 1, "Indirect Address 41"    },
    {604, 2, 1, 1, "Indirect Address 42"    },
    {606, 2, 1, 1, "Indirect Address 43"    },
    {608, 2, 1, 1, "Indirect Address 44"    },
    {610, 2, 1, 1, "Indirect Address 45"    },
    {612, 2, 1, 1, "Indirect Address 46"    },
    {614, 2, 1, 1, "Indirect Address 47"    },
    {616, 2, 1, 1, "Indirect Address 48"    },
    {618, 2, 1, 1, "Indirect Address 49"    },
    {620, 2, 1, 1, "Indirect Address 50"    },
    {622, 2, 1, 1, "Indirect Address 51"    },
    {624, 2, 1, 1, "Indirect Address 52"    },
    {626, 2, 1, 1, "Indirect Address 53"    },
    {628, 2, 1, 1, "Indirect Address 54"    },
    {630, 2, 1, 1, "Indirect Address 55"    },
    {632, 2, 1, 1, "Indirect Address 56"    },
    {634, 1, 1, 1, "Indirect Data 29"       },
    {635, 1, 1, 1, "Indirect Data 30"       },
    {636, 1, 1, 1, "Indirect Data 31"       },
    {637, 1, 1, 1, "Indirect Data 32"       },
    {638, 1, 1, 1, "Indirect Data 33"       },
    {639, 1, 1, 1, "Indirect Data 34"       },
    {640, 1, 1, 1, "Indirect Data 35"       },
    {641, 1, 1, 1, "Indirect Data 36"       },
    {642, 1, 1, 1, "Indirect Data 37"       },
    {643, 1, 1, 1, "Indirect Data 38"       },
    {644, 1, 1, 1, "Indirect Data 39"       },
    {645, 1, 1, 1, "Indirect Data 40"       },
    {646, 1, 1, 1, "Indirect Data 41"       },
    {647, 1, 1, 1, "Indirect Data 42"       },
    {648, 1, 1, 1, "Indirect Data 43"       },
    {649, 1, 1, 1, "Indirect Data 44"       },
    {650, 1, 1, 1, "Indirect Data 45"       },
    {651, 1, 1, 1, "Indirect Data 46"       },
    {652, 1, 1, 1, "Indirect Data 47"       },
    {653, 1, 1, 1, "Indirect Data 48"       },
    {654, 1, 1, 1, "Indirect Data 49"       },
    {655, 1, 1, 1, "Indirect Data 50"       },
    {656, 1, 1, 1, "Indirect Data 51"       },
    {657, 1, 1, 1, "Indirect Data 52"       },
    {658, 1, 1, 1, "Indirect Data 53"       },
    {659, 1, 1, 1, "Indirect Data 54"       },
    {660, 1, 1, 1, "Indirect Data 55"       },
    {661, 1, 1, 1, "Indirect Data 56"       },    
};

//initialize_address_table - does what it says on the tin
//Inputs
//  table[] - an empty array of type address_t and size ADDRESS_TABLE_SIZE
//Output
//No output, but configures the address table by assigning labels to addresses that exist and the nil address to addresses that don't
void initialize_address_table( address_t table[] ){
  for( int i = 0 ; i < ADDRESS_TABLE_SIZE ; i++ ){
    table[i] = nil_address;
  }

  for( int i = 0; i < ADDRESS_INFORMATION_SIZE ; i++){
    table[address_information[i].index] = address_information[i];
  }
}

#include "dxl.h" //Rest of the low level servo library. 

//srv_write - write any single address by label
//Inputs:
//  port      - a pointer to the IO system used.                                                      (ex: &Serial or &Serial3)
//  servo_id  - the ID number of the servo being addressed                                            (ex: 1, 3, or 5. The constant 254 (BROADCAST) may be used to address all servos.)
//  label     - A string literal containing the name of the address as defined in the protocol spec.  (ex: "LED", "Velocity Goal", "Present Position")
//  value     - Signed integer value to be written to the address.                                    (ex: 391, -200, 0)
//Output:
//  A bool containing true if the tranmsission was successful (servo responds, AND CRC check was successful, AND error byte in response data is 0). 
//  If BROADCAST is used, the servos will not respond, so in this case false is returned.
bool srv_write( Stream *port, uint8_t servo_id, const char *label, int value ){
  address_t address = find_address( label );
  uint16_t data_size = address.byte_count;
  
  uint8_t tmp_array[data_size];
  int_2_data(data_size, value, tmp_array);
  
  dxl_write( port, servo_id, address.index, data_size, tmp_array );
  if( servo_id != BROADCAST ){
    uint8_t rx_pack[1][MAX_BUFFER_LENGTH];
    
    bool completed = receive_packet( port, 1, rx_pack, DXL_TIMEOUT );
    return completed;
  }else{
    return false;
  }
}

//srv_read - read any single address by label
//Inputs:
//  port      - a pointer to the IO system used.                                                      (ex: &Serial or &Serial3)
//  servo_id  - the ID number of the servo being addressed (not BROADCAST)                            (ex: 1, 3, or 5.)
//  label     - A string literal containing the name of the address as defined in the protocol spec.  (ex: "LED", "Velocity Goal", "Present Position")
//Output:
//  If the transmission is successful, an integer containing the address value. 0 otherwise. Currently no error checking exists on this value.
int srv_read( Stream *port, uint8_t servo_id, const char *label  ){
  address_t address = find_address( label );
  uint16_t data_size = address.byte_count;
  uint8_t tmp_array[data_size];

  dxl_read( port, servo_id, address.index, data_size );
  
  if( servo_id != BROADCAST ){
    uint8_t rx_array[1][MAX_BUFFER_LENGTH];
    bool transmission_completed = receive_packet( port, 1, rx_array, DXL_TIMEOUT );
    if( transmission_completed ){
      for( int i = 0 ; i < data_size ; i++ ){
        tmp_array[data_size-i-1] = rx_array[0][i+9]; //index 9 is where return data starts
      }

      return data_2_int( data_size, tmp_array );
    }
  }

  return 0;
}

//svr_assign_indirect_memory - configure one servo to use indirect memory
//  port        - a pointer to the IO system used.                                                      (ex: &Serial or &Serial3)
//  servo_id    - the ID number of the servo being addressed (not BROADCAST)                            (ex: 1, 3, or 5.)
//  indirect    - A string literal containing the address 
//                label where indirect memory access should start                                       (ex: "Indirect Address 1", "Indirect Address 3")
//  labels[]    - An array of string literals containing the addresses to be assigned to
//                indirect memory accesses, in order                                                    (ex: {"LED", "Torque Enable", "Position Goal"})
//  label_count - How many addresses are redirected                                                     (ex: 3 for the example label array)
//  table[]     - The address table                                                                     (ex: 
bool srv_assign_indirect_memory( Stream *port, uint8_t servo_id, const char *indirect, const char *labels[], uint8_t label_count, address_t table[] ){ 
  address_t ind_addr = find_address(indirect);
  int k = 0;
  for( int i = 0 ; i < label_count ; i++ ){
      address_t current_address = find_address( labels[i] );
      for( int j = 0; j < current_address.byte_count ; j++ ){
        address_t current_indirect_address = table[ind_addr.index+2*k];
        if(!srv_write( port, servo_id, current_indirect_address.label, current_address.index+j) ){
          return false;
        }
        k++;
      }
  }
  return true;
}

bool srv_sync_read( Stream *port, uint8_t servo_ids[], uint8_t id_count, const char *label, uint16_t data_size, const char *data_labels[], int data[], uint8_t data_count ){
  address_t data_start = find_address( label );
  dxl_sync_read( port, servo_ids, id_count, data_start.index, data_size );
  uint8_t rx_array[id_count][MAX_BUFFER_LENGTH];
  bool transmission_completed = receive_packet( port, id_count, rx_array, DXL_TIMEOUT );
  if( transmission_completed ){
    for( int i = 0; i < id_count ; i++ ){//Get one response from each servo
      //print_packet( &Serial, rx_array[i] );
      int w = 0;//Keeps track of how many bytes total we've read from this packet
      for( int j = 0 ; j < data_count ; j++){//Get one packet for each data label
        uint8_t address_size = find_address( data_labels[j] ).byte_count;
        uint8_t tmp_array[address_size];
        
        for( int k = 0 ; k < address_size ; k++ ){//Read one byte from the packet for each byte in the address
          tmp_array[address_size-k-1] = rx_array[i][w+9]; //index 9 is where return data starts
          w++;
        }
        
        data[i*data_count + j] = data_2_int( address_size, tmp_array );
        
      }
    }
    //Serial.println();
    return true;
  }else{
    /*for( int i = 0; i < id_count; i++){
     print_packet( &Serial, rx_array[i] );
    }*/
    return false;
  }
}

void construct_data_array( uint8_t bytes, uint8_t value_count, int values[], uint8_t data[] ){
  for( int i = 0; i < value_count; i++ ){   
    uint8_t value_bytes[bytes];
    int_2_data( bytes, values[i], value_bytes );
    for( int j = 0; j < bytes; j++ ){
      data[i*bytes + j] = value_bytes[j];
    }
  }
}

void srv_sync_write( Stream *port, uint8_t servo_ids[], uint8_t id_count, const char *label, const char *value_labels[], int values[], int value_count ){
  address_t data_start = find_address( label );

  uint16_t data_size = 0;
  for( int i = 0; i < value_count ; i++){
    data_size += find_address(value_labels[i]).byte_count;
  }

  uint8_t tmp_array[data_size*id_count]; 
  for( int i = 0 ; i < data_size*id_count ; i++ ){
    tmp_array[i] = 0;
  }
  
  construct_data_array( data_size, id_count, values, tmp_array ); 
  dxl_sync_write( port, servo_ids, id_count, data_start.index, data_size, tmp_array );
}



#endif

