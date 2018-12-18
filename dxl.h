#ifndef dxl_h
#define dxl_h

//This file contains protocol level functions for the servo drivers. For the most part, these should be left alone.

//Program constansts - change these by overriding them in srv.h instead of here
#ifndef MAX_BUFFER_LENGTH
  #define MAX_BUFFER_LENGTH 255
#endif
#ifndef DXL_TIMEOUT
  #define DXL_TIMEOUT 5000
#endif

//Protocol constants - these should not be changed
#define MIN_BUFFER_LENGTH 10
#define LENGTH_POS 5
#define ERRORED 8
#define DATA_POS 9
#ifndef BROADCAST
  #define BROADCAST 254
#endif

#ifndef srv_h
#define ADDRESS_TABLE_SIZE 662
#define ADDRESS_INFORMATION_SIZE 162

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

void initialize_address_table( address_t table[] ){
  for( int i = 0 ; i < ADDRESS_TABLE_SIZE ; i++ ){
    table[i] = nil_address;
  }

  for( int i = 0; i < ADDRESS_INFORMATION_SIZE ; i++){
    table[address_information[i].index] = address_information[i];
  }
}

#endif

void print_address_table( Stream *port, address_t table[] ){
  port->print( "index\tsize\twrite_permission\tin_ram\tlabel\n" );
  for( int i = 0 ; i < ADDRESS_TABLE_SIZE ; i++ ){
    if( table[i].byte_count > 0 ){
        port->print( table[i].index ); port->print( '\t' );
        port->print( table[i].byte_count ); port->print( '\t' );
        port->print( table[i].write_permission ); port->print( "\t\t\t" );
        port->print( table[i].in_ram ); port->print( '\t' );
        port->print( table[i].label ); port->println();
    }
  }
}

address_t find_address( const char *label ){
  for( int i = 0 ; i < ADDRESS_INFORMATION_SIZE ; i++ ){
    if( label == address_information[i].label ){
      return address_information[i]; 
    }
  }
  return nil_address;
}

void int_2_data( uint8_t bytes, int value, uint8_t data[] ){
  for( int i = 0; i < bytes; i++){
    data[i] = value & 0x000000FF;
    value = value >> 8;
  }
}

int data_2_int( uint8_t bytes, uint8_t data[] ){
  int value = 0;
  
  for( int i = 0 ; i < bytes ; i++ ){
    value = value << 8;
    value = value | data[i];
  }
  return value;
}

const uint16_t crc_table[256] = {
  0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
  0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
  0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
  0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
  0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
  0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
  0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
  0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
  0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
  0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
  0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
  0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
  0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
  0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
  0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
  0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
  0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
  0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
  0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
  0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
  0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
  0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
  0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
  0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
  0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
  0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
  0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 
  0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
  0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
  0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
  0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
  0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
};
uint16_t calculate_crc( uint8_t data[], size_t data_size ){
   size_t i, j, crc;
   crc = 0;
   for( j = 0; j < data_size; j++ ){
    i = ( (uint16_t)( crc >> 8 )^( data[j] ) )&0xFF;
    crc = ( crc << 8 )^crc_table[i];
   }

   return crc;
}

uint8_t low_bits( uint16_t number ){
  return (uint8_t)( number & 0x00FF );
}

uint8_t high_bits( uint16_t number ){
  return (uint8_t)( number >> 8 );
}

void construct_packet( uint8_t servo_id, uint8_t op_code, uint8_t parameters[], uint16_t param_count, uint8_t packet_buffer[] ){
  uint16_t pack_length = param_count + 3;
  
  //Load static elements (defined in the protocol spec)
  packet_buffer[0] = 0xFF;
  packet_buffer[1] = 0xFF;
  packet_buffer[2] = 0xFD;
  packet_buffer[3] = 0x00;
  packet_buffer[4] = servo_id;
  packet_buffer[5] = low_bits( pack_length );
  packet_buffer[6] = high_bits( pack_length );
  packet_buffer[7] = op_code;

  //Load instruction parameters
  for( int i = 0; i < param_count; i++ ){
    packet_buffer[8+i] = parameters[i];
  } 

  //Calculate and load checksum
  uint16_t crc = calculate_crc( packet_buffer, 8+param_count );
  
  packet_buffer[8+param_count] = low_bits( crc );
  packet_buffer[9+param_count] = high_bits( crc );
}

uint16_t get_packet_length( uint8_t packet_buffer[] ){
  uint16_t pack_length;
  pack_length = packet_buffer[6] << 8;           //Get the high bits of the length value, shift into high registers
  pack_length = pack_length |  packet_buffer[5]; //OR with low bits to get total
  return pack_length+7;
}

void transmit_packet( Stream *port, uint8_t packet[] ){
  uint16_t pack_size = get_packet_length( packet );
  while( port->available() ){ port->read(); }
  //Write to servo
  port->write( packet, pack_size );
  port->flush();
}

bool receive_packet( Stream *port, uint8_t pack_count, uint8_t packs[][MAX_BUFFER_LENGTH], size_t timeout ){
  size_t start_time = micros();
  uint8_t packs_received = 0;
  
  while( micros() - start_time < timeout && packs_received < pack_count ){
    uint16_t i = 0;
    uint16_t pack_length = MIN_BUFFER_LENGTH;
    while( micros() - start_time < timeout && i < pack_length ){//If transmission complete or timeout, break
      if( port->available() > 0 ){
        packs[packs_received][i] = port->read();
        if( i == LENGTH_POS + 1 ){//Both length bytes have been transferred
         pack_length = get_packet_length( packs[packs_received] );
        }
        
        switch( i ){
          case 0:
            if(packs[packs_received][i] == 0xFF){
              i++;
              
            }else{
              i = 0;
            }
            break;

          case 1:
            if(packs[packs_received][i] == 0xFF){
              i++;
            }else{
              i = 0;
            }
            break;

          case 2:
            if(packs[packs_received][i] == 0xFD){
              i++;
            }else{
              i = 0;
            }
            break;

          case 3:
            if(packs[packs_received][i] == 0x00){
              i++;
            }else{
              i = 0;
            }
            break;

          default:
            i++;
            break;
        }
      }
    }
    uint16_t data_length = pack_length-2;
    uint16_t crc = calculate_crc(packs[packs_received], data_length);
    //Serial.print(packs_received);Serial.print(" ");Serial.println(crc, HEX);
    bool success = (low_bits(crc) == packs[packs_received][data_length]) && (high_bits(crc) == packs[packs_received][data_length+1]);
    if( success && !packs[packs_received][ERRORED] ){
      int k = 3;
      int j = 3;
      for( j = 3 ; j < data_length ; j++ ){
        packs[packs_received][k] = packs[packs_received][j];
        if( !(packs[packs_received][j] == 0xFD && packs[packs_received][j-1] == 0xFD && packs[packs_received][j-2] == 0xFF && packs[packs_received][j-3] == 0xFF) ){
          k++;
        }
      }
      packs[packs_received][LENGTH_POS] = packs[packs_received][LENGTH_POS] + (k-j);
      packs_received++;
    }else{
      return false;
    }
  }
  return true;
}

void print_packet( Stream *port, uint8_t packet[] ){
  for(int i=0; i < get_packet_length( packet ); i++){
    if(packet[i] < 16){
      port->print("0");
    }
    port->print(packet[i], HEX);port->print(" ");
  } port->println();
}

void dxl_ping( Stream *port, uint8_t servo_id ){
  uint8_t tx_pack[MIN_BUFFER_LENGTH];
  construct_packet( servo_id, 0x01, 0, 0, tx_pack );
  transmit_packet( port, tx_pack );
}

void dxl_read( Stream *port, uint8_t servo_id, uint16_t address, uint16_t bytes ){
  uint8_t params[] = { low_bits(address), high_bits(address), low_bits(bytes), high_bits(bytes) };
  
  uint16_t param_count = sizeof(params) / sizeof(uint8_t);
  uint8_t tx_pack[param_count + 9];
  
  construct_packet( servo_id, 0x02, params, param_count, tx_pack );
  transmit_packet( port, tx_pack );
}

void dxl_write(Stream *port, uint8_t servo_id, uint16_t address, uint16_t bytes, uint8_t data[] ){
  uint16_t param_count = bytes+2;
  uint8_t params[param_count];

  if(bytes > 2){
    params[0] = low_bits(address);
    params[1] = high_bits(address);
    for(uint8_t i=0; i<bytes; i++){
      params[2+i] = data[(bytes-1)-i];
    }
  }else{
    params[0] = low_bits(address);
    params[1] = high_bits(address);
    for(uint8_t i=0; i<bytes; i++){
      params[2+i] = data[i];
    }
  }

  uint8_t tx_pack[param_count + 9];
  construct_packet( servo_id, 0x03, params, param_count, tx_pack );
  transmit_packet( port, tx_pack );
}

void dxl_reg_write( Stream *port, uint8_t servo_id, uint16_t address, uint16_t bytes, uint8_t data[] ){
  uint16_t param_count = bytes+2;
  uint8_t params[param_count];

  if(bytes > 2){
    params[0] = low_bits(address);
    params[1] = high_bits(address);
    for(uint8_t i=0; i<bytes; i++){
      params[2+i] = data[(bytes-1)-i];
    }
  }else{
    params[0] = low_bits(address);
    params[1] = high_bits(address);
    for(uint8_t i=0; i<bytes; i++){
      params[2+i] = data[i];
    }
  }

  uint8_t tx_pack[param_count + 9];
  construct_packet( servo_id, 0x04, params, param_count, tx_pack );
  transmit_packet( port, tx_pack );
}
void dxl_action( Stream *port, uint8_t servo_id ){
  uint8_t tx_pack[MIN_BUFFER_LENGTH];
  construct_packet( servo_id, 5, 0, 0, tx_pack );
  transmit_packet( port, tx_pack );
}

void dxl_sync_write( Stream *port, uint8_t servo_ids[], uint8_t id_count, uint16_t address, uint16_t bytes, uint8_t data[] ){
  uint16_t param_count = 4 + id_count + id_count*bytes;
  uint8_t params[param_count];

  params[0] = low_bits(address);
  params[1] = high_bits(address);
  params[2] = low_bits(bytes);
  params[3] = high_bits(bytes);

  for( uint8_t i = 0; i<id_count; i++){
    params[4+i*(bytes+1)] = servo_ids[i];
  }

  for( uint8_t i = 0; i<id_count; i++){
    for( uint8_t j = 0; j<bytes; j++){
      if (bytes<2){
        params[5+(bytes+1)*i+j] = data[(bytes)*(i+1)-j-1];
      }else{
        params[5+(bytes+1)*i+j] = data[bytes*i+j];
      }
    }
  }

  uint8_t tx_pack[param_count+9];
  construct_packet( BROADCAST, 0x83, params, param_count, tx_pack );
  transmit_packet(port, tx_pack);
}

void dxl_sync_read( Stream *port, uint8_t servo_ids[], uint8_t id_count, uint16_t address, uint16_t bytes ){
    uint16_t param_count = 4+id_count;

    uint8_t params[param_count];
    
    params[0] = low_bits(address);
    params[1] = high_bits(address);
    params[2] = low_bits(bytes);
    params[3] = high_bits(bytes);

    for( uint8_t i = 0; i<id_count; i++){
      params[4+i] = servo_ids[i];
    }

    uint8_t tx_pack[param_count+9];
    construct_packet( BROADCAST, 0x82, params, param_count, tx_pack );
    transmit_packet(port, tx_pack);
}

#endif

