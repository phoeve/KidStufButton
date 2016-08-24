//
//   KIDSTUF DMX Decoder and Function Controller
//          This program is intended to run on a Arduino microcontroller.  It receives DMX packets, decodes
//          them, and activates the desired funtion. 
//          The program replaces the Arduino interrupt handler with its own ISR(USART3_RX_vect) to 
//          process incoming packets via a state machine.
//
//
//   Supported Processors (tested)
//          - Arduino Mega
//
//   Author(s) Peter Hoeve (peter@hoeve.us)  NPCC Production Team
//
//   Based on example code from Akiba@Freaklabs.com and Arduino
//
//   History:
//         8/9/16 - 0.1 (PH) Original


#define DMX_DEBUG



#define NUM_CHANNELS 8 // Number of DMX channels to listen to

                  // pin assignments
unsigned int output_pins[NUM_CHANNELS] = {2,3,4,5,6,7,8,9};  // Stay above console (0,1)


      // DMX info/ranges
#define DMX_MIN_VALUE 0
#define DMX_MAX_VALUE 255
      // DMX packet states
enum {DMX_IDLE,  DMX_BREAK,  DMX_START,  DMX_RUN};
      // current state of DMX state machine
volatile unsigned char dmx_state;                // http://arduino.cc/en/Reference/Volatile
      // this is the start address for the dmx frame
unsigned int dmx_start_addr = 1;
      // this is the current address of the dmx frame
unsigned int dmx_addr;
      // this is used to keep track of the channels
unsigned int chan_cnt;
      // this holds the dmx data
unsigned char dmx_data[NUM_CHANNELS];

      // tell us when to update the pins
volatile unsigned char update;    


     
#define DIP_ON LOW             // If voltage is LOW (grounded), the dip switch is in the ON position.  
#define NUM_ADDRESS_BITS 9     // 2**9 gives 0-511
unsigned int address_pins[NUM_ADDRESS_BITS] = {50,48,46,44,42,40,38,36,34};
unsigned int myBaseAddress = 0;


//===============================================================================================================


void setup()
{
  int i;
  unsigned char val=0;

  // Console port
  Serial.begin(9600);
  Serial.write("setup() ...\n");  

               
               // On power up or reset read base address from dip switch 
  for (i=0; i<NUM_ADDRESS_BITS; i++)
  {
    pinMode(address_pins[i], INPUT);           // set pin to input
    digitalWrite(address_pins[i], HIGH);       // turn on pullup resistor
    
    val = digitalRead(address_pins[i]);
    Serial.print(val);
    if (val==DIP_ON)
      bitSet(myBaseAddress, NUM_ADDRESS_BITS -i -1);   
  }
  
  dmx_start_addr = myBaseAddress;    
  
  Serial.write("myBaseAddress is ");
  Serial.print(myBaseAddress);
  Serial.write("\n");
  Serial.write("dmx_start_addr is ");
  Serial.print(dmx_start_addr);
  Serial.write("\n");

        // Setup outputs that correspond to DMX channels
  for(i=0; i< NUM_CHANNELS; i++){
    pinMode(output_pins[i], OUTPUT);      // sets the digital pin as output
  }
  


    // set update flag idle
  update = 0;
  
  // set default DMX state
  dmx_state = DMX_IDLE;


  // initialize UART for DMX
  // 250 kbps, 8 bits, no parity, 2 stop bits
  UCSR3C |= (1<<USBS3);
  Serial.begin(250000);

}


void loop()
{

  switch (dmx_data[0]) {
      case 0:
        Serial.write("Invoking Look 0 (Black)\n");
        break;
      case 1:
        Serial.write("Invoking Look 1\n");
        break;
      case 2:
        Serial.write("Invoking Look 2\n");
        break;
      case 3:
        Serial.write("Invoking Look 3\n");
        break;
      case 4:
        Serial.write("Invoking Look 4\n");
        break;
      default:
        Serial.write("DMX value = ");
        Serial.print(dmx_data[0]);
        Serial.write(" - Invoking Look 255\n");
        break;
    }

}



/**************************************************************************/
/*!
  This is the interrupt service handler for the DMX
*/
/**************************************************************************/
ISR(USART3_RX_vect)
{
  unsigned char status = UCSR3A;
  unsigned char data = UDR3;
  switch (dmx_state)
  {
    case DMX_IDLE:
      if (status & (1<<FE0))
      {
        dmx_addr = 0;
        dmx_state = DMX_BREAK;
        update = 1;
      }
    break;
    
    case DMX_BREAK:
      if (data == 0)
      {
        dmx_state = DMX_START;
      }
      else
      {
        dmx_state = DMX_IDLE;
      }
    break;
    
    case DMX_START:
      
      dmx_addr++;
      if (dmx_addr == dmx_start_addr)
      {
        chan_cnt = 0;
        dmx_data[chan_cnt++] = data;
        dmx_state = DMX_RUN;
      }
    break;
    
    case DMX_RUN:
      dmx_data[chan_cnt++] = data;
      if (chan_cnt >= NUM_CHANNELS)
      {
        dmx_state = DMX_IDLE;
      }
    break;
    
    default:
      dmx_state = DMX_IDLE;
    break;
  }
}

