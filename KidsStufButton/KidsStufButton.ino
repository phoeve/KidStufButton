#include <Adafruit_NeoPixel.h>



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
#include <Adafruit_NeoPixel.h>

#define PIN 8    // Neopixel data pin

Adafruit_NeoPixel strip = Adafruit_NeoPixel(48, PIN, NEO_GRB + NEO_KHZ800);

#define DMX_DEBUG


#define NUM_CHANNELS 1 // Number of DMX channels to listen to


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
//unsigned int address_pins[NUM_ADDRESS_BITS] = {48,46,44,42,40,38,36,34,32};
//unsigned int address_pins[NUM_ADDRESS_BITS] = {49,47,45,43,41,40,38,36,34};
unsigned int address_pins[NUM_ADDRESS_BITS] = {50,48,46,44,42,40,38,36,34};
//unsigned int address_pins[NUM_ADDRESS_BITS] = {51,49,47,45,43,41,39,37,35};
//unsigned int address_pins[NUM_ADDRESS_BITS] = {52,50,48,46,44,42,40,38,36};
//unsigned int address_pins[NUM_ADDRESS_BITS] = {53,51,49,47,45,43,41,39,37};
unsigned int myBaseAddress = 0;

boolean dmx_unchanged = true;


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


    // set update flag idle
  update = 0;
  
  // set default DMX state
  dmx_state = DMX_IDLE;


  // initialize UART for DMX
  // 250 kbps, 8 bits, no parity, 2 stop bits
  UCSR3C |= (1<<USBS3);
  Serial3.begin(250000);

  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

}
//        (strip.Color(255, 0, 0), 50); // Vinny (Red)
//        (strip.Color(255, 50, 0), 50); // Waldo (Orange)
//        (strip.Color(205, 105, 0), 10); // Dr Von TakeOut (Yellow)
//        (strip.Color(0, 255, 0), 10); // Klubhouse (Green)
//        (strip.Color(0, 0, 255), 50); // Gordo (Blue)
//        (strip.Color(255, 10, 100), 50); // Cammie (Pink)
//        (strip.Color(255, 0, 255), 50); // Sam (Purple)
//        (strip.Color(0, 0, 0), 50); // Black
//        (strip.Color(255, 255, 255), 50); // White


void loop()
{
    Serial.print(dmx_data[0]);
    Serial.write("\n");

  dmx_unchanged = true;

  switch (dmx_data[0]) {
      case 0:
//        Serial.write("Invoking Look 0 (Black)\n");
        break;
        
      case 1:

        Serial.write("Look 1: White Chase");
        theaterChase(strip.Color(255, 255, 255), 50); // White
        break;
        
      case 2:
        Serial.write("Look 2: White Solid");
        colorWipe(strip.Color(255, 255, 255), 30); // White 50 ms
        break;
        
      case 3:

        Serial.write("Look 3: Rainbow");
        rainbow(10);
        break;
        
      case 4:
        Serial.write("Look 4: RainbowCycle");
        rainbowCycle(10);
        break;
        
      case 5:
        Serial.write("Look 5: Rainbow Chase");
        theaterChaseRainbow(10);
        break;
        
      case 11:
        Serial.write("Look 11: Red Chase");
        theaterChase(strip.Color(255, 0, 0), 50); // Vinny (Red)
        break;
        
      case 12:
        Serial.write("Look 12: Red Solid");
        colorWipe(strip.Color(255, 0, 0), 10); // Vinny (Red)
        break;

      case 21:
        Serial.write("Look 21: Orange Chase");
        theaterChase(strip.Color(255, 50, 0), 50); // Waldo (Orange)
        break;
        
      case 22:
        Serial.write("Look 22: Orange Solid");
        colorWipe(strip.Color(255, 50, 0), 10); // Waldo (Orange)
        break;

      case 31:
        Serial.write("Look 31: Yellow Chase");
        theaterChase(strip.Color(205, 105, 0), 50); // Dr Von TakeOut (Yellow)
        break;
        
      case 32:
        Serial.write("Look 32: Yellow Solid");
        colorWipe(strip.Color(205, 105, 0), 10); // Dr Von TakeOut (Yellow)
        break;

      case 41:
        Serial.write("Look 41: Green Chase");
        theaterChase(strip.Color(0, 255, 0), 50); // Klubhouse (Green)
        break;
        
      case 42:
        Serial.write("Look 42: Green Solid");
        colorWipe(strip.Color(0, 255, 0), 10); // Klubhouse (Green)
        break;

      case 51:
        Serial.write("Look 51: Blue Chase");
        theaterChase(strip.Color(0, 0, 255), 50); // Gordo (Blue)
        break;
        
      case 52:
        Serial.write("Look 52: Blue Solid");
        colorWipe(strip.Color(0, 0, 255), 10); // Gordo (Blue)
        break;

      case 61:
        Serial.write("Look 61: Pink Chase");
        theaterChase(strip.Color(255, 10, 100), 50); // Cammie (Pink)
        break;
        
      case 62:
        Serial.write("Look 62: Pink Solid");
        colorWipe(strip.Color(255, 10, 100), 10); // Cammie (Pink)
        break;

      case 71:
        Serial.write("Look 71: Purple Chase");
        theaterChase(strip.Color(255, 0, 255), 50); // Sam (Purple)
        break;
        
      case 72:
        Serial.write("Look 72: Purple Solid");
        colorWipe(strip.Color(255, 0, 255), 10); // Sam (Purple)
        break;

      default:
        Serial.write("DMX value = ");
        Serial.print(dmx_data[0]);
        Serial.write(" - Invoking Look 255\n");
        break;
    }

}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels() && dmx_unchanged; i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256 && dmx_unchanged; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5 && dmx_unchanged; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
  for (int j=0; j<10 && dmx_unchanged; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3 && dmx_unchanged; q++) {
      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, c);    //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  for (int j=0; j < 256 && dmx_unchanged; j++) {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3 && dmx_unchanged; q++) {
      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
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
        
        if (dmx_data[chan_cnt] != data)       // Allow code to know when changed data occurs
          dmx_unchanged = false;
          
        dmx_data[chan_cnt++] = data;
        dmx_state = DMX_RUN;
      }
    break;
    
    case DMX_RUN:
      if (dmx_data[chan_cnt] != data)       // Allow code to know when changed data occurs
        dmx_unchanged = false;
            
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

