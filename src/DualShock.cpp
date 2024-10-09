#include <Arduino.h>

#include "DualShock.h"


static byte enter_config[]={0x01,0x43,0x00,0x01,0x00};
static byte set_mode[]={0x01,0x44,0x00,0x01,0x03,0x00,0x00,0x00,0x00};
// static byte set_bytes_large[]={0x01,0x4F,0x00,0xFF,0xFF,0x03,0x00,0x00,0x00};
static byte exit_config[]={0x01,0x43,0x00,0x00,0x5A,0x5A,0x5A,0x5A,0x5A};
static byte type_read[]={0x01,0x45,0x00,0x5A,0x5A,0x5A,0x5A,0x5A,0x5A};


void DualShock::poll() {
    readGamepad();
    if ((last_buttons ^ buttons) > 0) {
        processCB();
    }
    processStickCB();
}

int obtenerIndice(Button button) {
    // Descomposición de la búsqueda en partes
    if (button >= 0x1000) return (button >= 0x4000) ? ((button >= 0x8000) ? 15 : 14) : ((button >= 0x2000) ? 13 : 12);
    if (button >= 0x0100) return (button >= 0x0400) ? ((button >= 0x0800) ? 11 : 10) : ((button >= 0x0200) ? 9 : 8);
    if (button >= 0x0010) return (button >= 0x0040) ? ((button >= 0x0080) ? 7 : 6) : ((button >= 0x0020) ? 5 : 4);
    return (button >= 0x0004) ? ((button >= 0x0008) ? 3 : 2) : ((button >= 0x0002) ? 1 : 0);
}

void DualShock::setButtonEventCB(Button button, ButtonEventCB cb) {
    int index = obtenerIndice(button);
    this->cb_events[index] = cb;
}

void DualShock::setStickEventCB(StickButton button, StickEventCB cb) {
    if(button == StickButtonLeft) 
        this->cb_left = cb;
    else if(button == StickButtonRight)
        this->cb_right = cb;
}

DualShock::DualShock(uint8_t pin_clk, uint8_t pin_cmd, uint8_t pin_att, uint8_t pin_dat) {
    for(int i=0;i < MaxButtons; i++) {
        this->cb_events[i] = nullptr;
    }
    this->cb_left = nullptr;
    this->cb_right = nullptr;
    this->pin_clk = pin_clk;
    this->pin_cmd = pin_cmd;
    this->pin_att = pin_att;
    this->pin_dat = pin_dat;
    clk_mask = (port_mask_t) digitalPinToBitMask(this->pin_clk);
    clk_oreg = (port_reg_t*) portOutputRegister(digitalPinToPort(this->pin_clk));
    cmd_mask = (port_mask_t) digitalPinToBitMask(this->pin_cmd);
    cmd_oreg = (port_reg_t*) portOutputRegister(digitalPinToPort(this->pin_cmd));
    att_mask = (port_mask_t) digitalPinToBitMask(this->pin_att);
    att_oreg = (port_reg_t*) portOutputRegister(digitalPinToPort(this->pin_att));
    dat_mask = (port_mask_t) digitalPinToBitMask(this->pin_dat);
    dat_ireg = (port_reg_t*) portInputRegister(digitalPinToPort(this->pin_dat));

    pinMode(this->pin_clk, OUTPUT); //configure ports
    pinMode(this->pin_att, OUTPUT); ATT_SET();
    pinMode(this->pin_cmd, OUTPUT);
    pinMode(this->pin_dat, INPUT_PULLUP); // enable pull-up

    // CMD_SET(); // SET(*_cmd_oreg,_cmd_mask);
    CLK_SET();

    configureStub();
}

/////////////////////////////
void DualShock::processStickCB() {
    if(this->cb_left != nullptr) {
        if(PS2data[StickLeftX] != 128 || PS2data[StickLeftY] != 127) {
            this->cb_left(PS2data[StickLeftX], PS2data[StickLeftY]);
            last_stick_buttons |= StickButtonLeft;
        } else if(last_stick_buttons & StickButtonLeft) {
            this->cb_left(PS2data[StickLeftX], PS2data[StickLeftY]);
            last_stick_buttons ^= StickButtonLeft;
        }
    }
    if(this->cb_right != nullptr) {
        if(PS2data[StickRightX] != 128 || PS2data[StickRightY] != 127) {
            this->cb_right(PS2data[StickRightX], PS2data[StickRightY]);
            last_stick_buttons |= StickButtonRight;
        } else if(last_stick_buttons & StickButtonRight) {
            this->cb_right(PS2data[StickRightX], PS2data[StickRightY]);
            last_stick_buttons ^= StickButtonRight;
        }
    }
}

void DualShock::processCB() {
    for(int i = 0; i < 16; i++) {
        int button = (1<<i);
        if((((last_buttons ^ buttons) & button) > 0) & ((~buttons & button) > 0)) {
            if (this->cb_events[i] != nullptr)
                this->cb_events[i](ButtonEventPressed);
        }
        if((((last_buttons ^ buttons) & button) > 0) & ((~last_buttons & button) > 0)) {
            if (this->cb_events[i] != nullptr)
                this->cb_events[i](ButtonEventRelased);            
        }
    }
}

byte DualShock::configureStub() {
    byte temp[sizeof(type_read)];

    //new error checking. First, read gamepad a few times to see if it's talking
    readGamepad();
    readGamepad();

    //see if it talked - see if mode came back. 
    //If still anything but 41, 73 or 79, then it's not talking
    if(PS2data[1] != 0x41 && PS2data[1] != 0x42 && PS2data[1] != 0x73 && PS2data[1] != 0x79){ 
        return 1; //return error code 1
    }

    //try setting mode, increasing delays if need be.
    read_delay = 1;

    t_last_att = millis() + CTRL_PACKET_DELAY; // start reading right away

    for(int y = 0; y <= 10; y++) {
        sendCommandString(enter_config, sizeof(enter_config)); //start config run

        //read type
        delayMicroseconds(CTRL_BYTE_DELAY);

        //CLK_SET(); // CLK should've been set to HIGH already
        BEGIN_SPI();


        for (int i = 0; i<9; i++) {
            temp[i] = gamepadShiftinout(type_read[i]);
        }

        END_SPI();


        controller_type = temp[3];

        sendCommandString(set_mode, sizeof(set_mode));
        sendCommandString(exit_config, sizeof(exit_config));

        readGamepad();

        if(PS2data[1] == 0x73)
            break;

        if(y == 10){
            return 2; //exit function with error
        }
        read_delay += 1; //add 1ms to read_delay
    }
    return 0; //no error if here
}


boolean DualShock::readGamepad() {
    double temp = millis() - last_read;

    if (temp > 1500) //waited to long
        reconfigure();

    if(temp < read_delay)  //waited too short
        delay(read_delay - temp);

    byte dword[9] = {0x01,0x42,0,false,0x00,0,0,0,0};
    byte dword2[12] = {0,0,0,0,0,0,0,0,0,0,0,0};

    // Try a few times to get valid data...
    for (byte RetryCnt = 0; RetryCnt < 5; RetryCnt++) {
        BEGIN_SPI();
        //Send the command to send button and joystick data;
        for (int i = 0; i<9; i++) {
            PS2data[i] = gamepadShiftinout(dword[i]);
        } 

        if(PS2data[1] == 0x79) {  //if controller is in full data return mode, get the rest of data
            for (int i = 0; i<12; i++) {
                PS2data[i+9] = gamepadShiftinout(dword2[i]);
            }
        }

        END_SPI();

        // Check to see if we received valid data or not.  
        // We should be in analog mode for our data to be valid (analog == 0x7_)
        if ((PS2data[1] & 0xf0) == 0x70)
            break;

        // If we got to here, we are not in analog mode, try to recover...
        reconfigure(); // try to get back into Analog mode.
        delay(read_delay);
    }

    // If we get here and still not in analog mode (=0x7_), try increasing the read_delay...
    if ((PS2data[1] & 0xf0) != 0x70) {
        if (read_delay < 10)
            read_delay++;   // see if this helps out...
    }

    last_buttons = buttons; //store the previous buttons states
#if defined(__AVR__)
    buttons = *(uint16_t*)(PS2data+3);   //store as one value for multiple functions
#else
    buttons =  (uint16_t)(PS2data[4] << 8) + PS2data[3];   //store as one value for multiple functions
#endif
    last_read = millis();
   
    return ((PS2data[1] & 0xf0) == 0x70);  // 1 = OK = analog mode - 0 = NOK
}

void DualShock::sendCommandString(byte string[], byte len) {
    BEGIN_SPI();
    for (int y=0; y < len; y++)
        gamepadShiftinout(string[y]);
    END_SPI();

    delay(read_delay);                  //wait a few
}

void DualShock::reconfigure(){
    sendCommandString(enter_config, sizeof(enter_config));
    sendCommandString(set_mode, sizeof(set_mode));
    sendCommandString(exit_config, sizeof(exit_config));
}

unsigned char DualShock::gamepadShiftinout(char byte) {
  if(_spi == NULL) {
    /* software SPI */
    unsigned char tmp = 0;

   for(unsigned char i=0;i<8;i++) {
      if(CHK(byte,i)) CMD_SET();
      else CMD_CLR();
	  
      CLK_CLR();
      delayMicroseconds(CTRL_CLK);

      //if(DAT_CHK()) SET(tmp,i);
      if(DAT_CHK()) bitSet(tmp,i);

      CLK_SET();
      delayMicroseconds(CTRL_CLK);

   }
   CMD_SET();
   delayMicroseconds(CTRL_BYTE_DELAY);
   return tmp;
  } else {
    unsigned char tmp = _spi->transfer(byte); // hardware SPI
    delayMicroseconds(CTRL_BYTE_DELAY);
    return tmp;
  }

}

inline void DualShock::CLK_SET(void) {
    register uint8_t old_sreg = SREG;
    cli();
    *clk_oreg |= clk_mask;
    SREG = old_sreg;
}

inline void DualShock::CLK_CLR(void) {
    register uint8_t old_sreg = SREG;
    cli();
    *clk_oreg &= ~clk_mask;
    SREG = old_sreg;
}

inline void  DualShock::CMD_SET(void) {
    register uint8_t old_sreg = SREG;
    cli();
    *cmd_oreg |= cmd_mask; // SET(*_cmd_oreg,_cmd_mask);
    SREG = old_sreg;
}

inline void  DualShock::CMD_CLR(void) {
    register uint8_t old_sreg = SREG;
    cli();
    *cmd_oreg &= ~cmd_mask; // SET(*_cmd_oreg,_cmd_mask);
    SREG = old_sreg;
}

inline void  DualShock::ATT_SET(void) {
  register uint8_t old_sreg = SREG;
  cli();
  *att_oreg |= att_mask ;
  SREG = old_sreg;
}

inline void DualShock::ATT_CLR(void) {
  register uint8_t old_sreg = SREG;
  cli();
  *att_oreg &= ~att_mask;
  SREG = old_sreg;
}

inline bool DualShock::DAT_CHK(void) {
  return (*dat_ireg & dat_mask) ? true : false;
}


inline void DualShock::BEGIN_SPI_NOATT(void) {
  if(_spi != NULL) {
#if defined(SPI_HAS_TRANSACTION)
    _spi->beginTransaction(_spi_settings);
#else
    // _spi->begin();
    _spi->setBitOrder(LSBFIRST);
    _spi->setDataMode(SPI_MODE2);
#if defined(__AVR__)
    _spi->setClockDivider(CTRL_DIVIDER);
#elif defined(__SAM3X8E__)
    _spi->setClockDivider(F_CPU / CTRL_BITRATE);
#else
    #error Unsupported method of setting clock divider without transaction, please update this library to support this platform, update the platform code to support SPI transaction, or use software SPI.
#endif
#endif
  } else {
    CMD_CLR();
    CLK_SET();
  }
}

inline void DualShock::BEGIN_SPI(void) {
  BEGIN_SPI_NOATT();
  while(millis() - t_last_att < CTRL_PACKET_DELAY);
  ATT_CLR(); // low enable joystick
  delayMicroseconds(CTRL_BYTE_DELAY);
}

inline void DualShock::END_SPI_NOATT(void) {
  if(_spi != NULL) {
#if defined(SPI_HAS_TRANSACTION)
    _spi->endTransaction();
#else
    // _spi->end();
#endif
  } else {
    CMD_CLR();
    CLK_SET();
  }
}

inline void DualShock::END_SPI(void) {
  ATT_SET();
  END_SPI_NOATT();
  t_last_att = millis();
}
