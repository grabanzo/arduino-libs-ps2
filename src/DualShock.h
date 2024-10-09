#ifndef DUALSHOCK_H
#define DUALSHOCH_H

#include <Arduino.h>
#include <SPI.h>


#if defined(__AVR__)
typedef volatile uint8_t port_reg_t;
typedef uint8_t port_mask_t;
#endif

/* SPI timing configuration */
#define CTRL_BITRATE        250000UL // SPI bitrate (Hz). Please note that on AVR Arduinos, the lowest bitrate possible is 125kHz.
#if (1000000UL / (2 * CTRL_BITRATE) > 0)
#define CTRL_CLK      (1000000UL / (2 * CTRL_BITRATE)) // delay duration between SCK high and low
#else
#define CTRL_CLK      1
#endif
#define CTRL_BYTE_DELAY     10 // delay duration between byte reads (uS)
#define CTRL_PACKET_DELAY   16 // delay duration between packets (mS) - according to playstation.txt this should be set to 16mS, but it seems that it can go down to 4mS without problems

#define SET(x,y) (x|=(1<<y))
#define CLR(x,y) (x&=(~(1<<y)))
#define CHK(x,y) (x & (1<<y))
#define TOG(x,y) (x^=(1<<y))



enum ButtonEvent {
    ButtonEventPressed = 0,
    ButtonEventRelased = 1
};

enum Button {
    ButtonSelect     = 0x0001,
    ButtonL3         = 0x0002,
    ButtonR3         = 0x0004,
    ButtonStart      = 0x0008,
    ButtonPadUp      = 0x0010,
    ButtonPadRight   = 0x0020,
    ButtonPadDown    = 0x0040,
    ButtonPadLeft    = 0x0080,
    ButtonL2         = 0x0100,
    ButtonR2         = 0x0200,
    ButtonL1         = 0x0400,
    ButtonR1         = 0x0800,
    ButtonTriangle   = 0x1000,
    ButtonCircle     = 0x2000,
    ButtonCross      = 0x4000,
    ButtonSquare     = 0x8000
};
const int MaxButtons = 16;

enum StickButton {
    StickButtonLeft  = 0x1,
    StickButtonRight = 0x2
};

enum StickValues {
    //These are stick values
    StickRightX     = 5,
    StickRightY     = 6,
    StickLeftX      = 7,
    StickLeftY      = 8
};

typedef void (*ButtonEventCB)(ButtonEvent button_event);
typedef void (*StickEventCB)(byte value_x, byte value_y);


class DualShock {
public:
    DualShock(uint8_t pin_clk, uint8_t pin_cmd, uint8_t pin_att, uint8_t pin_dat);

    void setButtonEventCB(Button button, ButtonEventCB cb);
    void setStickEventCB(StickButton button, StickEventCB cb);

    void poll();
    // byte Analog(StickButton button) { return PS2data[button]; }

private:
    // FUNCTIONS
    bool readGamepad();
    void processCB();
    void processStickCB();
    byte configureStub();
    void reconfigure();
    void sendCommandString(byte string[], byte len);
    unsigned char gamepadShiftinout(char byte);


    inline void CLK_SET(void);
    inline void CLK_CLR(void);
    inline void CMD_SET(void);
    inline void CMD_CLR(void);
    inline void ATT_SET(void);
    inline void ATT_CLR(void);
    inline bool DAT_CHK(void);
    inline void BEGIN_SPI_NOATT(void);
    inline void END_SPI_NOATT(void);
    inline void BEGIN_SPI(void);
    inline void END_SPI(void);

    // VARS
    ButtonEventCB cb_events[16];
    StickEventCB cb_left;
    StickEventCB cb_right;

    unsigned char PS2data[21];
    unsigned int last_buttons;
    unsigned int buttons;
    unsigned long last_read;
    unsigned int last_stick_buttons;
    byte read_delay;
    byte controller_type;
    volatile unsigned long t_last_att; // time since last ATT inactive

    /* SPI configuration */
    SPIClass* _spi; // hardware SPI class (null = software SPI)
    #if defined(SPI_HAS_TRANSACTION)
      SPISettings _spi_settings; // hardware SPI transaction settings
    #endif

    uint8_t pin_clk;
    uint8_t pin_cmd;
    uint8_t pin_att;
    uint8_t pin_dat;

    port_mask_t clk_mask; 
    port_reg_t *clk_oreg;
    port_mask_t cmd_mask; 
    port_reg_t *cmd_oreg;
    port_mask_t att_mask; 
    port_reg_t *att_oreg;
    port_mask_t dat_mask; 
    port_reg_t *dat_ireg;
};

#endif // DUALSHOCK_H