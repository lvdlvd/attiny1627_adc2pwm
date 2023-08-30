/*
    adc2pwm2 for Attiny1627

    Measures AINx (Pxy) and outputs a 610Hz PWM signal with a duty cycle
    equal to Vin/Vdd.
    
   
    Theory of operation:

    - ADC is configured as follows:
        * single ended AIN6 (PA6) as +input
        * VDD as Vref
        * sample duration 48 oversample 16x
            conversion time = ( (48+14) * 16 + 1.5 ) / 5MHz = 199us. (or 248us)
        * burst mode on event
    - TCA0 is set to count to 16384 on the 10MHz clock (1.6384 ms period or 610.35Hz)
        * on the 8MHz clock this is 2.048 ms period or 488.28Hz
    - an update triggers the ADC event
    - Channel 0 is configured to PWM mode, output on PB0
    - the UART is configured to 115200 8N1, output on PB2
    - ADC IRQ handler copies the accumulated result to the TCA0 channel 0 pulse with register
      and to the serial buffer as HDLC framed 2 bytes big endian. 


    PA6 analog input signal Vin
    PB0 TCA W0 output PWM  xxHz, duty cycle = Vin/VDD
    PB2 USART0 TXD output serial 115200 8N1 hdlc encoded messages
    PB7 LED on the curiosity nano

*/
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>

static void setbaud(USART_t* usart, uint32_t bd) {
    uint32_t clk = F_CPU >> 8; // no check for validity; default will be 20MHz  17 bits
    switch (FUSE.OSCCFG & 0x3) {
    case 1: // 16MHhz mainclock
         clk = (16000000>>8) * (1024 + SIGROW.OSCCAL16M1); // should be the error at 3V  27 bits
    case 2: // 20MHz mainclock
         clk = (20000000>>8) * (1024 + SIGROW.OSCCAL20M1); // should be the error at 3V
    }

    if (CLKCTRL.MCLKCTRLB & 1) {
        clk /= ((uint8_t[16]){ 2, 4, 8, 16, 32, 64, 1, 1, 6, 10, 12, 24, 48, 1, 1, 1})[(CLKCTRL.MCLKCTRLB >> 1) & 0xf];
    }

    clk <<= 2;  // 29 bits
    clk /= bd;
    clk >>= 2; 

    usart->BAUD = (uint16_t)clk;
}

static uint8_t txbuf[16];
static uint8_t txhead = 0;
static uint8_t txtail = 0;
static inline int txbuf_empty() { return txhead == txtail; }
static inline int txbuf_full()  { return txhead == (uint8_t)(txtail + sizeof txbuf); }

enum {
        HDLC_ESC   = 0x7d, // 0b0111 1101
        HDLC_FLAG  = 0x7e, // 0b0111 1110
        HDLC_ABORT = 0x7f, // 0b0111 1111
};

static inline void put_char(uint8_t c) {
    switch (c) {
    case HDLC_ESC:
    case HDLC_FLAG:
    case HDLC_ABORT:
            c ^= 0x20;
            txbuf[txhead++ % sizeof txbuf] = HDLC_ESC;
    }
    txbuf[txhead++ % sizeof txbuf] = c;
    USART0.CTRLA |= USART_DREIE_bm; // enable TX register empty irq
}
static inline void put_flag() { txbuf[txhead++ % sizeof txbuf] = HDLC_FLAG; }

ISR(USART0_DRE_vect) {
    if (txbuf_empty()) {
        USART0.TXDATAL =  HDLC_FLAG;
    } else {
        USART0.TXDATAL = txbuf[txtail++ % sizeof txbuf];
    }
}

ISR(ADC0_RESRDY_vect) {
    // reading result clears the interrupt flag
    uint16_t val = ADC0.RESULT; 
    TCA0.SINGLE.CMP0BUF = val >> 2; // divide by 4 to get 14 bit range

    // no race because the USART0_DRE can't run while this ISR runs
    put_char(val >> 8);
    put_char(val);
    put_flag();
}

int main() {

    //  FUSE.OSCCONFIG determines if we have 20MHz or 16MHz main clock
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, 1);  // 11.5.2 mainclock div 2, for 10 or 8MHz 

    PORTA.PIN6CTRL = PORT_ISC_INPUT_DISABLE_gc; // 17.5.11 disable digital input buffer, for analog input.

    ADC0.CTRLA   = ADC_ENABLE_bm|ADC_LOWLAT_bm; // ADC0 on, keep on between triggers
    ADC0.CTRLB   = ADC_PRESC_DIV2_gc;           // 5MHz (4MHz)
    ADC0.CTRLC   = (5<<3)| ADC_REFSEL_VDD_gc;   // 5 (4) ticks is timebase for 1us, Vref = Vdd
    ADC0.CTRLE   = 48;                          // sample duration 48 cycles
    ADC0.CTRLF   = ADC_SAMPNUM_ACC16_gc;        // 16 samples (4 extra bits, 16 bits total)
    ADC0.MUXPOS  = ADC_MUXPOS_AIN6_gc;       // AINx () as positive input (single ended)
    ADC0.INTCTRL = ADC_RESRDY_bm;               // irq on result ready
    ADC0.COMMAND = ADC_MODE_BURST_gc | ADC_START_EVENT_TRIGGER_gc;  // burst mode, start on event

    EVSYS.CHANNEL0 = EVSYS_CHANNEL0_TCA0_OVF_LUNF_gc; // channel 0 = TCA overflow
    EVSYS.USERADC0START = 1;                          // ADC start on event channel 0

    TCA0.SINGLE.PER   = 0x3fff;    // count in 14 bits: 10MHz/16384 = 610Hz / 1.6384 ms, or 488Hz / 2.048ms
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm;  
    TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc;

    PORTB.DIRSET = 1<<0;          // PB0 is TCA0 WO0 signal from CMP0

//    setbaud(&USART0, 115200);     // USART0.BAUD  = 347, for 115200Bd for a 20MHz/2 F_PER
    USART0.BAUD  = 347;
    USART0.CTRLB = USART_TXEN_bm; // enable TX, add USART_ODME_bm to set open drain mode
    USART0.CTRLA = USART_DREIE_bm; // enable TX register empty irq
    PORTB.DIRSET = 1<<2;            // PB2 is TXD

    sei(); // enable interrupts

    for (;;)
        sleep_mode();

}





