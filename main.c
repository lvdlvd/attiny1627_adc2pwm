/*
    adc2pwm for Attiny1627

    Measures VDD and outputs a 610Hz PWM signal with a duty cycle
    equal to 1 - (2.5V / VDD) \in [ 0% ... 50% ] for VDD \in [ 2.5V ... 5V ].
    That means VDD can be recovered as 2.5V / (1-PWM).
    
    d(PWM)/d(VDD) = 2.5 / VDD^2 \in 0.4 .. 0.1 / V  or ~ 1.6 clock ticks of TCA0 per mv

    For Vref = 2.048V,   2.5V gives a PWM of 18.4%, 5V gives 59.2%, ~1.3 tics per mv

   
    Theory of operation:

    - VREF is set to 2.500 V, valid for Vdd = 3.0...5.5V, ±3% 0..85° , ±5% -45..125°
    - AC is configured to generate 2.500V as DACREF0 
        * NOTE: doc says output is 255/256 * 2.5V = 2.490V
        * apparently it's not required to switch the AC0 on.
    - ADC is configured as follows:
        * single ended DACREF0 as +input
        * VDD as Vref
        * sample duration 48 oversample 16x
            conversion time = ( (48+14) * 16 + 1.5 ) / 5MHz = 199us. (or 248us)
        * burst mode on event
    - TCA0 is set to count to 16384 on the 10MHz clock (1.6384 ms period or 610.35Hz)
        * on the 8MHz clock this is 2.048 ms period or 488.28Hz
    - an update triggers the ADC event
    - Channel 0 is configured to PWM mode, inverted output

    - ADC IRQ handler copies the accumulated result to the TCA0 channel 0 pulse with register

    * TODO
        - set FUSE.OSCCOFNIG to 16Mhz base clock
        - (maybe) set Brown-out detector fuses to 2.6V, VLM to 1.15% = 2.99V, enable VLM irq on crossing 
                    -> Vdd below 3.0V, set Vref to 2.048 multiply adc result with 2500/2048
        - deep sleep, switch on on external pulse on uart RX

    PB7 is LED on the curiosity nano

*/
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>

static void setbaud(USART_t* usart, uint32_t bd) {
    uint32_t clk = F_CPU; // no check for validity; default will be 20MHz
    switch (FUSE.OSCCFG & 0x3) {
    case 1: // 16MHhz mainclock
         clk = 16000000 * (1024 + SIGROW.OSCCAL16M0); // should be the error at 3V
    case 2: // 20MHz mainclock
         clk = 20000000 * (1024 + SIGROW.OSCCAL20M0); // should be the error at 3V
    }

    if (CLKCTRL.MCLKCTRLB & 1) {
        clk /= ((uint8_t[16]){ 2, 4, 8, 16, 32, 64, 1, 1, 6, 10, 12, 24, 48, 1, 1, 1})[(CLKCTRL.MCLKCTRLB >> 1) & 0xf];
    }

    clk *= 4;
    clk /= bd;
    clk >>= 10;

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
}
static inline void put_flag() { txbuf[txhead++ % sizeof txbuf] = HDLC_FLAG; }

ISR(USART0_DRE_vect) {
    if (txbuf_empty()) {
        USART0.TXDATAL = HDLC_FLAG;
    } else {
        USART0.TXDATAL = txbuf[txtail++ % sizeof txbuf];
    }
}

ISR(ADC0_RESRDY_vect) {
    // reading result clears the interrupt flag
    uint16_t val = ADC0.RESULT; 
    TCA0.SINGLE.CMP0BUF = val >> 2; // divide by 4 to get 14 bit range

    val = (2040*65536UL) / val;  // convert to Vdd[mv]

    // no race because the USART0_DRE can't run while this ISR runs
    put_char(val >> 8);
    put_char(val);
    put_flag();
}

int main() {
    //  FUSE.OSCCONFIG determines if we have 20MHz or 16MHz main clock
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, 1);  // 11.5.2 mainclock div 2, for 10 or 8MHz 

//  VREF.CTRLA   = VREF_AC0REFSEL_2V5_gc;       // sec 19.5.1 2.500V
    VREF.CTRLA   = VREF_AC0REFSEL_2V048_gc;     // sec 19.5.1 2.048V
    VREF.CTRLB   = VREF_AC0REFEN_bm;            // sec 19.5.2 keep on
    AC0.DACREF   = 0xFF;                        // sec 29.5.3 0xff is default -> 2.490V  / 2.040V

    ADC0.CTRLA   = ADC_ENABLE_bm|ADC_LOWLAT_bm; // ADC0 on, keep on between triggers
    ADC0.CTRLB   = ADC_PRESC_DIV2_gc;           // 5MHz (4MHz)
    ADC0.CTRLC   = (5<<3)| ADC_REFSEL_VDD_gc;   // 5 (4) ticks is timebase for 1us, Vref = Vdd
    ADC0.CTRLE   = 48;                          // sample duration 48 cycles
    ADC0.CTRLF   = ADC_SAMPNUM_ACC16_gc;        // 16 samples (4 extra bits, 16 bits total)
    ADC0.MUXPOS  = ADC_MUXPOS_DACREF0_gc;       // dacref0 as positive input (single ended)
    ADC0.INTCTRL = ADC_RESRDY_bm;               // irq on result ready
    ADC0.COMMAND = ADC_MODE_BURST_gc | ADC_START_EVENT_TRIGGER_gc;  // burst mode, start on event

    EVSYS.CHANNEL0 = EVSYS_CHANNEL0_TCA0_OVF_LUNF_gc; // channel 0 = TCA overflow
    EVSYS.USERADC0START = 1;                          // ADC start on event channel 0

    TCA0.SINGLE.PER   = 0x3fff;         // count in 14 bits: 10MHz/16384 = 610Hz / 1.6384 ms, or 488Hz / 2.048ms
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm;  
    TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc;

    PORTB.DIRSET   = 1<<0;             // set PB0 to be TCA0 WO0 signal from CMP0
    PORTB.PIN0CTRL = PORT_INVEN_bm;    // invert PB0 out (WO0) so we have PWM=1-(2.5V / Vdd)

    //USART0.BAUD  = 347; // 115200Bd for a 20MHz/2 F_PER
    setbaud(&USART0, 115200);
    USART0.CTRLB = USART_TXEN_bm; // enable TX, add USART_ODME_bm to set open drain mode
    USART0.CTRLA = USART_DREIE_bm; // enable TX register empty irq
    PORTB.DIRSET = 1<<2; // PB2 is TXD

    sei(); // enable interrupts

    for (;;)
        sleep_mode();
}





