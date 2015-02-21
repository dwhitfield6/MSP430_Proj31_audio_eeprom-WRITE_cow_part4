/*
* Here's an example using an MSP430G2553 to read/write using I2C to an 24LC512 and DS3231 RTC.
* Two functions are provided for read and write.
*
* The code avoids any while loops, and relies completely on interrupts and low power modes of the MSP430 to reduce the power consumption to the
* minimum.
*
* Author: Ahmed Talaat (aa_talaat@yahoo.com)
* Date: 19 October 2012
*/

#include  "msp430g2553.h"
#include "sounddata1.h"

    #define ONEBYTEADDR 1
    #define TWOBYTEADDR 2
    #define WRITE       0                       // ISR mode WRITE or READ
    #define READ        1
    #define NOACK       2
    #define EEPROM_ADDR 0x50

    unsigned char txdataEEPROM[] = "Here's an example using an MSP430G2553 and a 24LC512 and DS3231 RTC. Two functions are provided for read and write.";

    unsigned char rxdata[255];
    unsigned char nowdata;

    unsigned char count =0;
    unsigned char add2 =0;
    unsigned char add1 =0;
   unsigned long i=0;

    typedef struct {
        volatile unsigned char *data_buf;       // address of tx or rx data buffer
        volatile unsigned char buf_size;        // size of the buffer
        volatile unsigned char buf_index;       // index in the buffer
        volatile unsigned char addr_index;      // index of the byte address (0,1)
        volatile unsigned char isr_mode;        // Tx or Rx affects the interrupt logic
        volatile unsigned char addr_high_byte;  // High byte of the address to read/write to
        volatile unsigned char addr_low_byte;   // Low byte of the address to read/write to
        volatile unsigned char addr_type;       // two bytes like eeprom or 1 byte like RTC for example
    } i2c_t;

    i2c_t   i2c_packet;

    void i2c_init(void);
    void i2c_tx(unsigned char, unsigned char *, unsigned char, unsigned char, unsigned char, unsigned char);
    void i2c_rx(unsigned char, unsigned char *, unsigned char, unsigned char, unsigned char, unsigned char);
    void usdelay(int);

    void main(void)
    {
       WDTCTL = WDTPW + WDTHOLD;                // Stop WDT
       BCSCTL1 = CALBC1_1MHZ;                   // Set DCO to 1MHz
       DCOCTL = CALDCO_1MHZ;
       i2c_init();                              // Initialize I2C
                                                // Address is High byte then low byte
       for(i=21000; i<(NUM_ELEMENTS + 21000);i++)
       {
    	   nowdata = data[(i-21000)];
    	   add2 = ((i & 0xFF00) >> 8);
    	   add1 = (i & 0x00FF);
    	  // nowdata = (i & 0x00FF);
       i2c_tx(EEPROM_ADDR, &nowdata, 1,TWOBYTEADDR,add2,add1);//i2c TX 115 bytes starting @ address 01:00
       _delay_cycles(200);
       }


     //  i2c_rx(EEPROM_ADDR, rxdata, 255,TWOBYTEADDR,0x02,0xFF);//i2c RX 115 bytes from EEPROM starting @ address 01:00

_delay_cycles(200);


    }

    void i2c_init(void){
        P1SEL |= BIT6 + BIT7;                   //Set I2C pins
        P1SEL2|= BIT6 + BIT7;
        UCB0CTL1 |= UCSWRST;                    //Enable SW reset
        UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;   //I2C Master, synchronous mode
        UCB0CTL1 = UCSSEL_2 + UCSWRST;          //Use SMCLK, keep SW reset
        UCB0BR0 = 10;                           //fSCL = SMCLK/11 = ~100kHz
        UCB0BR1 = 0;
        UCB0CTL1 &= ~UCSWRST;                   //Clear SW reset, resume operation
        IE2 |= UCB0TXIE;                        //Enable TX interrupt
        IE2 |= UCB0RXIE;                        //Enable RX interrupt
        UCB0I2CIE |= UCNACKIE;                  //Need to enable the status change interrupt
        __enable_interrupt();                   //Enable global interrupt
    }

/*
 * This functions writes to any I2C device. It takes as parameters:
 *  - The slave address (7 bits)
 *  - Pointer to the data buffer to transmit
 *  - Size of the buffer
 *  - Size of the address location to start writing at. Being 2 bytes for some of the EEPROMs and single byte
 *    for other devices. Please note that the ISR of this function assumes that the address bytes are written high
 *    byte first, then low byte second.
 *    In case of single byte address device, only the high byte will be used to set the address.
 *  - The high byte and low byte address for the location to start writing at.
 *
 *  In case the start condition of the write operation is not ack (for example EEPROM busy with a previous write cycle),
 *  the corresponding interrupts detects this condition, generates a stop signal, and a Timer A1 (not Timer A0)
 *  is activated for 0.5 ms, then the trial for writing is repeated.
 *
 *  Please note that this function does not cater for the EEPROM 128byte paging requirements. So, if you are going
 *  to write more than 128 bytes to an EEPROM, you will need to write a higher level function to segment the
 *  writing into consecutive 128byte chunks.
 *
 */

    void i2c_tx(unsigned char slave_addr, unsigned char *txdata, unsigned char bufSize, unsigned char addr_size,
                        unsigned char high_byte_addr, unsigned char low_byte_addr) {
        i2c_packet.isr_mode=WRITE;
        i2c_packet.data_buf=txdata;
        i2c_packet.buf_size=bufSize;
        i2c_packet.buf_index=0;
        i2c_packet.addr_type=addr_size;
        i2c_packet.addr_high_byte=high_byte_addr;
        i2c_packet.addr_low_byte=low_byte_addr;
        i2c_packet.addr_index=0;
        UCB0I2CSA = slave_addr;                 //Slave Address

        while (1) {
            UCB0CTL1 |= UCTR + UCTXSTT;         // I2C TX, start condition
            LPM0;                               // Enter LPM0
                                                // and remain until all data is TX'd
            if (i2c_packet.isr_mode == NOACK){  // If no ack received, then sleep for 0.5ms and try again
                i2c_packet.isr_mode = WRITE;
                i2c_packet.addr_index=0;        // Reset the address index for the next write operation
                usdelay(500);

            } else {
                break;                          // Successful write, then quit
            }
        }
    }

    /*
     * This functions reads from any I2C device. It takes as parameters:
     *  - The slave address (7 bits)
     *  - Pointer to the data buffer to fill with data read.
     *  - Size of the buffer
     *  - Size of the address location to start writing at. Being 2 bytes for some of the EEPROMs and single byte
     *    for other devices. Please note that the ISR of this function assumes that the address bytes are written high
     *    byte first, then low byte second.
     *    In case of single byte address device, only the high byte will be used to set the address.
     *  - The high byte and low byte address for the location to start reading at.
     *
     *  The function starts with a write operation to specify the address at which the read operation with start
     *  In case the start condition of the write operation is not ack (for example EEPROM busy with a a previous write cycle),
     *  the corresponding interrupts detects this condition, generates a stop signal, and a Timer A1 (not Timer A0)
     *  is activated for 0.5 ms, then the trial for writing is repeated.
     *
     *  Once the write address is successful, the functions switch to read mode, and fills the buffer provided
     *
     */


    void i2c_rx(unsigned char slave_addr, unsigned char *rxdata, unsigned char bufSize, unsigned char addr_size,
            unsigned char high_byte_addr, unsigned char low_byte_addr) {
        i2c_packet.isr_mode=READ;               // The ISR will send the address bytes, then wake CPU.
        i2c_packet.addr_type=addr_size;
        i2c_packet.addr_high_byte=high_byte_addr;
        i2c_packet.addr_low_byte=low_byte_addr;
        i2c_packet.addr_index=0;
        UCB0I2CSA = slave_addr;                 // Slave Address

        while (1) {
            UCB0CTL1 |= UCTR + UCTXSTT;         // I2C TX, start condition
            LPM0;                               // Enter LPM0
                                                // and remain until all data is TX'd
            if (i2c_packet.isr_mode == NOACK){  // If no ack received, then sleep for 0.5ms and try again
                i2c_packet.isr_mode = READ;
                i2c_packet.addr_index=0;        // Reset the address index for the next write operation
                usdelay(500);
            } else {
                break;                          // Successful write, then quit
            }
        }
                                                // We wrote already the address, so now read only data.
        i2c_packet.addr_index=i2c_packet.addr_type;
        i2c_packet.data_buf=rxdata;
        i2c_packet.buf_size=bufSize;
        i2c_packet.buf_index=0;
        UCB0CTL1 &= ~UCTR;                      // I2C RX
        UCB0CTL1 |= UCTXSTT;                    // I2C re-start condition
        LPM0;                                   // Enter LPM0
                                                // and remain until all data is received
    }


    //interrupt(USCIAB0RX_VECTOR) state change to trap the no_Ack from slave case
    #pragma vector = USCIAB0RX_VECTOR
    __interrupt void USCIAB0RX_ISR(void)
    {
        if(UCNACKIFG & UCB0STAT) {
            UCB0STAT &= ~UCNACKIFG;             // Clear flag so that not to come here again
            i2c_packet.isr_mode=NOACK;          // The main function needs to act based on noack
            UCB0CTL1 |= UCTXSTP;                // I2C stop condition
            LPM0_EXIT;                          // Exit LPM0
        }
    }

/*
 * This interrupt is called each time the UCSI_B module is either ready to get a new byte in UCB0TXBUF to send to the I2C device, or
 * a new byte is read into UCB0RXBUF and we should pick it up.
 * The interrupt is called as both UCB0TXIE and UCB0RXIE are enabled. To stop this interrupt being called indefinitely, the corresponding
 * interrupt flag should be cleared.
 * These flags are automatically clearly by the USCI_B module if the UCB0XXBUF is access. However, if we are to do something different than reading
 * or writing a byte to/from the UCB0XXBUF, we need to clear the corresponding flag by ourselves or the ISR will be called for ever,
 * and the whole program will hang.
 */

    //interrupt(USCIAB0TX_VECTOR) USCIAB0TX_ISR(void)
    #pragma vector = USCIAB0TX_VECTOR
    __interrupt void USCIAB0TX_ISR(void)
    {
        // Transmit address bytes irrespective of send or receive mode.
        if (i2c_packet.addr_index==0){
           UCB0TXBUF = i2c_packet.addr_high_byte;
           i2c_packet.addr_index++;
        }
        else if (i2c_packet.addr_index==1 && i2c_packet.addr_type==TWOBYTEADDR){
            UCB0TXBUF = i2c_packet.addr_low_byte;
            i2c_packet.addr_index++;
        }
        else if(UCB0TXIFG & IFG2 && i2c_packet.isr_mode==READ) {
                                                // USCI_B is ready to get a new data byte to transmit it, and we are in READ mode.
                                                // So, we should not continue writing, but should exit to the calling function to
                                                // switch the USCI_B into read mode
            IFG2 &= ~UCB0TXIFG;                 // Clear USCI_B0 TX int flag manually as we did not write to the UCB0TXBUF
            LPM0_EXIT;                          // Exit LPM0
        }

        else if(UCB0TXIFG & IFG2 && i2c_packet.isr_mode==WRITE){// USCI_B is ready to get a new data byte to transmit it, and we are in write mode.

            if(i2c_packet.buf_index == i2c_packet.buf_size){    // If no more data to transmit, then issue stop condition and wake CPU.
                IFG2 &= ~UCB0TXIFG;                             // Clear USCI_B0 TX int flag manually as we did not write to the UCB0TXBUF
                UCB0CTL1 |= UCTXSTP;                            // I2C stop condition
                LPM0_EXIT;                                      // Exit LPM0
            } else {
                UCB0TXBUF = i2c_packet.data_buf[i2c_packet.buf_index];
                i2c_packet.buf_index++;                         // Increment TX byte counter
            }
       }
        else if (UCB0RXIFG & IFG2 && i2c_packet.addr_index==i2c_packet.addr_type) {
                                                                // Read mode, and we already completed writing the address
        i2c_packet.data_buf[i2c_packet.buf_index]= UCB0RXBUF;
        i2c_packet.buf_index++;                             // Increment RX byte counter
        if(i2c_packet.buf_index == i2c_packet.buf_size){    // If last byte to receive, then issue stop condition and wake CPU.
            IFG2 &= ~UCB0RXIFG;                             // Clear USCI_B0 RX int flag
            UCB0CTL1 |= UCTXSTP;                            // I2C stop condition here to avoid reading any extra bytes
            LPM0_EXIT;                                      // Exit LPM0
        }
    }
   }

    //------------------------------------------------------------------------------
    // micro seconds delays
    //
    void usdelay(int interval){
                                    // Setup TimerA
        TA1CCTL0 = CCIE;            // interrupt enabled
        TA1CCR0 = TA1R + interval;  // micro secs @ 1Mhz Clock
        TA1CTL = TASSEL_2 + MC_2;   // SMCLK, continuous mode.
        LPM0;                       // suspend CPU
    }

    // Timer A1 interrupt service routine. TIMERx_Ay_VECTOR.(x being the index of the timer, y of the vector for this timer)
    #pragma vector=TIMER1_A0_VECTOR
    __interrupt void Timer1_A0 (void)
    {
        TA1CTL = 0;                     // Stop Timer_A1
        LPM0_EXIT;                      // Return active
    }


