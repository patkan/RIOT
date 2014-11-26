/**
 * @file
 * @ingroup     STM32F3Discovery-diy
 * @brief       CC1100 STM32F3Discovery dependend functions
 *
 * @author      Michael Zapf <michael.zapf@fau.de>
 * @version     $Revision: 1 $
 */

#include <stdio.h>
#include <stddef.h>
/* core */
#include "irq.h"
/* cpu */
#include "cpu.h"
/* drivers  */
#include "cc110x_legacy.h"

#include "gpioint.h"

/* SPI PORT and PINs */
#define SPI_PORT		GPIOA 	
#define SPI_PIN_MOSI	7
#define SPI_PIN_MISO	6
#define SPI_PIN_CLK		5

/* Latch enable of C1101 Transceiver */
#define C1101_LE_PORT	GPIOA	
#define C1101_LE_PIN	4	

/* Values for SPI control register 1 (SPIx_CR1) */
#define SPI_CPHA		(1 << 0)
#define SPI_CPOL		(1 << 1)
#define SPI_MSTR		(1 << 2)
#define SPI_BR_DIV2		(0b000 << 3)
#define SPI_BR_DIV4		(0b001 << 3)
#define SPI_BR_DIV8		(0b010 << 3)
#define SPI_BR_DIV16	(0b011 << 3)
#define SPI_BR_DIV32	(0b100 << 3)
#define SPI_BR_DIV64	(0b101 << 3)
#define SPI_BR_DIV128	(0b110 << 3)
#define SPI_BR_DIV256	(0b111 << 3)
#define SPI_SPE			(1 << 4)
#define SPI_LSBFIRST	(1 << 5)
#define SPI_SSI			(1 << 6)
#define SPI_SSM			(1 << 7)
#define SPI_RXONLY		(1 << 8)
#define SPI_CRCL		(1 << 9)
#define SPI_CRCNEXT		(1 << 10)
#define SPI_CRCEN		(1 << 11)
#define SPI_BIDIOE		(1 << 12)
#define SPI_BIDIMODE	(1 << 13)

/* Values for SPI control register 2 (SPIx_CR2) */
#define SPI_RXDMAEN		(1 << 0)
#define SPI_TXDMAEN		(1 << 1)
#define SPI_SSOE		(1 << 2)
#define SPI_NSSP		(1 << 3)
#define SPI_FRF			(1 << 4)
#define SPI_ERRIE		(1 << 5)
#define SPI_RXNEIE		(1 << 6)
#define SPI_TXEIE		(1 << 7)
#define SPI_DS_4bit		(0b0011 << 8)
#define SPI_DS_5bit		(0b0100 << 8)
#define SPI_DS_6bit		(0b0101 << 8)
#define SPI_DS_7bit		(0b0110 << 8)
#define SPI_DS_8bit		(0b0111 << 8)
#define SPI_DS_9bit		(0b1000 << 8)
#define SPI_DS_10bit	(0b1001 << 8)
#define SPI_DS_11bit	(0b1010 << 8)
#define SPI_DS_12bit	(0b1011 << 8)
#define SPI_DS_13bit	(0b1100 << 8)
#define SPI_DS_14bit	(0b1101 << 8)
#define SPI_DS_15bit	(0b1110 << 8)
#define SPI_DS_16bit	(0b1111 << 8)
#define SPI_FRXTH		(1 << 12)
#define SPI_LDMA_RX		(1 << 13)
#define SPI_LDMA_TX		(1 << 14)

/* Values for SPI status register (SPIx_SR) */
#define SPI_RXNE		(1 << 0)
#define SPI_TXE			(1 << 1)
#define SPI_CHSIDE		(1 << 2)
#define SPI_UDR			(1 << 3)
#define SPI_CRCERR		(1 << 4)
#define SPI_MODF		(1 << 5)
#define SPI_OVR			(1 << 6)
#define SPI_BSY			(1 << 7)
#define SPI_FRE			(1 << 8)
#define SPI_FRLVL_FIFO_empty	(0b00 << 0)
#define SPI_FRLVL_FIFO_DIV4		(0b01 << 0)
#define SPI_FRLVL_FIFO_DIV2		(0b10 << 0)
#define SPI_FRLVL_FIFO_full		(0b11 << 0)
#define SPI_FTLVL_FIFO_empty	(0b00 << 0)
#define SPI_FTLVL_FIFO_DIV4		(0b01 << 0)
#define SPI_FTLVL_FIFO_DIV2		(0b10 << 0)
#define SPI_FTLVL_FIFO_full		(0b11 << 0)


#define CC1100_GDO0         (FIO0PIN & BIT27)   // read serial I/O (GDO0)   //TODO
#define CC1100_GDO1         (FIO1PIN & BIT23)   // read serial I/O (GDO1)   //TODO
#define CC1100_GDO2         (FIO0PIN & BIT28)   // read serial I/O (GDO2)   //TODO

#define SPI_TX_EMPTY                (SPI_PORT->SPI1_SR & SPI_TXE)
#define SPI_BUSY                    (SPI_PORT->SPI1_SR & SPI_BSY)
#define SPI_RX_AVAIL                (SPI_PORT->SPI1_SR & SPI_RXNE)

#define CC1100_GDO1_LOW_RETRY        (100)      // max. retries for GDO1 to go low
#define CC1100_GDO1_LOW_COUNT       (2700)      // loop count (timeout ~ 500 us) to wait
// for GDO1 to go low when CS low

//#define DEBUG
#ifdef DEBUG

static unsigned long time_value;

static void set_time(void)
{
    time_value = 0;
}

static int test_time(int code)
{
    time_value++;

    if (time_value > 10000000) {
        printf("CC1100 SPI alarm: %d!\n", code);
        time_value = 0;
        return 1;
    }

    return 0;
}
#endif

int cc110x_get_gdo0(void)
{
    return  CC1100_GDO0;
}

int cc110x_get_gdo1(void)
{
    return  CC1100_GDO1;
}

int cc110x_get_gdo2(void)
{
    return  CC1100_GDO2;
}


void cc110x_spi_init(void)
{
    // configure chip-select (hier: NSS)
	// NSS (Active Low), connected to PA4 (check if up-to-date!)
	C1101_LE_PORT->AFRL   |= (0x5  << 4*C1101_LE_PIN); // AFRL:   0x05 (SPI1_NSS)
	C1101_LE_PORT->OTYPER |= (0b0  << 1*C1101_LE_PIN); // OTYPER: 0b0  (Push/Pull)
	C1101_LE_PORT->OSPEED |= (0b11 << 2*C1101_LE_PIN); // OSPEED: 0b01 (High-Speed)
	C1101_LE_PORT->PUPDR  |= (0b00 << 2*C1101_LE_PIN); // PUPDR:  0b00 (No Pull up/down)
	C1101_LE_PORT->MODER  |= (0b01 << 2*C1101_LE_PIN); // MODER:  0b01 (Output)

	//set le high
	C1101_LE_PORT->ODR  |= (0b1 << 1*C1101_LE_PIN); // ODR:  0b1 (set Output pin high)


	// 1. Write proper GPIO registers: Configure GPIO for MOSI, MISO and SCK pins.
	//are OTYPER, OSPEED and PUPDR really necessary? It already knows the function of the pins 
	//by the AFRL-Register. I think they should be left out if it doesn't work.

	// configure SPI1_MOSI - Pin
	SPI_PORT->AFRL   |= (0x5  << 4*SPI_MOSI_PIN); // AFRL:   0x05 (SPI1_MOSI)
	SPI_PORT->OTYPER |= (0b0  << 1*SPI_MOSI_PIN); // OTYPER: 0b0  (Push/Pull)
	SPI_PORT->OSPEED |= (0b11 << 2*SPI_MOSI_PIN); // OSPEED: 0b01 (High-Speed)
	SPI_PORT->PUPDR  |= (0b00 << 2*SPI_MOSI_PIN); // PUPDR:  0b00 (No Pull up/down)
	SPI_PORT->MODER  |= (0b01 << 2*SPI_MOSI_PIN); // MODER:  0b01 (Output)

	// configure SPI1_MISO - Pin
	SPI_PORT->AFRL   |= (0x5  << 4*SPI_MISO_PIN); // AFRL:   0x05 (SPI1_MISO)
	SPI_PORT->OTYPER |= (0b0  << 1*SPI_MISO_PIN); // OTYPER: 0b0  (Push/Pull)
	SPI_PORT->OSPEED |= (0b11 << 2*SPI_MISO_PIN); // OSPEED: 0b01 (High-Speed)
	SPI_PORT->PUPDR  |= (0b00 << 2*SPI_MISO_PIN); // PUPDR:  0b00 (No Pull up/down)
	SPI_PORT->MODER  |= (0b01 << 2*SPI_MISO_PIN); // MODER:  0b00 (Input)

	// configure SPI1_CLK - Pin
	SPI_PORT->AFRL   |= (0x5  << 4*SPI_CLK_PIN); // AFRL:   0x05 (SPI1_CLK)
	SPI_PORT->OTYPER |= (0b0  << 1*SPI_CLK_PIN); // OTYPER: 0b0  (Push/Pull)
	SPI_PORT->OSPEED |= (0b11 << 2*SPI_CLK_PIN); // OSPEED: 0b01 (High-Speed)
	SPI_PORT->PUPDR  |= (0b00 << 2*SPI_CLK_PIN); // PUPDR:  0b00 (No Pull up/down)
	SPI_PORT->MODER  |= (0b01 << 2*SPI_CLK_PIN); // MODER:  0b01 (Output)

	// 2. Write to the SPI_CR1 register:
	// 2a) configure the serial clock baud rate 
	SPI_PORT->SPI_CR1 |= SPI_BR_DIV128;	//72MHz/128 = 562.5kHz that should be enough :)

	// 2b) Configure the CPOL and CPHA bits combination 
	//SPI_PORT->SPI_CR1 |= SPI_CPOL;	// CPOL = 0 -> uncommented
	//SPI_PORT->SPI_CR1 |= SPI_CPHA;	// CPHA = 0 -> uncommented

	// 2c) Select simplex or half-duplex mode by configuring RXONLY or BIDIMODE and BIDIOE
	SPI_PORT->SPI_CR1 |= SPI_BIDIMODE | SPI_BIDIOE;

	// 2d) Configure the LSBFIRST bit to define the frame format
	//SPI_PORT->SPI_CR1 |= SPI_LSBFIRST; // lsb/msb??? TODO!

	// 2e) Configure the CRCL and CRCEN bits if CRC is needed (while SCK clock signal is at idle state).
	// I hope, we don't need that

	// 2f) Configure SSM and SSI 
	// No Slave, no need for that

	//Configure the MSTR bit
	SPI_PORT->SPI_CR1 |= SPI_MSTR;

	// 3. Write to SPI_CR2 register:
	// 3a) Configure the DS[3:0] bits to select the data length for the transfer.
	SPI_PORT->SPI_CR2 |= SPI_DS_8bit;

	// 3b) Configure SSOE - was das?
	//SPI_PORT->SPI_CR2 |= SPI_SSOE;	

	// 3c) Set the FRF bit if the TI protocol is required (keep NSSP bit cleared in TI mode).
	//SPI_PORT->SPI_CR2 |= SPI_FRF;

	// 3d) Set the NSSP bit if the NSS pulse mode between two data units is required 
	//SPI_PORT->SPI_CR2 |= SPI_NSSP;

	// 3e) Configure the FRXTH bit. The RXFIFO threshold must be aligned to the read access size for the SPIx_DR register.
	//SPI_PORT->SPI_CR2 |= SPI_SSOE;

	// 3f) Initialize LDMA_TX and LDMA_RX bits if DMA is used in packed mode.
	//SPI_PORT->SPI_CR2 |= SPI_SSOE;

	//SPI enable
	SPI_PORT->SPI_CR1 |= SPI_SPE;


    int dummy;

    // Clear RxFIFO:
    while (SPI_RX_AVAIL) {                      // while RNE (Receive FIFO Not Empty)...
        dummy = SSP0DR;                         // read data
    }

    /* to suppress unused-but-set-variable */
    (void) dummy;
}
    

uint8_t cc110x_txrx(uint8_t c)
{
    uint8_t result;
    SSP0DR = c; //TODO
#ifdef DEBUG
    set_time();
#endif

    while (!SPI_TX_EMPTY) {
#ifdef DEBUG
        test_time(0);
#endif
    }

#ifdef DEBUG
    set_time();
#endif

    while (SPI_BUSY) {
#ifdef DEBUG
        test_time(1);
#endif
    }

#ifdef DEBUG
    set_time();
#endif

    while (!SPI_RX_AVAIL) {
#ifdef DEBUG
        test_time(2);
#endif
    }

    result = (uint8_t)SSP0DR; //TODO
    return result;
}

void cc110x_spi_cs(void)
{
    FIO1CLR = BIT21; //TODO
}


// TODO 
// Was bitte macht diese Funktion???
void
cc110x_spi_select(void)
{
    volatile int retry_count = 0;
    volatile int abort_count;
    // Switch to GDO mode input
    PINSEL3 &= ~(BIT14 + BIT15);// Set MISO function to GPIO
    FIO1DIR &= ~BIT23;
cs_low:
    // CS to low
    abort_count = 0;
    FIO1CLR = BIT21; //TODO
    // Wait for SO to go low (voltage regulator
    // has stabilized and the crystal is running)
loop:
    asm volatile("nop");

    if (CC1100_GDO1) {
        abort_count++;

        if (abort_count > CC1100_GDO1_LOW_COUNT) {
            retry_count++;

            if (retry_count > CC1100_GDO1_LOW_RETRY) {
                puts("[CC1100 SPI] fatal error\n");
                goto final;
            }

            FIO1SET = BIT21;    // CS to high
            goto cs_low;        // try again
        }

        goto loop;
    }

final:
    // Switch to SPI mode
    PINSEL3 |= (BIT14 + BIT15); // Set MISO function to SPI
}

//TODO
void
cc110x_spi_unselect(void)
{
    FIO1SET = BIT21;//TODO
}

void cc110x_before_send(void)
{
    // Disable GDO2 interrupt before sending packet
    cc110x_gdo2_disable();
}

void cc110x_after_send(void)
{
    // Enable GDO2 interrupt after sending packet
    cc110x_gdo2_enable();
}

// TODO
void cc110x_gdo0_enable(void)
{
	//Wo wird diese Funktion definiert?
	//müsste man ja eigentlich so sein lassen dürfen?

    gpioint_set(0, BIT27, GPIOINT_RISING_EDGE, &cc110x_gdo0_irq);
}

// TODO
void cc110x_gdo0_disable(void)
{
    gpioint_set(0, BIT27, GPIOINT_DISABLE, NULL);
}

// TODO
void cc110x_gdo2_disable(void)
{
    gpioint_set(0, BIT28, GPIOINT_DISABLE, NULL);
}

// TODO
void cc110x_gdo2_enable(void)
{
    gpioint_set(0, BIT28, GPIOINT_FALLING_EDGE, &cc110x_gdo2_irq);
}

// TODO
void cc110x_init_interrupts(void)
{
    // Enable external interrupt on low edge (for GDO2)
    //FIO0DIR &= ~BIT28;
    //cc110x_gdo2_enable();
    // Enable external interrupt on low edge (for GDO0)
    //FIO0DIR &= ~BIT27;
}
