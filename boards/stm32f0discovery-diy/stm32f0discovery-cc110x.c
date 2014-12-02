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

#include "periph/spi.h"
#include "periph/gpio.h"

/* SPI PORT and PINs */
//#define SPI_PORT		GPIOA 	
//#define SPI_PIN_MOSI	7
//#define SPI_PIN_MISO	6
//#define SPI_PIN_CLK	5

/* Latch enable of C1101 Transceiver */
#define C1101_LE_PORT	GPIOA	
#define C1101_LE_PIN	4	



#define CC1100_GDO0         (gpio_read(GPIO_3) != 0)   // read serial I/O (GDO0)   
#define CC1100_GDO1         (gpio_read(GPIO_4) != 0)   // read serial I/O (GDO1)   
#define CC1100_GDO2         (gpio_read(GPIO_5) != 0)   // read serial I/O (GDO2)   

#define SPI_TX_EMPTY                (SPI_0_DEV->SR & SPI_SR_TXE)
#define SPI_BUSY                    (SPI_0_DEV->SR & SPI_SR_BSY)
#define SPI_RX_AVAIL                (SPI_0_DEV->SR & SPI_SR_RXNE)

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
	
	gpio_init_out(GPIO_2, GPIO_NOPULL); //LE von CC1101
	gpio_set(GPIO_2);    // CS to high

	spi_init_master(SPI_0, SPI_CONF_FIRST_RISING, SPI_SPEED_100KHZ);

	gpio_init_int(GPIO_3, GPIO_NOPULL, GPIO_RISING, (void *)cc110x_gdo0_irq, 0);	//CC1100_GDO0
	gpio_init_in(GPIO_4, GPIO_NOPULL);	//CC1100_GDO1 = MISO!
	gpio_init_int(GPIO_5, GPIO_NOPULL, GPIO_RISING, (void *)cc110x_gdo2_irq, 0);	//CC1100_GDO2




    int dummy;

    // Clear RxFIFO:
    while (SPI_RX_AVAIL) {                      // while RNE (Receive FIFO Not Empty)...
        dummy = *((volatile uint8_t *)(&SPI_0_DEV->DR));	// read data
    }

    /* to suppress unused-but-set-variable */
    (void) dummy;
}
    

uint8_t cc110x_txrx(uint8_t c)
{
    uint8_t result;
    *((volatile uint8_t *)(&SPI_0_DEV->DR)) = (uint8_t)c;
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

    result = *((volatile uint8_t *)(&SPI_0_DEV->DR)); 
    return result;
}

void cc110x_spi_cs(void)
{
	gpio_clear(GPIO_2);  //set cs low
}



// Was bitte macht diese Funktion???
void
cc110x_spi_select(void)
{
    volatile int retry_count = 0;
    volatile int abort_count;
    // Switch to GDO mode input
    //PINSEL3 &= ~(BIT14 + BIT15);// Set MISO function to GPIO 
	spi_poweroff(SPI_0); //SPI deaktivieren. Hier lieber das EN-Flag veraendern?

    //FIO1DIR &= ~BIT23; //setze GDO1 auf input
	//GDO1 als Eingang definieren. Ist es schon

cs_low:
    // CS to low
    abort_count = 0;
    gpio_clear(GPIO_2); 
    // Wait for SO to go low (voltage regulator
    // has stabilized and the crystal is running)
loop:
    asm volatile("nop");

    if (CC1100_GDO1) { //= MISO-PIN
        abort_count++;

        if (abort_count > CC1100_GDO1_LOW_COUNT) {
            retry_count++;

            if (retry_count > CC1100_GDO1_LOW_RETRY) {
                puts("[CC1100 SPI] fatal error\n");
                goto final;
            }

            gpio_set(GPIO_2);    // CS to high
            goto cs_low;        // try again
        }

        goto loop;
    }

final:
    // Switch to SPI mode
    //PINSEL3 |= (BIT14 + BIT15); // Set MISO function to SPI
	spi_poweron(SPI_0); //SPI aktivieren. Hier lieber das EN-Flag veraendern?

}


void
cc110x_spi_unselect(void)
{
    gpio_set(GPIO_2); //set cs high
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

void cc110x_gdo0_enable(void)
{
    //gpioint_set(0, BIT27, GPIOINT_RISING_EDGE, &cc110x_gdo0_irq);
	gpio_irq_enable(GPIO_3);
}

void cc110x_gdo0_disable(void)
{
    gpio_irq_disable(GPIO_3);
}

void cc110x_gdo2_disable(void)
{
    gpio_irq_disable(GPIO_5);
}

void cc110x_gdo2_enable(void)
{
    gpio_irq_enable(GPIO_5);
}

void cc110x_init_interrupts(void)
{
    // Enable external interrupt on low edge (for GDO2)
    //FIO0DIR &= ~BIT28;
    //cc110x_gdo2_enable();
    // Enable external interrupt on low edge (for GDO0)
    //FIO0DIR &= ~BIT27;
}
