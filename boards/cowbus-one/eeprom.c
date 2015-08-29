#include "eeprom.h"


void eeprom_init(void) {

    // TODO: SPI init

    gpio_init_out(EEPROM_CS_GPIO, GPIO_NOPULL);
    gpio_set(EEPROM_CS_GPIO);

    gpio_init_out(EEPROM_HOLD_GPIO, GPIO_NOPULL);
    gpio_set(EEPROM_CS_HOLD);
}

char eeprom_get_status(void){
    char ret = 0;

    //drive CS low
    gpio_clear(GPIO_4);

    //Write 0x05
    spi_transfer_byte(SPI_0, 0x05, &ret);

    //read 8 bit
    spi_transfer_byte(SPI_0, 0x00, &ret);

    //drive CS high
    gpio_set(GPIO_4);

    return ret;
}
