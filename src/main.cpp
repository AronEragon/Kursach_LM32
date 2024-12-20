
// #include "src/rcc.h"
// #include "src/adc.h"
// #include "src/gpio.h"
// #include "src/spi.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>

#define LBLUE GPIOE, GPIO8
#define LRED GPIOE, GPIO9
#define LORANGE GPIOE, GPIO10
#define LGREEN GPIOE, GPIO11
#define LBLUE2 GPIOE, GPIO12
#define LRED2 GPIOE, GPIO13
#define LORANGE2 GPIOE, GPIO14
#define LGREEN2 GPIOE, GPIO15

#define LD4 GPIOE, GPIO8
#define LD3 GPIOE, GPIO9
#define LD5 GPIOE, GPIO10
#define LD7 GPIOE, GPIO11
#define LD9 GPIOE, GPIO12
#define LD10 GPIOE, GPIO13
#define LD8 GPIOE, GPIO14
#define LD6 GPIOE, GPIO15


static void spi_setup(void)
{
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_SPI1);

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5 | GPIO6 | GPIO7);
    gpio_set_af(GPIOA, GPIO_AF5, GPIO5 | GPIO6 | GPIO7);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO5 | GPIO6 | GPIO7);

	spi_disable(SPI1);
    spi_set_master_mode(SPI1);
    spi_set_baudrate_prescaler(SPI1, SPI_CR1_BR_FPCLK_DIV_64);
    spi_set_clock_polarity_0(SPI1);
    spi_set_clock_phase_0(SPI1);
    spi_set_data_size(SPI1, SPI_CR2_DS_8BIT);
    spi_send_msb_first(SPI1);
    spi_enable_software_slave_management(SPI1);
    spi_set_nss_high(SPI1);
    spi_fifo_reception_threshold_8bit(SPI1);
    spi_enable(SPI1);


    // spi_disable(SPI1);
    // spi_set_master_mode(SPI1);
    // spi_set_baudrate_prescaler(SPI1, SPI_CR1_BR_FPCLK_DIV_64);
    // spi_set_clock_polarity_0(SPI1);
    // spi_set_clock_phase_0(SPI1);
    // spi_set_data_size(SPI1, SPI_CR2_DS_8BIT);
    // spi_send_msb_first(SPI1);
    // spi_enable_software_slave_management(SPI1);
    // spi_set_nss_high(SPI1);
    // spi_fifo_reception_threshold_8bit(SPI1);

    // spi_enable(SPI1);

}

/*


PA5 ---  SCK
PA7 ---  MOSI





lm35
VCC --- +5V/+3V
DAT --- PA0


usart2
rx зеленый --- PA3
tx белый --- PA2
черный --- GND
*/

static void adc_setup(void)
{
	//ADC
	rcc_periph_clock_enable(RCC_ADC12);
	rcc_periph_clock_enable(RCC_GPIOA);
	//ADC
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);
	adc_power_off(ADC1);
	adc_set_clk_prescale(ADC1, ADC_CCR_CKMODE_DIV2);
	adc_set_single_conversion_mode(ADC1);
	adc_disable_external_trigger_regular(ADC1);
	adc_set_right_aligned(ADC1);
	/* We want to read the temperature sensor, so we have to enable it. */
	adc_enable_temperature_sensor();
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_61DOT5CYC);
	uint8_t channel_array[] = { 1 }; /* ADC1_IN1 (PA0) */
	adc_set_regular_sequence(ADC1, 1, channel_array);
	adc_set_resolution(ADC1, ADC_CFGR1_RES_12_BIT);
	adc_power_on(ADC1);

	/* Wait for ADC starting up. */
	int i;
	for (i = 0; i < 800; i++)
		__asm__("nop");

}

static void gpio_setup(void)
{
	rcc_periph_clock_enable(RCC_GPIOE);
	gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
		GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13 |
		GPIO14 | GPIO15);
}

static void clock_setup(void)
{
	rcc_clock_setup_hsi(&rcc_hsi_configs[RCC_CLOCK_HSI_64MHZ]);
}


int main(void)
{
	uint16_t temp;

	clock_setup();
	gpio_setup();
	adc_setup();
	spi_setup();
	
	while (1) {
		adc_start_conversion_regular(ADC1);
		while (!(adc_eoc(ADC1)));
		temp=adc_read_regular(ADC1);
 		gpio_port_write(GPIOE, temp << 4);
		
		temp = temp*0.01;

		temp=(temp% 10);
		

		// Чтение температуры
		//gpio_clear(GPIOE, GPIO3);		
		spi_send8(SPI1, static_cast<uint8_t>(temp));
		// spi_send8(SPI1, 85);

		spi_read8(SPI1);
		
		

		//gpio_set(GPIOE, GPIO3);
		
		int i;
		for (i = 0; i < 2000000; i++)    /* Wait a bit. */
			__asm__("nop");



	}

	return 0;
}

// int main(void)
// {
// 	uint16_t temp;

// 	clock_setup();
// 	gpio_setup();
// 	adc_setup();
// 	//usart_setup();
// 	spi_setup();

// 	while (1) {
// 		adc_start_conversion_regular(ADC1);
// 		while (!(adc_eoc(ADC1)));
// 		temp=adc_read_regular(ADC1);
//  		gpio_port_write(GPIOE, temp << 4);
// 		//my_usart_print_int(USART2, temp);
		
// 		// Чтение температуры
// 		gpio_clear(GPIOE, GPIO3);
		
// 		spi_send8(SPI1, static_cast<uint8_t>(temp));

// 		//spi_send8(SPI1, 85);
// 		spi_read8(SPI1);
// 		spi_send8(SPI1, 0);
// 		temp=spi_read8(SPI1);
// 		//my_usart_print_int(USART2, (temp));
// 		gpio_set(GPIOE, GPIO3);
		
// 		int i;
// 		for (i = 0; i < 80000; i++)    /* Wait a bit. */
// 			__asm__("nop");
// 	}

// 	return 0;
// }