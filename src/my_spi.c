static void spi_setup(void)
{
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5 | GPIO6 | GPIO7);
    gpio_set_af(GPIOA, GPIO_AF5, GPIO5 | GPIO6 | GPIO7);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO5 | GPIO6 | GPIO7);

    spi_disable(SPI1);
    spi_set_master_mode(SPI1);
    spi_set_baudrate_prescaler(SPI1, SPI_CR1_BR_FPCLK_DIV_64);
    spi_set_clock_polarity_0(SPI1);
    spi_set_clock_phase_0(SPI1);
    spi_send_msb_first(SPI1);
    spi_enable_software_slave_management(SPI1);
    spi_set_crcl_8bit(SPI1);
    spi_set_nss_high(SPI1);
    gpio_set(GPIOA, GPIO4);

    spi_enable(SPI1);
}