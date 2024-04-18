#ifdef TTGO_T1
// SPI specific
#define LCD_SPI_HOST    SPI3_HOST
#define LCD_PIN_NUM_MOSI 19
#define LCD_PIN_NUM_CLK 18
#define LCD_PIN_NUM_CS 5
#define LCD_PIN_NUM_DC 16
#define LCD_PIN_NUM_RST 23
#define LCD_PIN_NUM_BCKL 4
#define LCD_BCKL_ON_LEVEL 1
#define LCD_MAX_TRANSFER (LCD_HRES*LCD_VRES*2)
#define LCD_PIXEL_CLOCK_HZ (40 * 1000 * 1000)
// End SPI specific
#define LCD_BCKL_OFF_LEVEL !LCD_BCKL_ON_LEVEL
#define LCD_PANEL esp_lcd_new_panel_st7789
#define LCD_HRES 135
#define LCD_VRES 240
#define LCD_COLOR_SPACE ESP_LCD_COLOR_SPACE_RGB
#define LCD_GAP_X 52
#define LCD_GAP_Y 40
#define LCD_MIRROR_X false
#define LCD_MIRROR_Y false
#define LCD_INVERT_COLOR true
#define LCD_SWAP_XY false
#endif // TTGO_T1

#ifdef HELTEC_WIFI_KIT_V2
// I2C specific
#define LCD_I2C_HOST    0
#define LCD_I2C_ADDR 0x3C
#define LCD_CONTROL_PHASE_BYTES 1
#define LCD_DC_BIT_OFFSET 6
#define LCD_PIN_NUM_SCL 15
#define LCD_PIN_NUM_SDA 4
#define LCD_PIXEL_CLOCK_HZ (400 * 1000)
// END I2C specific
#define LCD_BIT_DEPTH 1
#define LCD_PIN_NUM_RST 16
#define LCD_PANEL esp_lcd_new_panel_ssd1306
#define LCD_HRES 128
#define LCD_VRES 64
#define LCD_COLOR_SPACE ESP_LCD_COLOR_SPACE_MONOCHROME
#define LCD_GAP_X 0
#define LCD_GAP_Y 0
#define LCD_MIRROR_X true
#define LCD_MIRROR_Y true
#define LCD_INVERT_COLOR false
#define LCD_SWAP_XY false
#define LCD_Y_ALIGN 8
#endif // HELTEC_WIFI_KIT_V2



#include <stdint.h>
#include <memory.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <driver/i2c.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
volatile int flushing;
esp_lcd_panel_handle_t lcd_handle;
esp_err_t lcd_draw_bitmap(int x1, int y1, int x2, int y2, void* bitmap) {
    flushing = 1;
    return esp_lcd_panel_draw_bitmap(lcd_handle, x1, y1, x2 + 1, y2 + 1, bitmap);
}
bool lcd_flush_complete(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx) {
    flushing = 0;
    return true;
}
void lcd_init() {
    #ifdef LCD_PIN_NUM_BCKL
    gpio_set_direction(LCD_PIN_NUM_BCKL, GPIO_MODE_OUTPUT);
#endif               // LCD_PIN_NUM_BCKL
#ifdef LCD_I2C_HOST // I2C
    i2c_config_t i2c_conf;
    memset(&i2c_conf,0,sizeof(i2c_config_t));
    
    i2c_conf.mode = I2C_MODE_MASTER,
    i2c_conf.sda_io_num = LCD_PIN_NUM_SDA;
    i2c_conf.scl_io_num = LCD_PIN_NUM_SCL;
    i2c_conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_conf.master.clk_speed = LCD_PIXEL_CLOCK_HZ;
    
    ESP_ERROR_CHECK(i2c_param_config(LCD_I2C_HOST, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(LCD_I2C_HOST, I2C_MODE_MASTER, 0, 0, 0));

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config;
    memset(&io_config,0,sizeof(esp_lcd_panel_io_i2c_config_t));
    io_config.dev_addr = LCD_I2C_ADDR;
#ifdef LCD_CONTROL_PHASE_BYTES
    io_config.control_phase_bytes = LCD_CONTROL_PHASE_BYTES;
#else
    io_config.control_phase_bytes = 0;
#endif
    io_config.lcd_cmd_bits = 8;   
    io_config.lcd_param_bits = 8; 
    io_config.on_color_trans_done = lcd_flush_complete;
    io_config.dc_bit_offset = LCD_DC_BIT_OFFSET;  
#if defined(LCD_ENABLE_CONTROL_PHASE) && LCD_ENABLE_CONTROL_PHASE != 0
    io_config.flags.disable_control_phase = 1;
#endif
    esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)LCD_I2C_HOST, &io_config, &io_handle);
#elif defined(LCD_SPI_HOST)  // 1-bit SPI
    spi_bus_config_t bus_config;
    memset(&bus_config, 0, sizeof(bus_config));
    bus_config.sclk_io_num = LCD_PIN_NUM_CLK;
    bus_config.mosi_io_num = LCD_PIN_NUM_MOSI;
#ifdef LCD_PIN_NUM_MISO
    bus_config.miso_io_num = LCD_PIN_NUM_MISO;
#else
    bus_config.miso_io_num = -1;
#endif  // LCD_PIN_NUM_MISO
#ifdef LCD_PIN_NUM_QUADWP
    bus_config.quadwp_io_num = LCD_PIN_NUM_QUADWP;
#else
    bus_config.quadwp_io_num = -1;
#endif
#ifdef LCD_PIN_NUM_QUADHD
    bus_config.quadhd_io_num = LCD_PIN_NUM_QUADHD;
#else
    bus_config.quadhd_io_num = -1;
#endif
    bus_config.max_transfer_sz = LCD_MAX_TRANSFER + 8;

    // Initialize the SPI bus on LCD_SPI_HOST
    spi_bus_initialize(LCD_SPI_HOST, &bus_config, SPI_DMA_CH_AUTO);

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config;
    memset(&io_config, 0, sizeof(io_config));
    io_config.dc_gpio_num = LCD_PIN_NUM_DC,
    io_config.cs_gpio_num = LCD_PIN_NUM_CS,
    io_config.pclk_hz = LCD_PIXEL_CLOCK_HZ,
    io_config.lcd_cmd_bits = 8,
    io_config.lcd_param_bits = 8,
    io_config.spi_mode = 0,
    io_config.trans_queue_depth = 10,
    io_config.on_color_trans_done = lcd_flush_complete;
    // Attach the LCD to the SPI bus
    esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_SPI_HOST,
                             &io_config, &io_handle);
#endif
    lcd_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config;
    memset(&panel_config, 0, sizeof(panel_config));
#ifdef LCD_PIN_NUM_RST
    panel_config.reset_gpio_num = LCD_PIN_NUM_RST;
#else
    panel_config.reset_gpio_num = -1;
#endif
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
if(((int)LCD_COLOR_SPACE) == 0) {
    panel_config.rgb_endian = LCD_RGB_ENDIAN_RGB;
} else {
    panel_config.rgb_endian = LCD_RGB_ENDIAN_BGR;
}
#else
    panel_config.color_space = LCD_COLOR_SPACE;
#endif
#ifndef LCD_BIT_DEPTH
    panel_config.bits_per_pixel = 16;
#else
    panel_config.bits_per_pixel = LCD_BIT_DEPTH;
#endif
    // Initialize the LCD configuration
    LCD_PANEL(io_handle, &panel_config, &lcd_handle);

#ifdef LCD_PIN_NUM_BCKL
    // Turn off backlight to avoid unpredictable display on
    // the LCD screen while initializing
    // the LCD panel driver. (Different LCD screens may need different levels)
    gpio_set_level((gpio_num_t)LCD_PIN_NUM_BCKL, LCD_BCKL_OFF_LEVEL);
#endif  // LCD_PIN_NUM_BCKL
    // Reset the display
    esp_lcd_panel_reset(lcd_handle);

    // Initialize LCD panel
    esp_lcd_panel_init(lcd_handle);

    esp_lcd_panel_swap_xy(lcd_handle, LCD_SWAP_XY);
    esp_lcd_panel_set_gap(lcd_handle, LCD_GAP_X, LCD_GAP_Y);
    esp_lcd_panel_mirror(lcd_handle, LCD_MIRROR_X, LCD_MIRROR_Y);
    esp_lcd_panel_invert_color(lcd_handle, LCD_INVERT_COLOR);
    // Turn on the screen
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    esp_lcd_panel_disp_on_off(lcd_handle, true);
#else
    esp_lcd_panel_disp_off(lcd_handle, false);
#endif

#ifdef LCD_PIN_NUM_BCKL
    // Turn on backlight (Different LCD screens may need different levels)
    gpio_set_level((gpio_num_t)LCD_PIN_NUM_BCKL, LCD_BCKL_ON_LEVEL);
#endif  // LCD_PIN_NUM_BCKL
}
uint8_t frame_buffer[LCD_VRES*LCD_HRES*2];
void app_main() {
    flushing = 0;
    memset(frame_buffer,0xFF,sizeof(frame_buffer));
    lcd_init();
    lcd_draw_bitmap(0,0,LCD_HRES-1,LCD_VRES-1,frame_buffer);
    
}