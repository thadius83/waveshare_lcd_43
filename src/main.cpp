#include <stdio.h>
#include <unistd.h>
#include <sys/lock.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_timer.h>
#include <esp_idf_version.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_rgb.h>
#include <esp_lcd_panel_vendor.h>
#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <string.h>
#include "lcd_config.h"
#include <lvgl.h>
#include "ui/ui.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_gt911.h"
#define I2C_MASTER_SCL_IO           9       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           8       /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0       /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

// Global/shared I2C bus
static i2c_master_bus_handle_t s_i2c_bus = nullptr;

#define GPIO_INPUT_IO_4    4
#define GPIO_INPUT_PIN_SEL  1ULL<<GPIO_INPUT_IO_4

static esp_lcd_panel_handle_t lcd_handle = nullptr;
static SemaphoreHandle_t sem_vsync_end;
static SemaphoreHandle_t sem_gui_ready;
// LVGL library is not thread-safe, this example will call LVGL APIs from different tasks, so use a mutex to protect it
static _lock_t lvgl_api_lock;

static void i2c_initialize(void)
{
    printf("I2C Init: Configuring I2C master bus...\n");
    printf("I2C Init: SCL GPIO=%d, SDA GPIO=%d, Freq=%dHz\n", 
           I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, I2C_MASTER_FREQ_HZ);
    
    i2c_master_bus_config_t i2c_mst_config;
    memset(&i2c_mst_config,0,sizeof(i2c_mst_config));
    i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_mst_config.i2c_port = (i2c_port_num_t)I2C_MASTER_NUM;
    i2c_mst_config.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    i2c_mst_config.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    i2c_mst_config.glitch_ignore_cnt = 7;
    i2c_mst_config.flags.enable_internal_pullup = 1;
    
    esp_err_t ret = i2c_new_master_bus(&i2c_mst_config, &s_i2c_bus);
    if (ret == ESP_OK) {
        printf("I2C Init: Master bus created successfully\n");
    } else {
        printf("I2C Init: Failed to create master bus, error: 0x%x\n", ret);
    }
    ESP_ERROR_CHECK(ret);
}


static bool lcd_vsync_event(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *event_data, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;

    if (xSemaphoreTakeFromISR(sem_gui_ready, &high_task_awoken) == pdTRUE) {
        xSemaphoreGiveFromISR(sem_vsync_end, &high_task_awoken);
    }

    return high_task_awoken == pdTRUE;
}
static void lcd_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(2);
}
static void lvgl_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    esp_lcd_panel_handle_t panel_handle = lcd_handle;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;

    xSemaphoreGive(sem_gui_ready);
    xSemaphoreTake(sem_vsync_end, portMAX_DELAY);

    // pass the draw buffer to the driver
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, px_map);
    lv_display_flush_ready(disp);
}


static void lcd_lvgl_task(void *arg)
{
    for (;;) {
        _lock_acquire(&lvgl_api_lock);
        uint32_t wait_ms = lv_timer_handler();
        _lock_release(&lvgl_api_lock);
        vTaskDelay(pdMS_TO_TICKS(wait_ms ? wait_ms : 1));
    }
}

static void lcd_initialize() {

    sem_vsync_end = xSemaphoreCreateBinary();
    assert(sem_vsync_end);
    sem_gui_ready = xSemaphoreCreateBinary();
    assert(sem_gui_ready);

    #if LCD_PIN_NUM_BCKL >= 0
    gpio_config_t bk_gpio_config;
    memset(&bk_gpio_config,0,sizeof(gpio_config_t));
    bk_gpio_config.mode = GPIO_MODE_OUTPUT;
    bk_gpio_config.pin_bit_mask = 1ULL << LCD_PIN_NUM_BCKL;
    ESP_ERROR_CHECK(gpio_config((gpio_num_t)LCD_PIN_NUM_BCKL,&bk_gpio_config));
    gpio_set_level((gpio_num_t)LCD_PIN_NUM_BCKL, LCD_BCKL_OFF_LEVEL);
    #endif
    
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_rgb_panel_config_t panel_config;
    memset(&panel_config,0,sizeof(esp_lcd_rgb_panel_config_t));
     
        panel_config.data_width = 16; // RGB565 in parallel mode, thus 16bit in width
        //panel_config.dma_burst_size = 64;
        panel_config.num_fbs = 1,
        panel_config.clk_src = LCD_CLK_SRC_DEFAULT,
        panel_config.disp_gpio_num = -1,
        panel_config.pclk_gpio_num = LCD_PIN_NUM_CLK,
        panel_config.vsync_gpio_num = LCD_PIN_NUM_VSYNC,
        panel_config.hsync_gpio_num = LCD_PIN_NUM_HSYNC,
        panel_config.de_gpio_num = LCD_PIN_NUM_DE,
        panel_config.data_gpio_nums[0]=LCD_PIN_NUM_D00;
        panel_config.data_gpio_nums[1]=LCD_PIN_NUM_D01;
        panel_config.data_gpio_nums[2]=LCD_PIN_NUM_D02;
        panel_config.data_gpio_nums[3]=LCD_PIN_NUM_D03;
        panel_config.data_gpio_nums[4]=LCD_PIN_NUM_D04;
        panel_config.data_gpio_nums[5]=LCD_PIN_NUM_D05;
        panel_config.data_gpio_nums[6]=LCD_PIN_NUM_D06;
        panel_config.data_gpio_nums[7]=LCD_PIN_NUM_D07;
        panel_config.data_gpio_nums[8]=LCD_PIN_NUM_D08;
        panel_config.data_gpio_nums[9]=LCD_PIN_NUM_D09;
        panel_config.data_gpio_nums[10]=LCD_PIN_NUM_D10;
        panel_config.data_gpio_nums[11]=LCD_PIN_NUM_D11;
        panel_config.data_gpio_nums[12]=LCD_PIN_NUM_D12;
        panel_config.data_gpio_nums[13]=LCD_PIN_NUM_D13;
        panel_config.data_gpio_nums[14]=LCD_PIN_NUM_D14;
        panel_config.data_gpio_nums[15]=LCD_PIN_NUM_D15;

        memset(&panel_config.timings,0,sizeof(esp_lcd_rgb_timing_t));
        
        panel_config.timings.pclk_hz = LCD_PIXEL_CLOCK_HZ;
        panel_config.timings.h_res = LCD_HRES;
        panel_config.timings.v_res = LCD_VRES;
        panel_config.timings.hsync_back_porch = LCD_HSYNC_BACK_PORCH;
        panel_config.timings.hsync_front_porch = LCD_HSYNC_FRONT_PORCH;
        panel_config.timings.hsync_pulse_width = LCD_HSYNC_PULSE_WIDTH;
        panel_config.timings.vsync_back_porch = LCD_VSYNC_BACK_PORCH;
        panel_config.timings.vsync_front_porch = LCD_VSYNC_FRONT_PORCH;
        panel_config.timings.vsync_pulse_width = LCD_VSYNC_PULSE_WIDTH;
        panel_config.timings.flags.pclk_active_neg = true;
        panel_config.timings.flags.hsync_idle_low = false;
        panel_config.timings.flags.pclk_idle_high = LCD_CLK_IDLE_HIGH;
        panel_config.timings.flags.de_idle_high = LCD_DE_IDLE_HIGH;
        panel_config.timings.flags.vsync_idle_low = false;
        panel_config.flags.bb_invalidate_cache = true;
        panel_config.flags.disp_active_low = false;
        panel_config.flags.double_fb = false;
        panel_config.flags.no_fb = false;
        panel_config.flags.refresh_on_demand = false;
        panel_config.flags.fb_in_psram = true; // allocate frame buffer in PSRAM
        //panel_config.sram_trans_align = 4;
        //panel_config.psram_trans_align = 64;
        panel_config.num_fbs = 2;
        panel_config.bounce_buffer_size_px = LCD_HRES*(LCD_VRES/LCD_DIVISOR);
        ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
        lcd_handle = panel_handle;
        #if LCD_PIN_NUM_BCKL >= 0
        gpio_set_level((gpio_num_t)LCD_PIN_NUM_BCKL, LCD_BCKL_ON_LEVEL);
        #endif
        lv_init();
        // create a lvgl display
        lv_display_t *display = lv_display_create(LCD_HRES, LCD_VRES);
        
        // set color depth
        lv_display_set_color_format(display, LV_COLOR_FORMAT_RGB565);
        // create draw buffers
        void *buf1 = NULL;
        // allocate the draw buffer from PSRAM since internal RAM is limited
        size_t draw_buffer_sz = LCD_HRES * 50 * sizeof(lv_color16_t);
        buf1 = heap_caps_malloc(draw_buffer_sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        assert(buf1);
        // set LVGL draw buffers and partial mode
        lv_display_set_buffers(display, buf1, NULL, draw_buffer_sz, LV_DISPLAY_RENDER_MODE_PARTIAL);

                // set the callback which can copy the rendered image to an area of the display
        lv_display_set_flush_cb(display, lvgl_flush);
        esp_lcd_rgb_panel_event_callbacks_t cbs;
        memset(&cbs,0,sizeof(esp_lcd_rgb_panel_event_callbacks_t));
        cbs.on_vsync = lcd_vsync_event;
        ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(panel_handle, &cbs, display));
        // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
        esp_timer_create_args_t lvgl_tick_timer_args;
        memset(&lvgl_tick_timer_args,0,sizeof(esp_timer_create_args_t));
        lvgl_tick_timer_args.callback = &lcd_increase_lvgl_tick;
        lvgl_tick_timer_args.name = "lvgl_tick";
        esp_timer_handle_t lvgl_tick_timer = NULL;
        ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
        ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, 2 * 1000));
        xTaskCreate(lcd_lvgl_task, "lcd_task", 8192, NULL, 2, NULL);
    
        

        _lock_acquire(&lvgl_api_lock);
        ui_init();
        _lock_release(&lvgl_api_lock);
        
}
esp_lcd_touch_handle_t touch_handle = NULL;
static void my_input_read(lv_indev_t * indev, lv_indev_data_t * data)
{
    uint16_t x[5],y[5],s[5];
    uint8_t count;
    static bool was_pressed = false;
    
    if(esp_lcd_touch_get_coordinates(touch_handle,x,y,s,&count,5)) {
        data->point.x = x[0];
        data->point.y = y[0];
        data->state = LV_INDEV_STATE_PRESSED;
        
        // Log touch events (only on press, not while held)
        if(!was_pressed) {
            printf("Touch: X=%d Y=%d\n", x[0], y[0]);
            was_pressed = true;
        }
        return;
    }
    
    // Log touch release
    if(was_pressed) {
        printf("Touch released\n");
        was_pressed = false;
    }
    
    data->state = LV_INDEV_STATE_RELEASED;
}
static void touch_reset()
{
    i2c_master_dev_handle_t i2c=NULL;
    i2c_device_config_t dev_cfg;
    memset(&dev_cfg,0,sizeof(dev_cfg));
    dev_cfg.scl_speed_hz = 200*1000;
    dev_cfg.device_address = 0x24;
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    
    if(i2c_master_bus_add_device(s_i2c_bus, &dev_cfg,&i2c) == ESP_OK) {
        uint8_t write_buf = 0x01;
        i2c_master_transmit(i2c,&write_buf,1,100);
        i2c_master_bus_rm_device(i2c);
    }
    
    dev_cfg.device_address = 0x38;
    if(i2c_master_bus_add_device(s_i2c_bus, &dev_cfg,&i2c) == ESP_OK) {
        uint8_t write_buf = 0x2C;
        i2c_master_transmit(i2c,&write_buf,1,100);
        vTaskDelay(pdMS_TO_TICKS(100));
        
        gpio_set_level((gpio_num_t)GPIO_INPUT_IO_4, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        
        write_buf = 0x2E;
        i2c_master_transmit(i2c,&write_buf,1,100);
        vTaskDelay(pdMS_TO_TICKS(200));
        i2c_master_bus_rm_device(i2c);
    }
}


static void touch_initialize() {
    printf("Touch Init: Starting GT911 touch controller initialization...\n");
    
    // Configure GPIO4 as output for touch reset
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    
    touch_reset();
    
    esp_lcd_panel_io_i2c_config_t tio_cfg;
    esp_lcd_panel_io_handle_t tio_handle;
    esp_lcd_touch_config_t tp_cfg;
    i2c_master_bus_handle_t i2c_handle;
    
    esp_err_t ret = (s_i2c_bus ? ESP_OK : i2c_master_get_bus_handle(I2C_MASTER_NUM,&i2c_handle));
    if (s_i2c_bus) i2c_handle = s_i2c_bus;
    if (ret != ESP_OK) {
        printf("Touch Init: Failed to get I2C bus handle, error: 0x%x\n", ret);
        return;
    }
    
    memset(&tio_cfg,0,sizeof(tio_cfg));
    tio_cfg.dev_addr = ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS;
    tio_cfg.control_phase_bytes = 1;
    tio_cfg.dc_bit_offset = 0;
    tio_cfg.lcd_cmd_bits = 16;
    tio_cfg.lcd_param_bits = 0;
    tio_cfg.flags.disable_control_phase = 1;
    tio_cfg.flags.dc_low_on_data = 0;
    tio_cfg.on_color_trans_done = NULL;
    tio_cfg.scl_speed_hz = 200*1000;
    tio_cfg.user_ctx = NULL;
    
    ret = esp_lcd_new_panel_io_i2c(i2c_handle, &tio_cfg,&tio_handle);
    if (ret != ESP_OK) {
        printf("Touch Init: Failed to create I2C panel IO, error: 0x%x\n", ret);
        return;
    }
    
    memset(&tp_cfg,0,sizeof(tp_cfg));
    tp_cfg.x_max = LCD_HRES;
    tp_cfg.y_max = LCD_VRES;
    tp_cfg.rst_gpio_num = (gpio_num_t)-1;
    tp_cfg.int_gpio_num = (gpio_num_t)-1;
    tp_cfg.levels.reset = 0;
    tp_cfg.levels.interrupt = 0;
    tp_cfg.flags.swap_xy = 0;
    tp_cfg.flags.mirror_x = 0;
    tp_cfg.flags.mirror_y = 0;

    ret = esp_lcd_touch_new_i2c_gt911(tio_handle,&tp_cfg,&touch_handle);
    if (ret == ESP_OK) {
        printf("Touch Init: GT911 touch controller created successfully\n");
        lv_indev_t * indev = lv_indev_create();
        lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
        lv_indev_set_read_cb(indev, my_input_read);
        printf("Touch Init: Touch initialization completed successfully\n");
    } else {
        printf("Touch Init: GT911 failed (error: 0x%x) - continuing without touch\n", ret);
        touch_handle = NULL;
    }
}
extern "C" void app_main() {
    // Initialize USB CDC for serial output
    #if CONFIG_ESP_CONSOLE_USB_CDC
    esp_err_t ret = esp_usb_console_init();
    if (ret != ESP_OK) {
        return;
    }
    #endif
    
    // Initialize serial output
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
    
    printf("\n=== Waveshare ESP32-S3 4.3\" LCD Test ===\n");
    printf("ESP-IDF version: %d.%d.%d\n",ESP_IDF_VERSION_MAJOR,ESP_IDF_VERSION_MINOR,ESP_IDF_VERSION_PATCH);
    
    // Kick off heavy work in a dedicated task, keep main lean
    xTaskCreatePinnedToCore([](void*) {
        printf("Initializing I2C...\n");
        i2c_initialize();

        printf("Initializing LCD...\n");
        lcd_initialize();

        printf("Initializing Touch...\n");
        touch_initialize();

        printf("System ready! Touch the screen to test.\n");

        uint32_t loop_count = 0;
        for(;;) {
            if (touch_handle) {
                esp_lcd_touch_read_data(touch_handle);
            }
            // No lv_timer_handler() here – it's in lcd_lvgl_task

            if ((loop_count % 1000) == 0) {
                printf("System running... Loop: %lu\n", loop_count);
            }
            loop_count++;
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }, "ui_main", 12288, nullptr, 5, nullptr, 0);

    // Optionally delete main task (saves stack)
    vTaskDelete(nullptr);
}
