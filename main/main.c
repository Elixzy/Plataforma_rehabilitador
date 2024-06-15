#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//librerias para los pines y PWM
#include "driver/gpio.h"
#include "driver/ledc.h"
//librerias para el wifi y esp-now
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "esp_mac.h"
//librerias del giroscopio y display que usan el protocolo I2C
#include "mpu6050.c"
#include "ssd1306.h"
#include "driver/i2c.h"
//etiqueta para logs en consola
#define TAG "esp"
//I2C master clock frequency
#define I2C_NUM_0              (0)
#define I2C_MASTER_FREQ_HZ 400000
static mpu6050_handle_t mpu6050 = NULL;
uint8_t peer_mac[6]={0xcc,0xdb,0xa7,0x49,0xdb,0x4c};//direccion mac del otro esp32
bool task_run=false;
bool adc_run=false;
bool x_or_y=false;
int adc_data=0;
char data_string[24];
//funcion donde se procesan los datos recibidos
void recv_cb(const esp_now_recv_info_t * esp_now_info, const uint8_t *data, int data_len){
	sprintf(data_string,"%s", data);
	//ESP_LOGI(TAG, "%s", data);
	if(strcmp(data_string, "STOP") == 0){
		task_run=false;
		adc_run=false;
	}
	else if(strcmp(data_string, "D0") == 0||strcmp(data_string, "D1") == 0
			||strcmp(data_string, "D2") == 0||strcmp(data_string, "D3") == 0){
		task_run=true;
	}
	else if(strcmp(data_string, "D4") == 0){
		adc_run=true;
		task_run=false;
		x_or_y=true;
	}
	else if(strcmp(data_string, "D5") == 0){
		adc_run=true;
		task_run=false;
		x_or_y=false;
	}
	if(adc_run){
		sprintf(data_string,"%s", data);
		adc_data=atoi(data_string);
		ESP_LOGI(TAG, "%d", adc_data);
	}
}
//funcion que se ejecuta el enviar datos
void send_cb(const uint8_t *mac_addr, esp_now_send_status_t status){
	if(status==ESP_NOW_SEND_SUCCESS){
		ESP_LOGI(TAG, "ESP_NOW_SEND_SUCCESS");
	}
	else{
		ESP_LOGW(TAG, "ESP_NOW_SEND_FAIL");
	}
	//se muestran en cosola si se enviaron correctamente los datos
}
//funcion para inicializar el wifi
void init_wifi(void){
	wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
	esp_netif_init();
	esp_event_loop_create_default();
	nvs_flash_init();
	esp_wifi_init(&wifi_init_config);
	esp_wifi_set_mode(WIFI_MODE_STA);
	esp_wifi_start();
	ESP_LOGI(TAG, " wifi init complete");
}
//funcion para incializar el esp-now
void init_esp_now(void){
	esp_now_init();
	esp_now_register_recv_cb(recv_cb);//se asignan las funciones que se ejecutaran al enviar o recibir datos
	esp_now_register_send_cb(send_cb);
	ESP_LOGI(TAG, " esp now init complete");
}
//funcion para inicializar la comunicacion I2C
void i2c_master_init_(SSD1306_t * dev, int16_t sda, int16_t scl, int16_t reset){
	//se configura en canal de I2C
	i2c_config_t i2c_config = {
		.mode = I2C_MODE_MASTER,//esp32 modo master
		.sda_io_num = sda,
		.scl_io_num = scl,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,//resistencias de pull up internas
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = I2C_MASTER_FREQ_HZ//frecuencia de 400kHz
	};
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_config));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

	if (reset >= 0) {
		//gpio_pad_select_gpio(reset);
		gpio_reset_pin(reset);
		gpio_set_direction(reset, GPIO_MODE_OUTPUT);
		gpio_set_level(reset, 0);
		vTaskDelay(50 / portTICK_PERIOD_MS);
		gpio_set_level(reset, 1);
	}
	dev->_address = 0x3C;
	dev->_flip = false;
}
//funcion para configurar la direccion mac del otro esp32
void register_peer(uint8_t *peer_addr){
	esp_now_peer_info_t esp_now_peer_info={};
	memcpy(esp_now_peer_info.peer_addr, peer_mac, 6);
	esp_now_peer_info.channel=1;
	esp_now_peer_info.ifidx=ESP_IF_WIFI_STA;
	esp_now_add_peer(&esp_now_peer_info);
	ESP_LOGI(TAG, "register peer init complete");
}
//funcion para enviar datos
void send_data(const uint8_t *data, size_t len) {
    // Enviar datos al dispositivo receptor
    esp_now_send(NULL, data, len);
    ESP_LOGI(TAG, "coso send complete");
}
//funcion para incializar el sensor giroscopio
void i2c_sensor_mpu6050_init(void){
	//se le manda el canal de I2C y la direccion de esclavo
    mpu6050 = mpu6050_create(I2C_NUM_0, MPU6050_I2C_ADDRESS);
    mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
    mpu6050_wake_up(mpu6050);
}
//tarea que se ejecutara en paralelo y controlará los servos
void servo_task(){
	//se configuran los canales de PWM
	ledc_channel_config_t ledc_channel2 = {
	    .speed_mode     = LEDC_LOW_SPEED_MODE,
	    .channel        = LEDC_CHANNEL_0,
	    .timer_sel      = LEDC_TIMER_0,
	    .intr_type      = LEDC_INTR_DISABLE,
	    .gpio_num       = 27,
	    .duty           = 614,
	    .hpoint         = 0
	};

	ledc_channel_config_t ledc_channel1 = {
	    .speed_mode     = LEDC_LOW_SPEED_MODE,
	    .channel        = LEDC_CHANNEL_1,
	    .timer_sel      = LEDC_TIMER_0,
	    .intr_type      = LEDC_INTR_DISABLE,
	    .gpio_num       = 26,
	    .duty           = 614,
	    .hpoint         = 0
	};
	//se configura el timer_0 con una frecuencia de 50Hz
	//y con una resolucion de duty cicle de 13 bits
	ledc_timer_config_t ledc_timer = {
	    .speed_mode       = LEDC_LOW_SPEED_MODE,
	    .timer_num        = LEDC_TIMER_0,
	    .duty_resolution  = LEDC_TIMER_13_BIT,
	    .freq_hz          = 50,
	    .clk_cfg          = LEDC_AUTO_CLK
	};
	//se incializan los canales PWM
	ledc_timer_config(&ledc_timer);
	ledc_channel_config(&ledc_channel1);
	ledc_channel_config(&ledc_channel2);
	while(1){
		//codigo para las rutinas de los servos
		if(adc_run){
			if(x_or_y){
				ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, adc_data+358);
				ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);}
			else if(!x_or_y){
				ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, adc_data+358);
				ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);}
		}
		//rutinas
		else if(task_run){
			//rutina 1
			if(strcmp(data_string, "D0")==0){
				for(int i=0;i<10;i++){
					for(int j=523;j<705;j++){
						ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, j);
						vTaskDelay(30/ portTICK_PERIOD_MS);
						ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
						if(!task_run){break;}}
					for(int j=705;j>523;j--){
						ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, j);
						vTaskDelay(30/ portTICK_PERIOD_MS);
						ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
						if(!task_run){break;}}}
					task_run=false;}
			//rutina 2
			if(strcmp(data_string, "D1")==0){
				for(int i=0;i<10;i++){
					for(int j=614;j<705;j++){
						ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, j);
						vTaskDelay(10/ portTICK_PERIOD_MS);
						ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
						if(!task_run){break;}}
					for(int j=705;j>614;j--){
						ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, j);
						vTaskDelay(10/ portTICK_PERIOD_MS);
						ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
						if(!task_run){break;}}}
					task_run=false;}
			//rutina 3
			if(strcmp(data_string, "D2")==0){
					for(int i=0;i<10;i++){
						for(int j=523;j<614;j++){
							ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, j);
							vTaskDelay(10/ portTICK_PERIOD_MS);
							ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
							if(!task_run){break;}}
						for(int j=614;j>523;j--){
							ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, j);
							vTaskDelay(10/ portTICK_PERIOD_MS);
							ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
							if(!task_run){break;}}}
					task_run=false;}
			//rutina 4
			if(strcmp(data_string, "D3")==0){
					for(int i=0;i<10;i++){
						for(int j=523;j<705;j++){
							ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, j);
							vTaskDelay(10/ portTICK_PERIOD_MS);
							ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
							if(!task_run){break;}}
						for(int j=705;j>523;j--){
							ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, j);
							vTaskDelay(10/ portTICK_PERIOD_MS);
							ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
							if(!task_run){break;}}}
					task_run=false;}
		}
		vTaskDelay(50/ portTICK_PERIOD_MS);
	}
}
//funcion principal
void app_main(void){
	SSD1306_t dev;
	//inicializacion de la cominicacion I2C
	i2c_master_init_(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
	//configuracion del display
	ssd1306_init(&dev, 128, 64);
	ssd1306_contrast(&dev, 0xff);
	ssd1306_clear_screen(&dev, false);
	uint8_t *buffer = (uint8_t *)malloc(1024);
	ssd1306_get_buffer(&dev, buffer);
	//iniciazlizacion del wifi
	init_wifi();
	init_esp_now();
	register_peer(peer_mac);

    uint8_t mpu6050_deviceid;
    mpu6050_acce_value_t acce;
    float deg;
    //inicializacion del sensor mpu6050
    i2c_sensor_mpu6050_init();
    char int_string[40];
    //se crea la tarea de los servos para que se ejecute en paralelo
    xTaskCreate(servo_task, "servo task", 1024*6, NULL, 2, NULL);
	while(1) {
		//se obtienen los datos del sensor
	    mpu6050_get_deviceid(mpu6050, &mpu6050_deviceid);
	    mpu6050_get_acce(mpu6050, &acce);
	    deg=acce.acce_x*90;
	    //se muestran los datos en el display
		ssd1306_set_buffer(&dev, buffer);
		sprintf(int_string,"x: %.1f", deg);
		ssd1306_display_text(&dev, 2, int_string, strlen(int_string), false);
		ssd1306_display_text(&dev, 4, data_string, strlen(data_string), false);
		ssd1306_show_buffer(&dev);
		vTaskDelay(250/ portTICK_PERIOD_MS);
		//se envian los datos de los angulos obtenidos del girioscopio al otro esp32
		send_data((const uint8_t *)int_string, strlen(int_string));
	}
}
