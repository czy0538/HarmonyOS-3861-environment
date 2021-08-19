/*
 * Copyright (c) 2020, HiHope Community.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "net_demo.h"
#include "net_common.h"

//合并的头文件
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_i2c.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_pwm.h"
#include "wifiiot_adc.h"
#include "wifiiot_errno.h"

#include "aht20.h"
#include "oled_ssd1306.h"

#include "MQTTPacket.h"
#include "transport.h"

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) sizeof(a) / sizeof(a[0])
#endif


#define MS_PER_S 1000

#define BEEP_TIMES 3
#define BEEP_DURATION 100
#define BEEP_PWM_DUTY 30000
#define BEEP_PWM_FREQ 60000
#define BEEP_PIN_NAME WIFI_IOT_IO_NAME_GPIO_9
#define BEEP_PIN_FUNCTION WIFI_IOT_IO_FUNC_GPIO_9_PWM0_OUT

#define GAS_SENSOR_CHAN_NAME WIFI_IOT_ADC_CHANNEL_5
// #define GAS_SENSOR_PIN_NAME WIFI_IOT_IO_NAME_GPIO_11

#define AHT20_BAUDRATE 400 * 1000
#define AHT20_I2C_IDX WIFI_IOT_I2C_IDX_0

#define ADC_RESOLUTION 2048

static int MQTT_PUBLISH_DELAY=1000;
float humidity = 0.0f;
float temperature = 0.0f;
float gasSensorResistance = 0.0f;

int mqtt_rc = 0;
int mqtt_sock = 0;
int mqtt_len = 0;
unsigned char mqtt_buf[200];
int mqtt_buflen = sizeof(mqtt_buf);
int mqtt_req_qos = 0;
int mqtt_msgid = 1;
int toStop = 0;
MQTTString topicString = MQTTString_initializer;
int connectedflag=0;

void mqtt_exit(void)
{
    transport_close(mqtt_sock);
    mqtt_rc = mqtt_rc;
    printf("[MQTT] ERROR EXIT\n");
}

//change light
void mqtt_onmessage(void){
	if (MQTTPacket_read(mqtt_buf, mqtt_buflen, transport_getdata) == PUBLISH){
			unsigned char dup;
			int qos;
			unsigned char retained;
			unsigned short msgid;
			int payloadlen_in;
			unsigned char* payload_in;
			int rc;
			MQTTString receivedTopic;
			rc = MQTTDeserialize_publish(&dup, &qos, &retained, &msgid, &receivedTopic,
					&payload_in, &payloadlen_in, mqtt_buf, mqtt_buflen);								// 接收数据
			printf("message arrived %.*s\n", payloadlen_in, payload_in);

            mqtt_rc = rc;
			if(strncmp("beep", (const char *)payload_in, strlen("open")) == 0)
			{
			    PwmStart(WIFI_IOT_PWM_PORT_PWM0, BEEP_PWM_DUTY, BEEP_PWM_FREQ);
                usleep(BEEP_DURATION * 1000);
                PwmStop(WIFI_IOT_PWM_PORT_PWM0);
                usleep((1000 - BEEP_DURATION) * 1000);
			}
			if(payloadlen_in>0&&payload_in[0]=='$')
			{
				char temp[256]="";
                int i=1;
                for(;i<payloadlen_in;++i)
                {
                    temp[i-1]=payload_in[i];
                }
                payload_in[i]='\0';
                if(atoi(temp)>0)
                {
                    MQTT_PUBLISH_DELAY=atoi(temp);
                }

			}
        }
}

void mqtt_publish(char* payload)
{

    int payloadlen = strlen(payload);

    if (MQTTPacket_read(mqtt_buf, mqtt_buflen, transport_getdata) == PUBLISH)
    {
        unsigned char dup;
        int qos;
        unsigned char retained;
        unsigned short msgid;
        int payloadlen_in;
        unsigned char *payload_in;
        int rc;
        MQTTString receivedTopic;
        rc = MQTTDeserialize_publish(&dup, &qos, &retained, &msgid, &receivedTopic,
                                        &payload_in, &payloadlen_in, mqtt_buf, mqtt_buflen); // 发送数据
        printf("message arrived %.*s\n", payloadlen_in, payload_in);

        mqtt_rc = rc;
    }

    printf("publishing reading\n");
    mqtt_len = MQTTSerialize_publish(mqtt_buf, mqtt_buflen, 0, 0, 0, 0, topicString, (unsigned char *)payload, payloadlen);
    mqtt_rc = transport_sendPacketBuffer(mqtt_sock, mqtt_buf, mqtt_len);

    osDelay(MQTT_PUBLISH_DELAY);

}

int mqtt_subscribe(char *topic)
{ // MQTT订阅
    /* subscribe */
    topicString.cstring = topic;
    mqtt_len = MQTTSerialize_subscribe(mqtt_buf, mqtt_buflen, 0, mqtt_msgid, 1, &topicString, &mqtt_req_qos); // MQTT订阅
    mqtt_rc = transport_sendPacketBuffer(mqtt_sock, mqtt_buf, mqtt_len);                                      // 传输发送缓冲区
    if (MQTTPacket_read(mqtt_buf, mqtt_buflen, transport_getdata) == SUBACK) /* wait for suback */            // 等待订阅返回
    {
        unsigned short submsgid;
        int subcount;
        int granted_qos;

        mqtt_rc = MQTTDeserialize_suback(&submsgid, 1, &subcount, &granted_qos, mqtt_buf, mqtt_buflen);
        if (granted_qos != 0)
        {
            printf("granted qos != 0, %d\n", granted_qos);
            mqtt_exit();
            return 0;
        }

        return 1;
    }
    else
    {
        mqtt_exit();
        return 0;
    }
}

int mqtt_init(void)
{ // MQTT初始化开始连接
    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    char *host = "broker-cn.emqx.io"; // 地址
    int port = 1883;                  // 端口

    mqtt_sock = transport_open(host, port);
    if (mqtt_sock < 0)
    {
        return mqtt_sock;
    }

    data.clientID.cstring = "hitclusterme"; // ClientID
    data.keepAliveInterval = 20;
    data.cleansession = 1;
    data.username.cstring = ""; // 用户名
    data.password.cstring = ""; // 密码

    printf("[MQTT]Sending to hostname %s port %d\n", host, port);

    mqtt_len = MQTTSerialize_connect(mqtt_buf, mqtt_buflen, &data);      // 开始连接
    mqtt_rc = transport_sendPacketBuffer(mqtt_sock, mqtt_buf, mqtt_len); // 发送缓冲区

    if (MQTTPacket_read(mqtt_buf, mqtt_buflen, transport_getdata) == CONNACK)
    { // 等待链接返回
        unsigned char sessionPresent, connack_rc;

        if (MQTTDeserialize_connack(&sessionPresent, &connack_rc, mqtt_buf, mqtt_buflen) != 1 || connack_rc != 0)
        {
            printf("Unable to connect, return code %d\n", connack_rc);
            mqtt_exit();
            return 0;
        }
    }
    else
    {
        mqtt_exit();
        return 0;
    }
    connectedflag=1;
    return 1;
}


float ConvertToVoltage(unsigned short data)
{
    return (float)data * 1.8 * 4 / 4096;
}

void EnvironmentTask(void *arg)
{
    (void)arg;
    uint32_t retval = 0;
    static char line[32] = {0};

    OledInit();
    OledFillScreen(0);
    I2cInit(AHT20_I2C_IDX, AHT20_BAUDRATE);

    // set BEEP pin as PWM function
    IoSetFunc(BEEP_PIN_NAME, BEEP_PIN_FUNCTION);
    GpioSetDir(BEEP_PIN_NAME, WIFI_IOT_GPIO_DIR_OUT);
    PwmInit(WIFI_IOT_PWM_PORT_PWM0);

    for (int i = 0; i < BEEP_TIMES; i++)
    {
        snprintf(line, sizeof(line), "beep %d/%d", (i + 1), BEEP_TIMES);
        OledShowString(0, 0, line, 1);

        PwmStart(WIFI_IOT_PWM_PORT_PWM0, BEEP_PWM_DUTY, BEEP_PWM_FREQ);
        usleep(BEEP_DURATION * 1000);
        PwmStop(WIFI_IOT_PWM_PORT_PWM0);
        usleep((1000 - BEEP_DURATION) * 1000);
    }

    while (WIFI_IOT_SUCCESS != AHT20_Calibrate())
    {
        printf("AHT20 sensor init failed!\r\n");
        usleep(1000);
    }

    while (1)
    {
        retval = AHT20_StartMeasure();
        if (retval != WIFI_IOT_SUCCESS)
        {
            printf("trigger measure failed!\r\n");
        }

        retval = AHT20_GetMeasureResult(&temperature, &humidity);
        if (retval != WIFI_IOT_SUCCESS)
        {
            printf("get humidity data failed!\r\n");
        }

        unsigned short data = 0;
        if (AdcRead(GAS_SENSOR_CHAN_NAME, &data, WIFI_IOT_ADC_EQU_MODEL_4, WIFI_IOT_ADC_CUR_BAIS_DEFAULT, 0) == WIFI_IOT_SUCCESS)
        {
            float Vx = ConvertToVoltage(data);

            // Vcc            ADC            GND
            //  |    ______   |     ______   |
            //  +---| MG-2 |---+---| 1kom |---+
            //       ------         ------
            // 查阅原理图，ADC 引脚位于 1K 电阻和燃气传感器之间，燃气传感器另一端接在 5V 电源正极上
            // 串联电路电压和阻止成正比：
            // Vx / 5 == 1kom / (1kom + Rx)
            //   => Rx + 1 == 5/Vx
            //   =>  Rx = 5/Vx - 1
            gasSensorResistance = 5 / Vx - 1;
        }

        //显示区
        OledShowString(0, 0, "Sensor values:", 1);

        snprintf(line, sizeof(line), "temp: %.2f", temperature); //float2str
        OledShowString(0, 1, line, 1);

        snprintf(line, sizeof(line), "humi: %.2f", humidity);
        OledShowString(0, 2, line, 1);

        snprintf(line, sizeof(line), "gas: %.2f kom", gasSensorResistance);
        OledShowString(0, 3, line, 1);

        sleep(1);
    }
}

void EnvironmentDemo_wifi(void)
{
    osThreadAttr_t attr;

    GpioInit();

    IoSetFunc(BEEP_PIN_NAME, BEEP_PIN_FUNCTION);
    GpioSetDir(BEEP_PIN_NAME, WIFI_IOT_GPIO_DIR_OUT);
    PwmInit(WIFI_IOT_PWM_PORT_PWM0);

    attr.name = "EnvironmentTask";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 4096;
    attr.priority = osPriorityNormal;

    if (osThreadNew(EnvironmentTask, NULL, &attr) == NULL)
    {
        printf("[EnvironmentDemo] Falied to create EnvironmentTask!\n");
    }
}

//end


void mqttreceTask(void){	// MQTT定时接收
	while(1){
		if(connectedflag==1){
			mqtt_onmessage();
		}

		osDelay(100);
	}
}

// mqtt接收
void mqttrec_Thread(void){
    osThreadAttr_t mqttrecattr;
    mqttrecattr.name = "mqttreceTask";
    mqttrecattr.attr_bits = 0U;
    mqttrecattr.cb_mem = NULL;
    mqttrecattr.cb_size = 0U;
    mqttrecattr.stack_mem = NULL;
    mqttrecattr.stack_size = 10240;
    mqttrecattr.priority =osPriorityNormal;
 
    if (osThreadNew((osThreadFunc_t)mqttreceTask, NULL, &mqttrecattr) == NULL) {
        printf("[mqttrec] Falied to create mqttreceTask!\n");
    }
}

static char message[128] = "";
void entry(void)
{
    EnvironmentDemo_wifi();//启动环境监测
    mqttrec_Thread();//read thread
    printf("[MQTT]Start MQTT\r\n");
    if (mqtt_init() == 1)
    {
        printf("[MQTT]MQTT Connect\r\n");
        mqtt_subscribe("substopic233"); //设置订阅

    }
    //new
    while (1)
    {
        //getEnvData
        char temp[32] = {0};
        snprintf(temp, sizeof(temp), "temp: %.2f\n", temperature); //float2str
        strcat(message, temp);
        snprintf(temp, sizeof(temp), "humi: %.2f\n", humidity);
        strcat(message, temp);
        snprintf(temp, sizeof(temp), "gas: %.2f kom", gasSensorResistance);
        strcat(message, temp);
        mqtt_publish(message);
        memset(message, 0, sizeof(message));
    }

    //end new


}

SERVER_TEST_DEMO(entry);
