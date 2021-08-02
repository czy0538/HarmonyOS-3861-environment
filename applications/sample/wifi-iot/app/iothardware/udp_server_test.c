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
#include <unistd.h>

#include "net_demo.h"
#include "net_common.h"
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_pwm.h"
#include "wifiiot_adc.h"
#include "wifiiot_errno.h"

#include "MQTTPacket.h"
#include "transport.h"

//传感器部分
#define HUMAN_SENSOR_CHAN_NAME WIFI_IOT_ADC_CHANNEL_3
#define LIGHT_SENSOR_CHAN_NAME WIFI_IOT_ADC_CHANNEL_4
// #define HUMAN_SENSOR_PIN_NAME WIFI_IOT_IO_NAME_GPIO_7
// #define LIGHT_SENSOR_PIN_NAME WIFI_IOT_IO_NAME_GPIO_9

#define RED_LED_PIN_NAME WIFI_IOT_IO_NAME_GPIO_10
#define RED_LED_PIN_FUNCTION WIFI_IOT_IO_FUNC_GPIO_10_GPIO

#define GREEN_LED_PIN_NAME WIFI_IOT_IO_NAME_GPIO_11
#define GREEN_LED_PIN_FUNCTION WIFI_IOT_IO_FUNC_GPIO_11_GPIO

#define BLUE_LED_PIN_NAME WIFI_IOT_IO_NAME_GPIO_12
#define BLUE_LED_PIN_FUNCTION WIFI_IOT_IO_FUNC_GPIO_12_GPIO

#define LED_DELAY_TIME_US 300000
#define LED_BRIGHT WIFI_IOT_GPIO_VALUE1
#define LED_DARK WIFI_IOT_GPIO_VALUE0

#define NUM_LEDS 3
#define NUM_BLINKS 2
#define NUM_SENSORS 2

#define ADC_RESOLUTION 4096
#define PWM_FREQ_DIVITION 64000

unsigned short duty[NUM_SENSORS] = {0, 0}; //处理后pwm
unsigned short data[NUM_SENSORS] = {0, 0}; //光照


int mqtt_rc = 0;
int mqtt_sock = 0;
int mqtt_len = 0;
unsigned char mqtt_buf[200];
int mqtt_buflen = sizeof(mqtt_buf);
int mqtt_req_qos = 0;
int mqtt_msgid = 1;
int toStop = 0;
MQTTString topicString = MQTTString_initializer;

void mqtt_exit(void)
{
    transport_close(mqtt_sock);
    mqtt_rc = mqtt_rc;
    printf("[MQTT] ERROR EXIT\n");
}

void mqtt_task(char *payload)
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

    osDelay(100);
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

    return 1;
}

void CorlorfulLightTask(void *arg)
{
    (void)arg;
    static const WifiIotGpioIdx pins[] = {RED_LED_PIN_NAME, GREEN_LED_PIN_NAME, BLUE_LED_PIN_NAME};

    //闪两遍
    for (int i = 0; i < NUM_BLINKS; i++)
    {
        for (unsigned j = 0; j < sizeof(pins) / sizeof(pins[0]); j++)
        {
            GpioSetOutputVal(pins[j], LED_BRIGHT);
            usleep(LED_DELAY_TIME_US);
            GpioSetOutputVal(pins[j], LED_DARK);
            usleep(LED_DELAY_TIME_US);
        }
    }

    // GpioDeinit();，切换到pwm功能
    IoSetFunc(RED_LED_PIN_NAME, WIFI_IOT_IO_FUNC_GPIO_10_PWM1_OUT);
    IoSetFunc(GREEN_LED_PIN_NAME, WIFI_IOT_IO_FUNC_GPIO_11_PWM2_OUT);
    IoSetFunc(BLUE_LED_PIN_NAME, WIFI_IOT_IO_FUNC_GPIO_12_PWM3_OUT);

    PwmInit(WIFI_IOT_PWM_PORT_PWM1); // R
    PwmInit(WIFI_IOT_PWM_PORT_PWM2); // G
    PwmInit(WIFI_IOT_PWM_PORT_PWM3); // B

    // use PWM control BLUE LED brightness
    for (int i = 1; i <= ADC_RESOLUTION; i *= 2)
    {
        PwmStart(WIFI_IOT_PWM_PORT_PWM3, i, PWM_FREQ_DIVITION);
        usleep(250000);
        PwmStop(WIFI_IOT_PWM_PORT_PWM3);
    }

    while (1)
    {

        //chan[1]是光照传感器
        static const WifiIotAdcChannelIndex chan[] = {HUMAN_SENSOR_CHAN_NAME, LIGHT_SENSOR_CHAN_NAME};
        static const WifiIotPwmPort port[] = {WIFI_IOT_PWM_PORT_PWM1, WIFI_IOT_PWM_PORT_PWM2};

        //暂时关闭传感器
        for (size_t i = 1; i < sizeof(chan) / sizeof(chan[0]); i++)
        {
            //光照传感器值放入了data[1]
            //四次取平均
            if (AdcRead(chan[i], &data[i], WIFI_IOT_ADC_EQU_MODEL_4, WIFI_IOT_ADC_CUR_BAIS_DEFAULT, 0) == WIFI_IOT_SUCCESS)
            {
                duty[i] = PWM_FREQ_DIVITION * (unsigned int)data[i] / ADC_RESOLUTION;
            }
            PwmStart(port[i], duty[i], PWM_FREQ_DIVITION);
            usleep(10000);
            PwmStop(port[i]);
        }
        printf("data light sensor：%u\n", data[1]);
    }
}

void ColorfulLightDemo(void)
{
    osThreadAttr_t attr;

    GpioInit();

    // set Red/Green/Blue LED pin to GPIO function
    IoSetFunc(RED_LED_PIN_NAME, RED_LED_PIN_FUNCTION);
    IoSetFunc(GREEN_LED_PIN_NAME, GREEN_LED_PIN_FUNCTION);
    IoSetFunc(BLUE_LED_PIN_NAME, BLUE_LED_PIN_FUNCTION);

    // set Red/Green/Blue LED pin as output
    GpioSetDir(RED_LED_PIN_NAME, WIFI_IOT_GPIO_DIR_OUT);
    GpioSetDir(GREEN_LED_PIN_NAME, WIFI_IOT_GPIO_DIR_OUT);
    GpioSetDir(BLUE_LED_PIN_NAME, WIFI_IOT_GPIO_DIR_OUT);

    attr.name = "CorlorfulLightTask";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 4096;
    attr.priority = osPriorityNormal;

    if (osThreadNew(CorlorfulLightTask, NULL, &attr) == NULL)
    {
        printf("[ColorfulLightDemo] Falied to create CorlorfulLightTask!\n");
    }
}


static char message[128] = "";
void UdpServerTest(void)
{
    ColorfulLightDemo();//启动板子
    printf("[MQTT]Start MQTT\r\n");
    if (mqtt_init() == 1)
    {
        printf("[MQTT]MQTT Connect\r\n");
        mqtt_subscribe("substopic233"); //设置订阅
    }
    while (1)
    {
        snprintf(message, sizeof(message), "light: %u\n", data[1]); //float2str
        mqtt_task(message);
        memset(message, 0, sizeof(message));
    }

}

SERVER_TEST_DEMO(UdpServerTest);
