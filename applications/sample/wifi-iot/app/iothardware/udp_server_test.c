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

//udp部分
static char message[128] = "";
void UdpServerTest(unsigned short port)
{
    ColorfulLightDemo();
    ssize_t retval = 0;
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0); // UDP socket

    struct sockaddr_in clientAddr = {0};
    socklen_t clientAddrLen = sizeof(clientAddr);
    struct sockaddr_in serverAddr = {0};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);

    retval = bind(sockfd, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
    if (retval < 0)
    {
        printf("bind failed, %ld!\r\n", retval);
        goto do_cleanup;
    }
    printf("bind to port %d success!\r\n", port);

    while (1)
    {

        retval = recvfrom(sockfd, message, sizeof(message), 0, (struct sockaddr *)&clientAddr, &clientAddrLen);
        if (retval < 0)
        {
            printf("recvfrom failed, %ld!\r\n", retval);
            goto do_cleanup;
        }
        printf("recv message {%s} %ld done!\r\n", message, retval);
        printf("peer info: ipaddr = %s, port = %d\r\n", inet_ntoa(clientAddr.sin_addr), ntohs(clientAddr.sin_port));
        if (strncmp(message, "exit", 4) == 0)
        {
            strcpy(message, "exit!");
            retval = sendto(sockfd, message, strlen(message), 0, (struct sockaddr *)&clientAddr, sizeof(clientAddr));
            goto do_cleanup;
        }
        //unsigned2char
        snprintf(message, sizeof(message), "light: %u\n", data[1]); //float2str

        retval = sendto(sockfd, message, strlen(message), 0, (struct sockaddr *)&clientAddr, sizeof(clientAddr));
        if (retval <= 0)
        {
            printf("send failed, %ld!\r\n", retval);
            goto do_cleanup;
        }
        printf("send message {%s} %ld done!\r\n", message, retval);
        memset(message, 0, sizeof(message));
    }

    //end new

do_cleanup:
    printf("do_cleanup...\r\n");

    close(sockfd);
}

SERVER_TEST_DEMO(UdpServerTest);
