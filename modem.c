/*
 * modem.c
 *
 *  Created on: 2021-08-24
 *      Author: GDR
 */

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "modem.h"
#include "StringCommandParser.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "math.h"
#include "time.h"

//#define SYSTEM_SHUT_DOWN

extern void handle_error(void);

/*Modem global variables storage*/
modem_data_storage_t modem_data;

/*AT parser variables storage*/
extern TSCPHandler   SCPHandler;

/*Arduino UART imported variables*/
extern cyhal_uart_t ardu_uart;
extern uint8_t RxByte;

/*ADC DMA imported variables*/
extern uint16_t aADCdata[2];

/*RTC variables*/
#ifdef SYSTEM_SHUT_DOWN
extern cyhal_rtc_t rtc_obj;
#endif

/*Telit Portal Authentication Templates*/
const char fcmd_HTTPPOST[] ="POST /api HTTP/1.0\r\nHost: api-de.devicewise.com\r\nContent-Type: application/json\r\nContent-Length:";
const char fcmd_dW_auth[]  = "{\"auth\":{\"command\":\"api.authenticate\",\"params\":{\"appToken\":\"%s\",\"appId\":\"%s\",\"thingKey\":\"%s\"}}}";
const char fcmd_dw_post_auth[]  = "{\"auth\":{\"sessionId\":\"%s\"},";

/*Telit Portal Post Templates*/
const char fcmd_dw_post_p1[]  = "\"1\":{\"command\":\"property.publish\",\"params\":{\"thingKey\":\"%s\",\"key\":\"temperature\",\"value\":%.2f}},";
const char fcmd_dw_post_p2[]  = "\"2\":{\"command\":\"property.publish\",\"params\":{\"thingKey\":\"%s\",\"key\":\"pressure\",\"value\":%.2f}},";
const char fcmd_dw_post_p3[]  = "\"3\":{\"command\":\"property.publish\",\"params\":{\"thingKey\":\"%s\",\"key\":\"humidity\",\"value\":%.2f}},";
const char fcmd_dw_post_p4[]  = "\"4\":{\"command\":\"property.publish\",\"params\":{\"thingKey\":\"%s\",\"key\":\"mag_xy\",\"value\":%d}},";
const char fcmd_dw_post_p5[]  = "\"5\":{\"command\":\"property.publish\",\"params\":{\"thingKey\":\"%s\",\"key\":\"batt_voltage\",\"value\":%d}},";
const char fcmd_dw_post_p6[]  = "\"6\":{\"command\":\"property.publish\",\"params\":{\"thingKey\":\"%s\",\"key\":\"signal_level\",\"value\":%d}},";
const char fcmd_dw_post_p6end[]  = "\"6\":{\"command\":\"property.publish\",\"params\":{\"thingKey\":\"%s\",\"key\":\"signal_level\",\"value\":%d}}}";
const char fcmd_dw_post_p7[]  = "\"7\":{\"command\":\"location.publish\",\"params\":{\"thingKey\":\"%s\",\"lat\":%s,\"lng\":%s,\"altitude\":%s,\"heading\":%s,\"speed\":%s,\"fixType\":\"manual\",\"fixAcc\":%s}}}";

/*App ID and tokens*/
#warning "Please define a telit cloud application ID and application Token here"
const char telit_appID[] = "xxx";
const char telit_appToken[] = "yyy";

/*Global variables for Telit cloud uploads*/
char telit_sessionId[48];
char post_buff[CMD_BUFF_LENGTH];
char post_length[16];

/*SMS Global Variables*/
char sms_buff[100];
char incoming_number[20];
_Bool msg_sts = false;
const char phone_number[]  = "xxxxxxxx";
const char google_link[] ="http://maps.google.com/maps?q=%s,%s";

/*Modem Task Attributes*/
TaskHandle_t ModemTaskHandle = NULL;

/*Specific string copy function. Copies only digits and dots*/
static void strxcpy(char *dest, char *src)
{
    int limit;

    limit = 10;

    while(((*src >= '0') && (*src <= '9')) || (*src == '.') )
    {
        while(*src == ' ')
        {
            src++;
            limit--;

            if(limit <= 0)
            {
                break;
            }
        }

        *dest = *src;
        dest++;
        src++;
        limit--;

        if(limit <= 0)
        {
            break;
        }
    }
}

/*Converts latitude and longitude from NMEA to decimal */
static double nmea2dec(char *nmea, char type, char *dir)
{
    unsigned int idx, dot = 0;
    double dec = 0;
    for (idx=0; idx<strlen(nmea);idx++){
        if (nmea[idx] == '.')
        {
            dot = idx;
            break;
        }
    }

    if ((dot < 3) || (dot > 5))
    {
        return 0;
    }

    int dd;
    double mm;
    char cdd[5], cmm[10];
    memset(cdd, 0, sizeof(cdd));
    memset(cmm, 0, sizeof(cmm));
    if(type == 1)
    {
        strncpy(cdd, nmea, dot-2);
        strncpy(cmm, nmea+dot-2, 7);
    }
    if(type == 2)
    {
        strncpy(cdd, nmea, dot-2);
        strncpy(cmm, nmea+dot-2, 7);
    }

    dd = atoi(cdd);
    mm = atof(cmm);

    dec = dd + (mm/60);

    if (type == 1 && dec > MAX_LATITUDE)
        return 0;
    else if (type == 2 && dec > MAX_LONGITUDE)
        return 0;

    if (*dir == 'N' || *dir == 'E')
      return dec;
    else
      return -1 * dec;
}

void *memmem(const void *l, size_t l_len, const void *s, size_t s_len)
{
	const char *cur, *last;
	const char *cl = l;
	const char *cs = s;

	/* a zero length needle should just return the haystack */
	if (s_len == 0)
		return (void *)cl;

	/* "s" must be smaller or equal to "l" */
	if (l_len < s_len)
		return NULL;

	/* special case where s_len == 1 */
	if (s_len == 1)
		return memchr(l, *cs, l_len);

	/* the last position where its possible to find "s" in "l" */
	last = cl + l_len - s_len;

	for (cur = cl; cur <= last; cur++)
		if (cur[0] == cs[0] && memcmp(cur, cs, s_len) == 0)
			return (void *)cur;

	return NULL;
}

void SCP_Tick_Callback(void)
{
	SCP_Tick(10);
}

static inline _Bool ModemOn(uint16_t *pwr_level)
{
  /*Turn on power for LTE Modem*/
  if(*pwr_level < PWRONLVL)
  {
	/*Power Supply ON*/
	cyhal_gpio_write(ARDU_IO3, 0);
	vTaskDelay(pdMS_TO_TICKS(100));

	/*Modem ON*/
    cyhal_gpio_write(ARDU_IO8, 1);
    vTaskDelay(pdMS_TO_TICKS(5000));
    cyhal_gpio_write(ARDU_IO8, 0);
    if(*pwr_level >= PWRONLVL)
    {
      vTaskDelay(pdMS_TO_TICKS(1000));
      printf("MODEM POWER ON\n\r\n\r");
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
	  printf("MODEM POWER ON\n\r\n\r");
	  return true;
  }
}

static inline _Bool ModemOff(uint16_t *pwr_level)
{
  /*Turn off power for LTE Modem*/
  if(*pwr_level >= PWRONLVL)
  {
	cyhal_gpio_write(ARDU_IO8, 1);
	vTaskDelay(pdMS_TO_TICKS(3000));
    cyhal_gpio_write(ARDU_IO8, 0);
    vTaskDelay(pdMS_TO_TICKS(3000));

	/*Power Supply OFF*/
    cyhal_gpio_write(ARDU_IO3, 1);
    vTaskDelay(pdMS_TO_TICKS(5000));

    if(*pwr_level < PWRONLVL)
    {
    	printf("MODEM POWER OFF\n\r\n\r");
    	return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
	  printf("MODEM POWER OFF\n\r\n\r");
	  return true;
  }
}

uint32_t uart_send_buff(uint8_t *data_out, uint32_t size)
{
	return cyhal_uart_write_async(&ardu_uart, (void*) data_out, (size_t) size);
}

uint32_t uart_read_byte(uint8_t *pData)
{
	return cyhal_uart_read_async( &ardu_uart, (void *) pData, 1);
}

/*Incoming Call Function*/
static void IncomingCall(const char *pString)
{
    (void)pString;

    /*Reject the incoming call*/
    SCP_SendCommandWaitAnswer("ATH\r", "OK", 1000, 1);
}

/*Returns network registration status*/
static int32_t NetworkRegistrationCheck(void)
{
    char *result = NULL;
    int32_t ntwrk_stat = 0;
    uint8_t i;

    /*Check what type of network is currently in use*/
    result = SCP_SendCommandWaitAnswer("AT+COPS?\r\n", "OK", 500, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    /*We have response, lets look for the info in the receiver buffer*/
    if(result)
    {
        result = NULL;
        result = memmem((char*)SCPHandler.RxBuffer, CMD_BUFF_LENGTH, "+COPS:", sizeof("+COPS:")-1);

        /*Response found, lets look for the third (,) */
        for(i = 0; i < 3; i++)
        {
        	result = strchr(result, ',');
            if(!result)
            {
            	/*Something is wrong with a data?*/
            	return 0;
            }

            result++;
        }

        /*Read the result*/
        ntwrk_stat = atoi(result);
    }

    /*GSM Network*/
    if(ntwrk_stat == 0)
    {
    	result = NULL;

        /*Request network status info*/
        result = SCP_SendCommandWaitAnswer("AT+CREG?\r\n", "OK", 500, 5);
        vTaskDelay(pdMS_TO_TICKS(100));

        if(result)
        {
            result = NULL;
            result = memmem((char*)SCPHandler.RxBuffer, CMD_BUFF_LENGTH, "+CREG: ", sizeof("+CREG: ")-1);
            if(result)
            {
                result += 9;
                ntwrk_stat = atoi(result);
            }
        }
        else
        {
            return 0;
        }

        return ntwrk_stat;
    }


    /*NB-IoT/ LTE-M*/
    else if(ntwrk_stat == 8 || ntwrk_stat == 9)
    {
    	/*Request network status info*/
        result = SCP_SendCommandWaitAnswer("AT+CEREG?\r\n", "OK", 500, 5);
        vTaskDelay(pdMS_TO_TICKS(100));

        if(result)
        {
            result = NULL;
            result = memmem((char*)SCPHandler.RxBuffer, CMD_BUFF_LENGTH, "+CEREG: ", sizeof("+CEREG: ")-1);
            if(result)
            {
                result += 10;
                ntwrk_stat = atoi(result);
            }
        }
        else
        {
            return 0;
        }
    }

    return ntwrk_stat;
}

/*Returns Context status*/
static int32_t ContextStatusCheck(void)
{
    char *result = NULL;
    int32_t lte_stat = 0;

    /*Request network status info*/
    result = SCP_SendCommandWaitAnswer("AT#SGACT?\r\n", "OK", 1000, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    if(result)
    {
        result = NULL;
        result = memmem((char*)SCPHandler.RxBuffer, CMD_BUFF_LENGTH, "#SGACT: 1", sizeof("#SGACT: 1")-1);
        if(result)
        {
            result += 10;
            lte_stat = atoi(result);
        }
    }
    else
    {
        return 0;
    }

    return lte_stat;
}

/*Returns received signal quality */
static int32_t SignalQuality(void)
{
    int32_t signal_level = 0;
    char *result = NULL;

    /*Request RSSI*/
    result = SCP_SendCommandWaitAnswer("AT+CSQ\r\n", "OK", 2000, 1);

    if(result)
    {
        result = NULL;
        result = memmem((char*)SCPHandler.RxBuffer, CMD_BUFF_LENGTH, "+CSQ:", sizeof("+CSQ:")-1);
        if(result)
        {
            result += 6;
            signal_level = atoi(result);
        }
    }
    else
    {
        return 0;
    }

    return signal_level;
}

/*Waits for network available, suspends actual task*/
static _Bool WaitForNetwork(void)
{
    int32_t test = 0;
    int signal = 0;

    /*Wait for network available*/
    for(uint32_t i=0; i < WAIT_FOR_NETWORK_S; i++)
    {
        /*Get current network state*/
        test = NetworkRegistrationCheck();

        /*registered, home network  or  registered, roaming is acceptable*/
        if((test == 1) || (test == 5))
        {
          /*Check the signal level*/
          signal = SignalQuality();
          if(signal != 99)
          {
            /*Turn off LED2*/
        	cyhal_gpio_write(LED2, CYBSP_LED_STATE_OFF);

            return true;
          }
          else
          {
            /* Wait 1000ms */
            vTaskDelay(pdMS_TO_TICKS(1000));
          }
        }
        else
        {
            /* Wait 1000ms */
        	vTaskDelay(pdMS_TO_TICKS(1000));
        }

        /*Indicate network search with LED2*/
        cyhal_gpio_toggle(LED2);
    }

    /*Turn off LED2*/
    cyhal_gpio_write(LED2, CYBSP_LED_STATE_OFF);
    return false;
}

/*Context Activation, returns true and IP address if succeeded, must have 15 bytes allocated*/
static _Bool ContextActivation(char *ip_address)
{
    char *result = NULL;
    char *temp = NULL;
    int check = 0;

    temp = ip_address;

    /*Check if the module is registered.*/
    check = NetworkRegistrationCheck();
    if(!check)
    {
      WaitForNetwork();
    }

    /*Context Activation*/
    result = NULL;
    result = SCP_SendCommandWaitAnswer("AT#SGACT?\r\n", "#SGACT: 1,0", 1000, 1);
    if (!result) result = SCP_SendCommandWaitAnswer("AT#SGACT=1,0\r\n", "OK", 1000, 1);
    if (!result) return false;

    vTaskDelay(pdMS_TO_TICKS(1000));

    result = NULL;
    result = SCP_SendCommandWaitAnswer("AT#SGACT=1,1\r\n", "OK", 60000, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    if(result)
    {
            result = NULL;
            result = memmem((char*)SCPHandler.RxBuffer, CMD_BUFF_LENGTH, "#SGACT: ", sizeof("#SGACT: ")-1);
            result += 8;

            if(result)
            {
                memset(ip_address, 0x00, 15);

                /*Maximum 15 chars for IP address*/
                for(uint8_t i = 0; i < 15; i++)
                {
                    /*Skip the tags*/
                    if(*result == '"')
                    {
                        result++;
                    }

                    /*Not a number or dot in IP address shall be treated as error*/
                    if(!(*result > 47 && *result < 58) && !(*result == '.') && (!(*result == '\r')))
                    {
                        memset(ip_address, 0x00, 15);
                        return false;
                    }

                    /*The end of the string*/
                    if(*result == '\r')
                    {
                        break;
                    }

                    *temp = *result;
                    result++;
                    temp++;
                }
            }
            return true;
    }
    return false;
}

static _Bool ContextDeactivation(void)
{
  char *result = NULL;
  result = SCP_SendCommandWaitAnswer("AT#SGACT=1,0\r\n", "OK", 1000, 1);
  if(result)
  {
    return true;
  }
  else return false;
}


/*Returns pointer to operator string*/
static char* GetOperator(void)
{
    char *result = NULL;
    static char operator[17];

    /*Request operator*/
    result = SCP_SendCommandWaitAnswer("AT+COPS?\r\n", "OK", 30000, 1);

    /*We have response, lets look for the info in the receiver buffer*/
    if(result)
    {
        result = NULL;
        result = memmem((char*)SCPHandler.RxBuffer, CMD_BUFF_LENGTH, "+COPS:", sizeof("+COPS:")-1);

        /*Response found, lets look for operator string, begins with (") */
        if(result)
        {
            result = strchr(result, '"');

            /*Copy operator to the RAM and return*/
            if(result)
            {
                /*Clean static buffer*/
                memset(operator, 0x00, 17);

                /*Maximum 16 chars for operator initials allowed*/
                for(uint8_t i = 0; i < 16; i++)
                {
                    operator[i] = *result;
                    result++;

                    /*Last operator char?*/
                    if(*result == '"')
                    {
                        i++;
                        operator[i] = *result;
                        return operator;
                    }
                }

                return operator;
            }
        }
    }

    return NULL;
}

/*Returns pointer to IMEI string of 15 numbers*/
static char* GetIMEI(void)
{
    char *result = NULL;
    static char imei[16];
    _Bool isDigit = false;
    uint32_t j = 0, i=0;

    /*Request IMEI*/
    result = SCP_SendCommandWaitAnswer("AT+CGSN\r\n", "OK", 100, 1);

    /*We have response, lets look for the info in the receiver buffer*/
    if(result)
    {
        result = NULL;

        /*Lets look for a ASCII number...*/
        while((j < strlen((char*)SCPHandler.RxBuffer)) && (!isDigit))
        {
          if((SCPHandler.RxBuffer[j] > 47) && (SCPHandler.RxBuffer[j] < 58))
          {
              isDigit = true;
              result = (char*)&SCPHandler.RxBuffer[j];
              break;
          }

          j++;
        }

        /*First number of IMEI found, copy the number to the RAM and return */
        if(result)
        {
            memset(imei, 0x00, 16);

            /*Maximum 15 chars for IMEI is allowed*/
            for(i = 0; i < 15; i++)
            {
                /*Not a number in IMEI shall be treated as error*/
                if(!(*result > 47 && *result < 58))
                {
                    return NULL;
                }

                imei[i] = *result;

                result++;
            }

            return imei;
        }
    }

    return NULL;
}

/*Request model identification*/
static char* GetID(void)
{
    char *result = NULL;
    static char device_id[21];

    /*Request model identification*/
    result = SCP_SendCommandWaitAnswer("AT+CGMM\r\n", "OK", 100, 1);

    /*We have response, lets look for the info in the receiver buffer*/
    if(result)
    {
        result = NULL;
        /*Lets look for a (\n) char as the begining of device name*/
        result = strchr((char*)SCPHandler.RxBuffer, '\n');

        if(result)
        {
            result++;

            /*Copy operator to the RAM and return*/
            memset(device_id, 0x00, 21);

            /*Maximum 20 chars for model identification allowed*/
            for(uint8_t i = 0; i < 20; i++)
            {
                device_id[i] = *result;
                result++;

                /*Device_id end*/
                if(*result == '\r')
                {
                    return device_id;
                }
            }
        }

    }

    return NULL;
}

/*Returns pointer to firmware version*/
static char* GetVersion(void)
{
    char *result = NULL;
    static char version[16];
    uint32_t i=0;

    /*Request IMEI*/
    result = SCP_SendCommandWaitAnswer("AT+CGMR\r\n", "OK", 100, 1);

    /*We have response, lets look for the info in the receiver buffer*/
    if(result)
    {
        result = NULL;

        /*Lets look for a (\n) char as the begining of firmware version string*/
        result = strchr((char*)SCPHandler.RxBuffer, '\n');

        /*Copy string to RAM */
        if(result)
        {
            result++;
            memset(version, 0x00, 16);

            /*Maximum 15 chars limit*/
            for(i = 0; i < 15; i++)
            {

                version[i] = *result;
                result++;

                if(*result == '\r')
                {
                    break;
                }
            }

            return version;
        }
    }

    return NULL;
}

/*Open socket to address*/
static _Bool ModemOpenTcpSocket(char *pAddress, uint32_t port)
{
    char *result = NULL;
    memset(post_buff, 0, sizeof(post_buff));
    /* Form open socket command */
    sprintf(post_buff, "AT#SD=1,0,%d,\"%s\"\r", (int)port, pAddress);
    result = SCP_SendCommandWaitAnswer(post_buff, "CONNECT", 15000, 1);

    if(result)
    {
        return true;
    }

    return false;
}

/* Close socket */
static _Bool ModemCloseTcpSocket(void)
{
    char *result = NULL;
    /* Form close socket command */
    (void) SCP_SendCommandWaitAnswer("+++\r\n", "OK", 1000, 1);

    result = SCP_SendCommandWaitAnswer("AT#SH=1\r\n", "OK", 1000, 1);

    if(result)
    {
        return true;
    }

    return false;
}

/* Authentication */
static _Bool TelitPortalAuthenticate(void)
{
    char *result = NULL;
    int i = 0;
    char local_buff[1024];

    memset(post_buff, 0, sizeof(post_buff));
    memset(local_buff, 0, sizeof(local_buff));
    memset(post_length, 0, sizeof(post_length));
    memset(telit_sessionId, 0, sizeof(telit_sessionId));

    /* Form data for lenght calculation*/
    sprintf(local_buff, fcmd_dW_auth, telit_appToken, telit_appID, modem_data.imei);

    /* Get data length */
    sprintf(post_length, "%d\r\n\r\n", strlen(local_buff));

    /*Generate full HTTP post*/
    sprintf(post_buff, (char *)fcmd_HTTPPOST);
    strcat(post_buff,post_length);
    strcat(post_buff,local_buff);
    strcat(post_buff,"\r\n");

    /*Reset rx buffer for data reception*/
    SCP_InitRx();

    /* Send HTTP POST data */
    SCP_SendData(post_buff, strlen(post_buff));
    printf("\n\rAUTHENTICATE:\n\r%s\n\r", post_buff);

    /* Wait for full answer */
    result = SCP_WaitForAnswer("}}}", 60000);
    if (result)
    {
        /* Getting session id */
        result = NULL;
        result = strstr((char*)SCPHandler.RxBuffer, "sessionId\":\"");
        if(result)
        {
            result += strlen("sessionId\":\"");
            while ((*result != '\"')&& (*result != 0))
            {
                telit_sessionId[i++]=*(result++);
            }
            /* We should wait for the server to shut down the connection */
            result = NULL;
            result = SCP_WaitForAnswer("NO CARRIER", 30000);
            if(result)
            {
                ModemCloseTcpSocket();
                printf("SessionID: ");
                printf(telit_sessionId);
                printf("\n\r");
                return true;
            }
        }
    }
    ModemCloseTcpSocket();
    return false;
}

/*Post*/
static _Bool TelitPortalPostData(void)
{
  char *result = NULL;
  char local_buff[1024];
  static float temperature = 0, humidity = 0, pressure = 0;
  static int mag_xy = 0;
  int signal_level,batt_voltage;

  /*Dummy values*/
  temperature++;
  humidity++;
  pressure++;
  mag_xy++;

  /*Battery voltage*/
  batt_voltage = (int)(aADCdata[1]*ADC_VOLTAGE_COEFF);

  memset(post_buff, 0, sizeof(post_buff));
  memset(local_buff, 0, sizeof(local_buff));
  memset(post_length, 0, sizeof(post_length));

  /*Copy variables that might to change during data post*/
  signal_level = modem_data.signal;

  /*Reset rx buffer for data reception*/
  SCP_InitRx();

  /*If we have no GPS data, we have to exclude them from post*/
  if(
     strlen(modem_data.gps_latitude) == 0
       || strlen(modem_data.gps_longitude) == 0
         || strlen(modem_data.gps_altitude) == 0
           || strlen(modem_data.gps_course) == 0
             || strlen(modem_data.gps_speed) == 0
               || strlen(modem_data.gps_precision) == 0
                 )
  {

	/* Generate JSON post */
	memset(post_buff, 0, sizeof(post_buff));
	sprintf((char *)post_buff, fcmd_dw_post_auth, telit_sessionId);
	strcat(local_buff,post_buff);

	memset(post_buff, 0, sizeof(post_buff));
	sprintf((char *)post_buff, fcmd_dw_post_p1, modem_data.imei, temperature);
	strcat(local_buff,post_buff);

	memset(post_buff, 0, sizeof(post_buff));
	sprintf((char *)post_buff, fcmd_dw_post_p2, modem_data.imei, pressure);
	strcat(local_buff,post_buff);

	memset(post_buff, 0, sizeof(post_buff));
	sprintf((char *)post_buff, fcmd_dw_post_p3, modem_data.imei, humidity);
	strcat(local_buff,post_buff);

	memset(post_buff, 0, sizeof(post_buff));
	sprintf((char *)post_buff, fcmd_dw_post_p4, modem_data.imei, mag_xy);
	strcat(local_buff,post_buff);

	memset(post_buff, 0, sizeof(post_buff));
	sprintf((char *)post_buff, fcmd_dw_post_p5, modem_data.imei, batt_voltage);
	strcat(local_buff,post_buff);

	memset(post_buff, 0, sizeof(post_buff));
	sprintf((char *)post_buff, fcmd_dw_post_p6end, modem_data.imei, signal_level);
	strcat(local_buff,post_buff);

    /*Generate HTTP post*/
	memset(post_buff, 0, sizeof(post_buff));
    sprintf(post_buff, (char *)fcmd_HTTPPOST);
    sprintf(post_length, "%d\r\n\r\n", strlen(local_buff));
    strcat(post_buff,post_length);
    strcat(post_buff,local_buff);
    strcat(post_buff,"\r\n");

    /* Send HTTP POST data */
    SCP_SendData((char *)post_buff, strlen(post_buff));
    printf("\n\rUPLOADING:\n\r%s\n\r", post_buff);
    vTaskDelay(pdMS_TO_TICKS(1000));

    result = SCP_WaitForAnswer("}}", 60000);
    if (result)
    {
      /* We should wait for the server to shut down the connection */
      result = NULL;
      result = SCP_WaitForAnswer("NO CARRIER", 30000);
      if(result)
      {
        ModemCloseTcpSocket();
        return true;
      }
    }
  }

  /*Include GNSS data*/
  else
  {
	  /* Generate JSON post */
	  memset(post_buff, 0, sizeof(post_buff));
	  sprintf((char *)post_buff, fcmd_dw_post_auth, telit_sessionId);
	  strcat(local_buff,post_buff);

	  memset(post_buff, 0, sizeof(post_buff));
	  sprintf((char *)post_buff, fcmd_dw_post_p1, modem_data.imei, temperature);
	  strcat(local_buff,post_buff);

	  memset(post_buff, 0, sizeof(post_buff));
	  sprintf((char *)post_buff, fcmd_dw_post_p2, modem_data.imei, pressure);
	  strcat(local_buff,post_buff);

	  memset(post_buff, 0, sizeof(post_buff));
	  sprintf((char *)post_buff, fcmd_dw_post_p3, modem_data.imei, humidity);
	  strcat(local_buff,post_buff);

	  memset(post_buff, 0, sizeof(post_buff));
	  sprintf((char *)post_buff, fcmd_dw_post_p4, modem_data.imei, mag_xy);
	  strcat(local_buff,post_buff);

	  memset(post_buff, 0, sizeof(post_buff));
	  sprintf((char *)post_buff, fcmd_dw_post_p5, modem_data.imei, batt_voltage);
	  strcat(local_buff,post_buff);

	  memset(post_buff, 0, sizeof(post_buff));
	  sprintf((char *)post_buff, fcmd_dw_post_p6, modem_data.imei, signal_level);
	  strcat(local_buff,post_buff);

	  memset(post_buff, 0, sizeof(post_buff));
	  sprintf((char *)post_buff,
            fcmd_dw_post_p7,
            modem_data.imei,
            modem_data.gps_latitude,
            modem_data.gps_longitude,
            modem_data.gps_altitude,
            modem_data.gps_course,
            modem_data.gps_speed,
            modem_data.gps_precision);
	  strcat(local_buff,post_buff);

	  /*Generate HTTP post*/
      memset(post_buff, 0, sizeof(post_buff));
	  sprintf(post_buff, (char *)fcmd_HTTPPOST);
	  sprintf(post_length, "%d\r\n\r\n", strlen(local_buff));
	  strcat(post_buff,post_length);
	  strcat(post_buff,local_buff);
	  strcat(post_buff,"\r\n");

	  /* Send HTTP POST data */
	  SCP_SendData((char *)post_buff, strlen(post_buff));
	  printf("\n\rUPLOADING:\n\r%s\n\r", post_buff);
	  vTaskDelay(pdMS_TO_TICKS(1000));

      result = SCP_WaitForAnswer("}}",60000);
      if (result)
      {
        /* We should wait for the server to shut down the connection */
        result = NULL;
        result = SCP_WaitForAnswer("NO CARRIER", 30000);
        if(result)
        {
          ModemCloseTcpSocket();
          return true;
        }
      }
  }

  /*Timeout. In case of error, no }} received*/
  ModemCloseTcpSocket();
  return false;
}

upload_error_t TelitCloudUpload(void)
{
  _Bool authenticated = false;
  _Bool result = false;
  char * scp_result = NULL;
  volatile upload_error_t return_error = UPLOAD_OK;

  printf("Telit Cloud Upload Procedure Started.\n\r");

  /*Apply power for IoT LTE module*/
  if(!ModemOn(&aADCdata[0]))
  {
    return_error = MODEM_POWER_ON_FAIL;
    goto error_exit;
  }

  /*Needs longer period to get ready?*/
  vTaskDelay(pdMS_TO_TICKS(5000));

  /*Start AT Commands*/
  scp_result = SCP_SendCommandWaitAnswer("AT\r\n", "OK", 200, 1);

  /*No response to AT, Retry*/
  if(!scp_result)
  {
	vTaskDelay(pdMS_TO_TICKS(5000));
    cyhal_uart_read_async(&ardu_uart, (void *)&RxByte, 1);
    scp_result = NULL;
    scp_result = SCP_SendCommandWaitAnswer("AT\r\n", "OK", 200, 1);
    if(!scp_result)
    {
      return_error = MODEM_CMD_NO_RESPONSE;
      goto error_exit;
    }
  }

  /*Modem is ON*/
  modem_data.mod_pwr_status = true;

  /*Check PIN*/
  if (scp_result) scp_result = SCP_SendCommandWaitAnswer("AT+CPIN?\r\n", "+CPIN: READY", 2000, 1);
  vTaskDelay(pdMS_TO_TICKS(100));

  /*Echo commands turn off*/
  if (scp_result) scp_result = SCP_SendCommandWaitAnswer("ATE0\r\n", "OK", 2000, 1);

  /*Select Text Mode SMS*/
  if (scp_result) scp_result = SCP_SendCommandWaitAnswer("AT+CMGF=1\r\n", "OK", 2000, 1);

  /*Sets LED Output mode*/
  if (scp_result) scp_result = SCP_SendCommandWaitAnswer("AT#SLED=4\r\n", "OK", 2000, 1);

  if(scp_result)
  {
    /*Get IMEI*/
    scp_result = NULL;
    scp_result = GetIMEI();
    if(scp_result)
    {
      memset(modem_data.imei, 0x00, 16);
      strcpy(modem_data.imei, scp_result);
      printf("IMEI:%s\n\r",modem_data.imei);
    }

    /*Get module type*/
    scp_result = NULL;
    scp_result = GetID();
    if(scp_result)
    {
      memset(modem_data.device_name, 0x00, 21);
      strcpy(modem_data.device_name, scp_result);
      printf("Device:%s\n\r",modem_data.device_name);
    }

    /*Get firmware version*/
    scp_result = NULL;
    scp_result = GetVersion();
    if(scp_result)
    {
      memset(modem_data.fw_version, 0x00, 16);
      strcpy(modem_data.fw_version, scp_result);
      printf("FW Version:%s\n\r",modem_data.fw_version);
    }
  }

  if(!scp_result)
  {
    return_error = MODEM_CMD_NO_RESPONSE;
    goto error_exit;
  }

  /*Set APN*/
#warning "Correct APN must be set."
  if (scp_result) scp_result = SCP_SendCommandWaitAnswer("AT+CGDCONT?\r\n", "omnitel", 1000, 1);
  if (!scp_result) scp_result = SCP_SendCommandWaitAnswer("AT+CGDCONT=1,\"IP\",\"omnitel\"\r\n", "OK", 2000, 1);
  //if (scp_result) scp_result = SCP_SendCommandWaitAnswer("AT+CGDCONT?\r\n", "lpwa.telia.iot", 1000, 1);
  //if (!scp_result) scp_result = SCP_SendCommandWaitAnswer("AT+CGDCONT=1,\"IP\",\"lpwa.telia.iot\"\r\n", "OK", 2000, 1);

  //if (scp_result) scp_result = SCP_SendCommandWaitAnswer("AT+WS46?\r\n", "+WS46: 28", 1000, 1);
  //if (!scp_result) scp_result = SCP_SendCommandWaitAnswer("AT+WS46=28\r\n", "OK", 1000, 1);

  //if (scp_result) scp_result = SCP_SendCommandWaitAnswer("AT+WS46?\r\n", "+WS46: 12", 1000, 1);
  //if (!scp_result) scp_result = SCP_SendCommandWaitAnswer("AT+WS46=12\r\n", "OK", 1000, 1);

  /*Network Support Setup*/
  if (scp_result) scp_result = SCP_SendCommandWaitAnswer("AT+WS46?\r\n", "+WS46: 28", 1000, 1);
  if (!scp_result) scp_result = SCP_SendCommandWaitAnswer("AT+WS46=28\r\n", "OK", 1000, 1);
  if (scp_result)
  {
    scp_result = SCP_SendCommandWaitAnswer("AT#WS46=3\r\n", "OK", 1000, 1);
    if(!scp_result)
    {
      return_error = MODEM_NET_SELECT_FAIL;
      goto error_exit;
    }
  }

  if (scp_result)
  {
    /*Check if we are connected to GSM network*/
    if(!WaitForNetwork())
    {
      return_error = MODEM_NO_OPERATOR_PRESENT;
      goto error_exit;
    }
  }
  else
  {
    return_error = MODEM_CMD_NO_RESPONSE;
    goto error_exit;
  }

  /*Store signal quality*/
  modem_data.signal = SignalQuality();

  /*Network status check*/
  modem_data.network_status =  NetworkRegistrationCheck();

  /*registered to home network or registered as roaming is acceptable*/
  if((modem_data.network_status == 1) || (modem_data.network_status == 5))
  {
    /*Store operator*/
    scp_result = NULL;
    scp_result = GetOperator();
    if(scp_result)
    {
      memset(modem_data.provider, 0x00, 17);
      strcpy(modem_data.provider, scp_result);
      printf("Operator: %s\n\r",modem_data.provider);
    }
    else
    {
      return_error = MODEM_NO_OPERATOR_PRESENT;
      goto error_exit;
    }

    /*Check LTE status*/
    modem_data.context = ContextStatusCheck();

    /*Try to connect if not connected*/
    if(!modem_data.context)
    {
      modem_data.context = ContextActivation(modem_data.ip_address);
      if(!modem_data.context)
      {
        ContextDeactivation();
        return_error = MODEM_NO_DATA_SERVICE;
        goto error_exit;
      }
      else
      {
    	  printf("IP Address:%s\n\r",modem_data.ip_address);
      }
    }
  }
  else
  {
    modem_data.context = 0;
    return_error = CLOUD_AUTH_ERROR;
    goto error_exit;
  }

  /*Start uploading*/
  cyhal_gpio_write(LED1, CYBSP_LED_STATE_ON);

  /*Authenticate*/
  result = ModemOpenTcpSocket("api-de.devicewise.com", 80);
  if(result)
  {
	  authenticated = TelitPortalAuthenticate();
  }

  vTaskDelay(pdMS_TO_TICKS(5000));

  /*Post*/
  if(authenticated)
  {
	  result = ModemOpenTcpSocket("api-de.devicewise.com", 80);
      if (result)
      {
        authenticated = TelitPortalPostData();
      }
  }

  /*Check the results*/
  if(!result || !authenticated)
  {
      ContextDeactivation();
      return_error = CLOUD_AUTH_ERROR;
      goto error_exit;
  }

  /*Turn off LED1*/
  cyhal_gpio_write(LED1, CYBSP_LED_STATE_OFF);
  /*Turn off Modem*/
  if(!ModemOff(&aADCdata[0]))
  {
    return_error = MODEM_POWER_OFF_FAIL;
    modem_data.mod_pwr_status = false;
    return return_error;
  }
  modem_data.mod_pwr_status = false;
  return return_error;

error_exit:
  /*Turn off LED1*/
  cyhal_gpio_write(LED1, CYBSP_LED_STATE_OFF);
  /*Turn off Modem*/
  if(!ModemOff(&aADCdata[0]))
  {
    return_error = MODEM_POWER_OFF_FAIL;
    modem_data.mod_pwr_status = false;
    return return_error;
  }
  modem_data.mod_pwr_status = false;
  return return_error;
}

static gnss_error_t GNSSFixLocation(uint32_t timeout, char*lat, char*lon, char*spd, char*cur, char*alt, char*acc)
{
	gnss_error_t return_error = GNSS_FIX_OK;
	char * result = NULL;
    char *dir = NULL;
    double coordinate = 0;
	uint8_t i;

	  /*Apply power for IoT LTE module*/
	  if(!ModemOn(&aADCdata[0]))
	  {
	    return_error = GNSS_POWER_ON_FAIL;
	    goto error_exit;
	  }

	  /*Start AT Commands*/
	  result = SCP_SendCommandWaitAnswer("AT\r\n", "OK", 200, 1);

	  /*No response to AT, Retry*/
	  if(!result)
	  {
		vTaskDelay(pdMS_TO_TICKS(5000));
	    cyhal_uart_read_async(&ardu_uart, (void *)&RxByte, 1);
	    result = NULL;
	    result = SCP_SendCommandWaitAnswer("AT\r\n", "OK", 200, 1);
	    if(!result)
	    {
	      return_error = GNSS_CMD_NO_RESPONSE;
	      goto error_exit;
	    }
	  }

	  /*Set GNSS priority*/
	  if (result) result = SCP_SendCommandWaitAnswer("AT$GPSCFG?\r\n", "$GPSCFG: 0", 1000, 1);
	  if (!result)
		  {
		  	SCP_SendCommandWaitAnswer("AT$GPSCFG=0,0\r\n", "OK", 1000, 1);
		  	SCP_SendCommandWaitAnswer("AT#REBOOT\r\n", "OK", 1000, 1);
		  	vTaskDelay(pdMS_TO_TICKS(5000));
		  	cyhal_uart_read_async(&ardu_uart, (void *)&RxByte, 1);
		    result = NULL;
		    result = SCP_SendCommandWaitAnswer("AT\r\n", "OK", 200, 1);
		    if(!result)
		    {
		      return_error = GNSS_CMD_NO_RESPONSE;
		      goto error_exit;
		    }
		  }

	  /*Turn the GNSS on*/
	  vTaskDelay(pdMS_TO_TICKS(100));
	  SCP_SendCommandWaitAnswer("AT$GPSNMUNEX=0,0,0,0,0,0,0,0,0,0,0,1,0\r\n", "OK", 1000, 1);
	  SCP_SendCommandWaitAnswer("AT$GPSNMUN=1,0,0,0,0,0,0\r\n", "OK", 1000, 1);
	  SCP_SendCommandWaitAnswer("AT$GPSP=1\r\n", "OK", 1000, 1);

	  printf("GNSS Receiver Started.\n\r");


	  /*Wait for location fix*/
	  while(timeout > 0)
	  {
		  result = NULL;
		  result = memmem((char*)SCPHandler.RxBuffer, CMD_BUFF_LENGTH, "$GNRMC", sizeof("$GNRMC")-1);
		  if(result)
		  {
			  /*Search for data valid indicator*/
		      for(i = 0; i < 2; i++)
		        {
		    	  result = strchr(result, ',');
		          if(!result)
		          {
		        	  /*Something is wrong with a data buffer?*/
		        	  break;
		          }

		          result++;
		        }

		      /*Check if data is valid*/
		      if(*result == 'A')
		      {
		    	  /*Just wait for a whole message to arrive*/
		    	  vTaskDelay(pdMS_TO_TICKS(50));

		    	  result +=2;
		    	  if(result)
		    	  {
		    		  /*Get the directory and calculate the latitude*/
		              dir = strchr(result, 'N');
		              if(!dir)
		              {
		                  dir = strchr(result, 'S');
		                  if(!dir)
		                  {
		                	  return_error = GNSS_NMEA_READ_ERROR;
		                	  goto error_exit;
		                  }
		              }
		              coordinate = nmea2dec(result, 1, dir);
		              sprintf(lat, "%.6f", coordinate);

		              result = dir;
		              result +=2;
		              if(result)
		              {
		            	  /*Get the directory and calculate the longitude*/
		            	  dir = strchr(result, 'E');
		                  if(!dir)
		                  {
		                      dir = strchr(result, 'W');
		                      if(!dir)
		                      {
			                	  return_error = GNSS_NMEA_READ_ERROR;
			                	  goto error_exit;
		                      }
		                  }
		                  coordinate = nmea2dec(result, 2, dir);
		                  sprintf(lon, "%.6f", coordinate);

			              result = dir;
			              result +=2;
			              if(result)
			              {
			            	  /*Get the speed and convert to km/h*/
			            	  coordinate = atof(result);
			            	  coordinate = coordinate * 1.8520;
			            	  sprintf(spd, "%.1f", coordinate);

			            	  result = strchr(result, ',');
			            	  result++;
			            	  if(result)
			            	  {
			            		  /*Copy the course*/
			            		  strxcpy(cur, result);
			            		  break;
			            	  }
			              }
		              }
		    	  }
		      }
		  }

		  /*Wait a second*/
		  timeout--;
		  vTaskDelay(pdMS_TO_TICKS(1000));
	  }

	if(timeout == 0) {goto error_exit;}

	printf("GNSS LOCATION FIXED\n\r");
    /*Some of the data is not used by this demo*/
    strcpy(alt, "0.0");
    strcpy(acc,"0.0");
    if(strlen(cur) == 0)
    {
  	  strcpy(cur,"0.0");
    }
	SCP_SendCommandWaitAnswer("AT$GPSNMUN=0\r\n", "OK", 1000, 1);
	SCP_SendCommandWaitAnswer("AT$GPSP=0\r\n", "OK", 1000, 1);
	printf("GNSS Receiver Stopped.\n\r");
	return return_error;

	error_exit:
	printf("GNSS NO DATA\n\r");
	SCP_SendCommandWaitAnswer("AT$GPSNMUN=0\r\n", "OK", 1000, 1);
	SCP_SendCommandWaitAnswer("AT$GPSP=0\r\n", "OK", 1000, 1);
	printf("GNSS Receiver Stopped.\n\r");
	return return_error;
}

#ifdef SEND_SMS
static upload_error_t SendSMS(void)
{
	char * scp_result = NULL;
	upload_error_t return_error = UPLOAD_OK;
    char local_buff[50];
    uint8_t i;

    printf("SMS Notification Procedure Started.\n\r");

    /*Apply power for IoT LTE module*/
    if(!ModemOn(&aADCdata[0]))
    {
      return_error = MODEM_POWER_ON_FAIL;
      goto error_exit;
    }

    /*Start AT Commands*/
    scp_result = SCP_SendCommandWaitAnswer("AT\r\n", "OK", 200, 1);

    /*No response to AT, Retry*/
    if(!scp_result)
    {
    	cyhal_uart_read_async(&ardu_uart, (void *)&RxByte, 1);
      for(i=0; i<50; i++)
      {
    	  vTaskDelay(pdMS_TO_TICKS(100));
          scp_result = NULL;
          scp_result = SCP_SendCommandWaitAnswer("AT\r\n", "OK", 100, 1);
          if(scp_result) break;
      }
      if(!scp_result)
      {
        return_error = MODEM_CMD_NO_RESPONSE;
        goto error_exit;
      }
    }

    /*Modem is ON*/
    modem_data.mod_pwr_status = true;

    /*Check PIN*/
    scp_result = SCP_SendCommandWaitAnswer("AT+CPIN?\r", "+CPIN: READY", 2000, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    /*Echo commands turn off*/
    if (scp_result) scp_result = SCP_SendCommandWaitAnswer("ATE0\r", "OK", 2000, 1);

    /*Select Text Mode SMS*/
    if (scp_result) scp_result = SCP_SendCommandWaitAnswer("AT+CMGF=1\r", "OK", 2000, 1);

    /*Sets LED Output mode*/
    if (scp_result) scp_result = SCP_SendCommandWaitAnswer("AT#SLED=4\r", "OK", 2000, 1);

    if(!scp_result)
    {
      return_error = MODEM_CMD_NO_RESPONSE;
      goto error_exit;
    }

    /*Network Support Setup*/
    if (scp_result) scp_result = SCP_SendCommandWaitAnswer("AT+WS46?\r\n", "+WS46: 30", 1000, 1);
    if (!scp_result) scp_result = SCP_SendCommandWaitAnswer("AT+WS46=30\r\n", "OK", 1000, 1);
    if (scp_result) scp_result = SCP_SendCommandWaitAnswer("AT#WS46?\r\n", "#WS46: 3", 1000, 1);
    if (!scp_result)
    {
      scp_result = SCP_SendCommandWaitAnswer("AT#WS46=3\r\n", "OK", 1000, 1);
      if(!scp_result)
      {
        return_error = MODEM_NET_SELECT_FAIL;
        goto error_exit;
      }
    }

    if (scp_result)
    {
      /*Check if we are connected to GSM network*/
      if(!WaitForNetwork())
      {
        return_error = MODEM_NO_OPERATOR_PRESENT;
        goto error_exit;
      }
    }
    else
    {
      return_error = MODEM_CMD_NO_RESPONSE;
      goto error_exit;
    }

    /*Store signal quality*/
    modem_data.signal = SignalQuality();

    /*Network status check*/
    modem_data.network_status =  NetworkRegistrationCheck();

    /*Registered to home network or registered as roaming is acceptable*/
    if((modem_data.network_status == 1) || (modem_data.network_status == 5))
    {
      /*Store operator*/
      scp_result = NULL;
      scp_result = GetOperator();
      if(scp_result)
      {
        memset(modem_data.provider, 0x00, 17);
        strcpy(modem_data.provider, scp_result);
        printf("Operator: %s\n\r",modem_data.provider);
      }
      else
      {
        return_error = MODEM_NO_OPERATOR_PRESENT;
        goto error_exit;
      }
    }

    /*Make first SMS*/
    memset(sms_buff, 0x00, sizeof(sms_buff));
    sprintf(sms_buff, "Device Activated!");
    memset(local_buff, 0x00, sizeof(local_buff));
    sprintf(local_buff, " Signal %d", (int)(modem_data.signal*100/31));
    strcat(local_buff, "%");
    strcat(sms_buff, local_buff);

    /*Termination symbol*/
    strcat(sms_buff, "\032");

    /*Send the SMS*/
    memset(local_buff, 0x00, sizeof(local_buff));
    sprintf(local_buff, "AT+CMGS=%s\r", phone_number);
    SCP_SendDoubleCommandWaitAnswer(local_buff, sms_buff, ">", "OK", 5000, 1);

    /*Wait for the message to be delivered*/
    vTaskDelay(pdMS_TO_TICKS(5000));

    /*Clear all old data*/
    memset(modem_data.gps_latitude,0x00,sizeof(modem_data.gps_latitude));
    memset(modem_data.gps_longitude,0x00,sizeof(modem_data.gps_longitude));
    memset(modem_data.gps_altitude,0x00,sizeof(modem_data.gps_altitude));
    memset(modem_data.gps_precision,0x00,sizeof(modem_data.gps_precision));
    memset(modem_data.gps_course,0x00,sizeof(modem_data.gps_course));
    memset(modem_data.gps_speed,0x00,sizeof(modem_data.gps_speed));

    /*Try to catch the GNSS signal*/
    GNSSFixLocation(60,modem_data.gps_latitude,modem_data.gps_longitude,modem_data.gps_speed,modem_data.gps_course, modem_data.gps_altitude, modem_data.gps_precision);

    /*Make second SMS*/
    if(strlen(modem_data.gps_latitude) != 0)
    {
        memset(sms_buff, 0x00, sizeof(sms_buff));
        memset(local_buff, 0x00, sizeof(local_buff));
        sprintf(sms_buff, google_link, modem_data.gps_latitude, modem_data.gps_longitude);
        /*Termination symbol*/
        strcat(sms_buff, "\032");
        /*Send the SMS*/
        sprintf(local_buff, "AT+CMGS=%s\r", phone_number);
        SCP_SendDoubleCommandWaitAnswer(local_buff, sms_buff, ">", "OK", 5000, 1);
    }

    /*Wait for the message to be delivered*/
    vTaskDelay(pdMS_TO_TICKS(5000));

    printf("SMS Notification Procedure Ended.\n\r");
    error_exit:
	return return_error;
}
#endif

void ModemTask(void *param)
{
	(void) param;
	static uint32_t cloud_time_counter = TELIT_CLOUD_UPDATE_INTERVAL;
	upload_error_t return_error;

	printf("Modem Task Started.\n\r");

#ifdef SYSTEM_SHUT_DOWN
	cy_rslt_t result;
	cyhal_gpio_write(LED1, CYBSP_LED_STATE_ON);
	vTaskDelay(pdMS_TO_TICKS(1000));
	cyhal_gpio_write(LED1, CYBSP_LED_STATE_OFF);
	result = Hibernate(&rtc_obj, 60);
    if (CY_RSLT_SUCCESS != result)
    {
    	printf("Failed to enter the system hibernation.\r\n");
        handle_error();
    }

    /*Clear all old data*/
    memset(modem_data.gps_latitude,0x00,sizeof(modem_data.gps_latitude));
    memset(modem_data.gps_longitude,0x00,sizeof(modem_data.gps_longitude));
    memset(modem_data.gps_altitude,0x00,sizeof(modem_data.gps_altitude));
    memset(modem_data.gps_precision,0x00,sizeof(modem_data.gps_precision));
    memset(modem_data.gps_course,0x00,sizeof(modem_data.gps_course));
    memset(modem_data.gps_speed,0x00,sizeof(modem_data.gps_speed));
    GNSSFixLocation(60,modem_data.gps_latitude,modem_data.gps_longitude,modem_data.gps_speed,modem_data.gps_course, modem_data.gps_altitude, modem_data.gps_precision);

    return_error = TelitCloudUpload();
  	while(return_error != UPLOAD_OK)
  	{
  		return_error = TelitCloudUpload();
  	}

  	/*Shut Down, system will Reset at wake-up*/
  	result = Hibernate(&rtc_obj, 60);
    if (CY_RSLT_SUCCESS != result)
    {
    	printf("Failed to enter the system hibernation.\r\n");
        handle_error();
    }
#endif

#ifdef SEND_SMS
  SendSMS();
#endif

  for(;;)
  {
    /* Wait 100ms */
	vTaskDelay(pdMS_TO_TICKS(100));

    /*Upload to Telit Cloud*/
    cloud_time_counter++;
    cyhal_gpio_toggle(LED1);
    if(cloud_time_counter >= TELIT_CLOUD_UPDATE_INTERVAL)
    {
      /*Reset counter now*/
      cloud_time_counter = 0;
      cyhal_gpio_write(LED1, CYBSP_LED_STATE_ON);

      /*Clear all old data*/
      memset(modem_data.gps_latitude,0x00,sizeof(modem_data.gps_latitude));
      memset(modem_data.gps_longitude,0x00,sizeof(modem_data.gps_longitude));
      memset(modem_data.gps_altitude,0x00,sizeof(modem_data.gps_altitude));
      memset(modem_data.gps_precision,0x00,sizeof(modem_data.gps_precision));
      memset(modem_data.gps_course,0x00,sizeof(modem_data.gps_course));
      memset(modem_data.gps_speed,0x00,sizeof(modem_data.gps_speed));

      /*Try to catch the GNSS signal, */
      GNSSFixLocation(240,modem_data.gps_latitude,modem_data.gps_longitude,modem_data.gps_speed,modem_data.gps_course, modem_data.gps_altitude, modem_data.gps_precision);

      printf("Latitude %s\n\r", modem_data.gps_latitude);
      printf("Longitude %s\n\r", modem_data.gps_longitude);
      printf("Speed %s\n\r", modem_data.gps_speed);
      printf("Course %s\n\r", modem_data.gps_course);

      /*Upload to cloud*/
      return_error = TelitCloudUpload();
      if(return_error == UPLOAD_OK)
      {
      	printf("UPLOAD OK\n\r\n\r");
      }
      else
      {
      	printf("UPLOAD ERROR\n\r\n\r");
      }
    }
  }
}

/*This task looks for Unsolicited Result Code and initializes String Command Parser*/
void URCReceiverTask(void *param)
{
    /*AT parser initialization*/
	SCP_Init(uart_send_buff, uart_read_byte);

	/*Reset AT Parser rx buffer */
	SCP_InitRx();

	/*AT Callback Register*/
    SCP_AddCallback("RING", IncomingCall);

    /*Start Modem Task*/
    xTaskCreate(ModemTask, "modem task", configMINIMAL_STACK_SIZE*64, NULL, configMAX_PRIORITIES - 3, &ModemTaskHandle);
    if(ModemTaskHandle == NULL)
    {
    	printf("Error: could not create main_task.\r\n");
    	handle_error();
    }

	for(;;)
	{
	    /* Wait 100ms */
		vTaskDelay(pdMS_TO_TICKS(100));

	    /*Look for URC received(Unsolicited Result Code)*/
	    SCP_Process();

	}
}



