/*
 * modem.h
 *
 *  Created on: 2021-08-24
 *      Author: GDR
 */

#ifndef MODEM_H_
#define MODEM_H_

#define true							1
#define false							0
#define CMD_BUFF_LENGTH                 1024
#define SESSIONID_LENGTH                32
#define TELIT_CLOUD_UPDATE_INTERVAL     600     //time interval for modem properties
#define PWRONLVL                        1000    //power on indication ADC value
#define WAIT_FOR_NETWORK_S              300     //wait for network in seconds
#define ADC_VOLTAGE_COEFF               17.99056603773585
#define M_PI_2                          1.57079632679489661923 /* pi/2 */
#define M_PI                            3.14159265358979323846 /* pi */
//#define SYSTEM_SHUT_DOWN              1 //Test shut down mode
//#define SEND_SMS                		1 //Test sms delivery
#define BACKUP_REG_NO                   0
#define BACKUP_REG_VALUE                0xCAFEBABE
#define MAX_LONGITUDE               	180
#define MAX_LATITUDE                	90

typedef struct
{
    _Bool   mod_pwr_status;
    int network_status;
    int   context;
    int adc1;
    int dac;
    unsigned int gpio;
    int signal;
    char provider[17];
    char imei[16];
    char ip_address[16];
    char device_name[21];
    char fw_version[16];
    _Bool   gnss_online;
    char gps_sw_version[50];
    char gps_utc[16];
    char gps_date[16];
    char gps_latitude[16];
    char gps_longitude[16];
    char gps_altitude[16];
    char gps_precision[16];
    char gps_course[16];
    char gps_speed[16];
    char gps_satt_in_use[16];
} modem_data_storage_t;

typedef enum upload_errors
{
  UPLOAD_OK=0,
  MODEM_POWER_ON_FAIL,
  MODEM_POWER_OFF_FAIL,
  MODEM_CMD_NO_RESPONSE,
  MODEM_NET_SELECT_FAIL,
  MODEM_NO_OPERATOR_PRESENT,
  MODEM_NO_NETWORK,
  MODEM_NO_DATA_SERVICE,
  CLOUD_AUTH_ERROR

} upload_error_t;

typedef enum gnss_errors
{
  GNSS_FIX_OK=0,
  GNSS_FIX_TIMEOUT,
  GNSS_POWER_ON_FAIL,
  GNSS_POWER_OFF_FAIL,
  GNSS_CMD_NO_RESPONSE,
  GNSS_NMEA_READ_ERROR,

} gnss_error_t;

void ModemTask(void *param);
void URCReceiverTask(void *param);
void SCP_Tick_Callback(void);
void *memmem(const void *l, size_t l_len, const void *s, size_t s_len);
extern cy_rslt_t Hibernate(cyhal_rtc_t *obj, uint32_t seconds);

#endif /* MODEM_H_ */
