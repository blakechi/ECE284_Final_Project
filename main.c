#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

// Simplelink includes
#include "simplelink.h"

//Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_apps_rcm.h"
#include "spi.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "utils.h"
#include "uart.h"

//Common interface includes
#include "common.h"
#include "uart_if.h"

// from heart rate dection
#include "timer_if.h"
#include "timer.h"
#include "pin_mux_config.h"

// from converting float to char
#include <math.h>

//*****************************************************************************
#define UART_PRINT              Report
#define MASTER_MODE      1

#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     6020

//*****************************************************************************
//                 GLOBAL VARIABLES
//*****************************************************************************
// heart rate detection
static unsigned char g_ucTxBuff[2];
static unsigned char g_ucRxBuff[2];
static unsigned short sample[TR_BUFF_SIZE];
static unsigned short sample_index;
static unsigned short smoothed_sample[TR_BUFF_SIZE];
static unsigned short sqw_ms[TR_BUFF_SIZE]; // used for square wave first, and then modified sample
static float HEART_RATE;
static short flag_heart_rate = 0;
static short flag_http_get = 0;

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

// IoT related
#define MAX_URI_SIZE 128
#define URI_SIZE MAX_URI_SIZE + 1

#define APPLICATION_NAME        "SSL"
#define APPLICATION_VERSION     "1.1.1.EEC.Fall2019"
#define SERVER_NAME             "apc3cc6yacpik-ats.iot.us-west-2.amazonaws.com"
#define GOOGLE_DST_PORT         8443

#define SL_SSL_CA_CERT "rootCA.der" //starfield class2 rootca (from firefox) // <-- this one works
#define SL_SSL_PRIVATE "private.der"
#define SL_SSL_CLIENT  "client.der"

// set time
#define DATE                6    /* Current Date */
#define MONTH               10     /* Month 1-12 */
#define YEAR                2019  /* Current year */
#define HOUR                10    /* Time - hours */
#define MINUTE              39    /* Time - minutes */
#define SECOND              0     /* Time - seconds */

#define POSTHEADER "POST /things/Cindy_CC3200Board/shadow HTTP/1.1\n\r"
#define GETHEADER "GET /things/Cindy_CC3200Board/shadow HTTP/1.1\n\r"
#define HOSTHEADER "Host: apc3cc6yacpik-ats.iot.us-west-2.amazonaws.com\r\n"
#define CHEADER "Connection: Keep-Alive\r\n"
#define CHEADER_GET "Connection: close\r\n"
#define CTHEADER "Content-Type: application/json; charset=utf-8\r\n"
#define CHEADER0 "x-amz-sns-topic-arn: arn:aws:sns:us-east-1:055734127750:SNS_TOPIC\r\n"
#define CLHEADER1 "Content-Length: "
#define CLHEADER2 "\r\n\r\n"

//Data before the message
#define DATA1_L "{\"state\": {\r\n\"desired\" : {\r\n\"var\" : \""
#define DATA1_R "\"\,\r\n\"Condition\" : \"Measured\"\r\n}}}\r\n\r\n"

// Application specific status/error codes
typedef enum{
    // LAN_CONNECTION_FAILED: avoid overlap with host-driver's error codes
    LAN_CONNECTION_FAILED = -0x7D0,
    INTERNET_CONNECTION_FAILED = LAN_CONNECTION_FAILED - 1,
    DEVICE_NOT_IN_STATION_MODE = INTERNET_CONNECTION_FAILED - 1,

    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

typedef struct
{
   // Time
   unsigned long tm_sec;
   unsigned long tm_min;
   unsigned long tm_hour;
   // Date
   unsigned long tm_day;
   unsigned long tm_mon;
   unsigned long tm_year;
   unsigned long tm_week_day;
   unsigned long tm_year_day;
   unsigned long reserved[3];
}SlDateTime;

// simple link
volatile unsigned long  g_ulStatus = 0;//SimpleLink Status
unsigned long  g_ulGatewayIP = 0; //Network Gateway IP address
unsigned char  g_ucConnectionSSID[SSID_LEN_MAX+1]; //Connection SSID
unsigned char  g_ucConnectionBSSID[BSSID_LEN_MAX]; //Connection BSSID
signed char    *g_Host = SERVER_NAME;
SlDateTime g_time;
#if defined(ccs) || defined(gcc)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

// local functions
static long WlanConnect();
static int set_time();
static void BoardInit(void);
static long InitializeAppVariables();
static int tls_connect();
static int connectToAccessPoint();
static int http_post(int);
static int http_get(int);


// SimpleLink Asynchronous Event Handlers
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent) {
    if(!pWlanEvent) {
        return;
    }

    switch(pWlanEvent->Event) {
        case SL_WLAN_CONNECT_EVENT: {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);

            //
            // Information about the connected AP (like name, MAC etc) will be
            // available in 'slWlanConnectAsyncResponse_t'.
            // Applications can use it if required
            //
            //  slWlanConnectAsyncResponse_t *pEventData = NULL;
            // pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
            //

            // Copy new connection SSID and BSSID to global parameters
            memcpy(g_ucConnectionSSID,pWlanEvent->EventData.
                   STAandP2PModeWlanConnected.ssid_name,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_len);
            memcpy(g_ucConnectionBSSID,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.bssid,
                   SL_BSSID_LENGTH);

            UART_PRINT("[WLAN EVENT] STA Connected to the AP: %s , "
                       "BSSID: %x:%x:%x:%x:%x:%x\n\r",
                       g_ucConnectionSSID,g_ucConnectionBSSID[0],
                       g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                       g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                       g_ucConnectionBSSID[5]);
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT: {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            // If the user has initiated 'Disconnect' request,
            //'reason_code' is SL_USER_INITIATED_DISCONNECTION
            if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code) {
                UART_PRINT("[WLAN EVENT]Device disconnected from the AP: %s,"
                    "BSSID: %x:%x:%x:%x:%x:%x on application's request \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            else {
                UART_PRINT("[WLAN ERROR]Device disconnected from the AP AP: %s, "
                           "BSSID: %x:%x:%x:%x:%x:%x on an ERROR..!! \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
            memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
        }
        break;

        default: {
            UART_PRINT("[WLAN EVENT] Unexpected event [0x%x]\n\r",
                       pWlanEvent->Event);
        }
        break;
    }
}

void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent) {
    if(!pNetAppEvent) {
        return;
    }

    switch(pNetAppEvent->Event) {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT: {
            SlIpV4AcquiredAsync_t *pEventData = NULL;

            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            //Ip Acquired Event Data
            pEventData = &pNetAppEvent->EventData.ipAcquiredV4;

            //Gateway IP address
            g_ulGatewayIP = pEventData->gateway;

            UART_PRINT("[NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , "
                       "Gateway=%d.%d.%d.%d\n\r",
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,3),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,2),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,1),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,0),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,3),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,2),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,1),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,0));
        }
        break;

        default: {
            UART_PRINT("[NETAPP EVENT] Unexpected event [0x%x] \n\r",
                       pNetAppEvent->Event);
        }
        break;
    }
}

void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent, SlHttpServerResponse_t *pHttpResponse) {
    // Unused in this application
}

void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent) {
    if(!pDevEvent) {
        return;
    }

    //
    // Most of the general errors are not FATAL are are to be handled
    // appropriately by the application
    //
    UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
               pDevEvent->EventData.deviceEvent.status,
               pDevEvent->EventData.deviceEvent.sender);
}

void SimpleLinkSockEventHandler(SlSockEvent_t *pSock) {
    if(!pSock) {
        return;
    }

    switch( pSock->Event ) {
        case SL_SOCKET_TX_FAILED_EVENT:
            switch( pSock->socketAsyncEvent.SockTxFailData.status) {
                case SL_ECLOSE:
                    UART_PRINT("[SOCK ERROR] - close socket (%d) operation "
                                "failed to transmit all queued packets\n\n",
                                    pSock->socketAsyncEvent.SockTxFailData.sd);
                    break;
                default:
                    UART_PRINT("[SOCK ERROR] - TX FAILED  :  socket %d , reason "
                                "(%d) \n\n",
                                pSock->socketAsyncEvent.SockTxFailData.sd, pSock->socketAsyncEvent.SockTxFailData.status);
                  break;
            }
            break;

        default:
            UART_PRINT("[SOCK EVENT] - Unexpected Event [%x0x]\n\n",pSock->Event);
          break;
    }
}

//
static long InitializeAppVariables() {
    g_ulStatus = 0;
    g_ulGatewayIP = 0;
    g_Host = SERVER_NAME;
    memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
    memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
    return SUCCESS;
}

static long ConfigureSimpleLinkToDefaultState() {
    SlVersionFull   ver = {0};
    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

    unsigned char ucVal = 1;
    unsigned char ucConfigOpt = 0;
    unsigned char ucConfigLen = 0;
    unsigned char ucPower = 0;

    long lRetVal = -1;
    long lMode = -1;

    lMode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(lMode);

    // If the device is not in station-mode, try configuring it in station-mode
    if (ROLE_STA != lMode) {
        if (ROLE_AP == lMode) {
            // If the device is in AP mode, we need to wait for this event
            // before doing anything
            while(!IS_IP_ACQUIRED(g_ulStatus)) {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask();
#endif
            }
        }

        // Switch to STA role and restart
        lRetVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Stop(0xFF);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(lRetVal);

        // Check if the device is in station again
        if (ROLE_STA != lRetVal) {
            // We don't want to proceed if the device is not coming up in STA-mode
            return DEVICE_NOT_IN_STATION_MODE;
        }
    }

    // Get the device's version-information
    ucConfigOpt = SL_DEVICE_GENERAL_VERSION;
    ucConfigLen = sizeof(ver);
    lRetVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &ucConfigOpt,
                                &ucConfigLen, (unsigned char *)(&ver));
    ASSERT_ON_ERROR(lRetVal);

    UART_PRINT("Host Driver Version: %s\n\r",SL_DRIVER_VERSION);
    UART_PRINT("Build Version %d.%d.%d.%d.31.%d.%d.%d.%d.%d.%d.%d.%d\n\r",
    ver.NwpVersion[0],ver.NwpVersion[1],ver.NwpVersion[2],ver.NwpVersion[3],
    ver.ChipFwAndPhyVersion.FwVersion[0],ver.ChipFwAndPhyVersion.FwVersion[1],
    ver.ChipFwAndPhyVersion.FwVersion[2],ver.ChipFwAndPhyVersion.FwVersion[3],
    ver.ChipFwAndPhyVersion.PhyVersion[0],ver.ChipFwAndPhyVersion.PhyVersion[1],
    ver.ChipFwAndPhyVersion.PhyVersion[2],ver.ChipFwAndPhyVersion.PhyVersion[3]);

    // Set connection policy to Auto + SmartConfig
    //      (Device's default connection policy)
    lRetVal = sl_WlanPolicySet(SL_POLICY_CONNECTION,
                                SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove all profiles
    lRetVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(lRetVal);

    //
    // Device in station-mode. Disconnect previous connection if any
    // The function returns 0 if 'Disconnected done', negative number if already
    // disconnected Wait for 'disconnection' event if 0 is returned, Ignore
    // other return-codes
    //
    lRetVal = sl_WlanDisconnect();
    if(0 == lRetVal) {
        // Wait
        while(IS_CONNECTED(g_ulStatus)) {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask();
#endif
        }
    }

    // Enable DHCP client
    lRetVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&ucVal);
    ASSERT_ON_ERROR(lRetVal);

    // Disable scan
    ucConfigOpt = SL_SCAN_POLICY(0);
    lRetVal = sl_WlanPolicySet(SL_POLICY_SCAN , ucConfigOpt, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Set Tx power level for station mode
    // Number between 0-15, as dB offset from max power - 0 will set max power
    ucPower = 0;
    lRetVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID,
            WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (unsigned char *)&ucPower);
    ASSERT_ON_ERROR(lRetVal);

    // Set PM policy to normal
    lRetVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Unregister mDNS services
    lRetVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove  all 64 filters (8*8)
    memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    lRetVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                       sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(lRetVal);

    lRetVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(lRetVal);

    InitializeAppVariables();

    return lRetVal; // Success
}

static long WlanConnect() {
    SlSecParams_t secParams = {0};
    long lRetVal = 0;

    secParams.Key = SECURITY_KEY;
    secParams.KeyLen = strlen(SECURITY_KEY);
    secParams.Type = SECURITY_TYPE;

    UART_PRINT("Attempting connection to access point: ");
    UART_PRINT(SSID_NAME);
    UART_PRINT("... ...");
    lRetVal = sl_WlanConnect(SSID_NAME, strlen(SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(lRetVal);

    UART_PRINT(" Connected!!!\n\r");

    // Wait for WLAN Event
    while((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus))) {
        // Toggle LEDs to Indicate Connection Progress
        _SlNonOsMainLoopTask();
        MAP_UtilsDelay(800000);
        _SlNonOsMainLoopTask();
        MAP_UtilsDelay(800000);
    }

    return SUCCESS;

}

long printErrConvenience(char * msg, long retVal) {
    UART_PRINT(msg);
    return retVal;
}

// This function updates the date and time of CC3200.
static int set_time() {
    long retVal;

    g_time.tm_day = DATE;
    g_time.tm_mon = MONTH;
    g_time.tm_year = YEAR;
    g_time.tm_sec = HOUR;
    g_time.tm_hour = MINUTE;
    g_time.tm_min = SECOND;

    retVal = sl_DevSet(SL_DEVICE_GENERAL_CONFIGURATION,
                          SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME,
                          sizeof(SlDateTime),(unsigned char *)(&g_time));

    ASSERT_ON_ERROR(retVal);
    return SUCCESS;
}

static int tls_connect() {
    SlSockAddrIn_t    Addr;
    int    iAddrSize;
    unsigned char    ucMethod = SL_SO_SEC_METHOD_TLSV1_2;
    unsigned int uiIP;
//    unsigned int uiCipher = SL_SEC_MASK_TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA;
    unsigned int uiCipher = SL_SEC_MASK_TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA256;
// SL_SEC_MASK_SSL_RSA_WITH_RC4_128_SHA
// SL_SEC_MASK_SSL_RSA_WITH_RC4_128_MD5
// SL_SEC_MASK_TLS_RSA_WITH_AES_256_CBC_SHA
// SL_SEC_MASK_TLS_DHE_RSA_WITH_AES_256_CBC_SHA
// SL_SEC_MASK_TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA
// SL_SEC_MASK_TLS_ECDHE_RSA_WITH_RC4_128_SHA
// SL_SEC_MASK_TLS_RSA_WITH_AES_128_CBC_SHA256
// SL_SEC_MASK_TLS_RSA_WITH_AES_256_CBC_SHA256
// SL_SEC_MASK_TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA256
// SL_SEC_MASK_TLS_ECDHE_ECDSA_WITH_AES_128_CBC_SHA256 // does not work (-340, handshake fails)
    long lRetVal = -1;
    int iSockID;

    lRetVal = sl_NetAppDnsGetHostByName(g_Host, strlen((const char *)g_Host),
                                    (unsigned long*)&uiIP, SL_AF_INET);

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't retrieve the host name \n\r", lRetVal);
    }

    Addr.sin_family = SL_AF_INET;
    Addr.sin_port = sl_Htons(GOOGLE_DST_PORT);
    Addr.sin_addr.s_addr = sl_Htonl(uiIP);
    iAddrSize = sizeof(SlSockAddrIn_t);
    //
    // opens a secure socket
    //
    iSockID = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, SL_SEC_SOCKET);
    if( iSockID < 0 ) {
        return printErrConvenience("Device unable to create secure socket \n\r", lRetVal);
    }

    //
    // configure the socket as TLS1.2
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_SECMETHOD, &ucMethod,\
                               sizeof(ucMethod));
    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }
    //
    //configure the socket as ECDHE RSA WITH AES256 CBC SHA
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_SECURE_MASK, &uiCipher,\
                           sizeof(uiCipher));
    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }



/////////////////////////////////
// START: COMMENT THIS OUT IF DISABLING SERVER VERIFICATION
    //
    //configure the socket with CA certificate - for server verification
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, \
                           SL_SO_SECURE_FILES_CA_FILE_NAME, \
                           SL_SSL_CA_CERT, \
                           strlen(SL_SSL_CA_CERT));

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }
// END: COMMENT THIS OUT IF DISABLING SERVER VERIFICATION
/////////////////////////////////


    //configure the socket with Client Certificate - for server verification
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, \
                SL_SO_SECURE_FILES_CERTIFICATE_FILE_NAME, \
                                    SL_SSL_CLIENT, \
                           strlen(SL_SSL_CLIENT));

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }

    //configure the socket with Private Key - for server verification
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, \
            SL_SO_SECURE_FILES_PRIVATE_KEY_FILE_NAME, \
            SL_SSL_PRIVATE, \
                           strlen(SL_SSL_PRIVATE));

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }

    /* connect to the peer device - Google server */
    lRetVal = sl_Connect(iSockID, ( SlSockAddr_t *)&Addr, iAddrSize);

    if(lRetVal >= 0) {
        UART_PRINT("Device has connected to the website:");
        UART_PRINT(SERVER_NAME);
        UART_PRINT("\n\r");
    }
    else if(lRetVal == SL_ESECSNOVERIFY) {
        UART_PRINT("Device has connected to the website (UNVERIFIED):");
        UART_PRINT(SERVER_NAME);
        UART_PRINT("\n\r");
    }
    else if(lRetVal < 0) {
        UART_PRINT("Device couldn't connect to server:");
        UART_PRINT(SERVER_NAME);
        UART_PRINT("\n\r");
        return printErrConvenience("Device couldn't connect to server \n\r", lRetVal);
    }

    return iSockID;
}

int connectToAccessPoint() {
    long lRetVal = -1;

    lRetVal = InitializeAppVariables();
    ASSERT_ON_ERROR(lRetVal);

    //
    // Following function configure the device to default state by cleaning
    // the persistent settings stored in NVMEM (viz. connection profiles &
    // policies, power policy etc)
    //
    // Applications may choose to skip this step if the developer is sure
    // that the device is in its default state at start of applicaton
    //
    // Note that all profiles and persistent settings that were done on the
    // device will be lost
    //
    lRetVal = ConfigureSimpleLinkToDefaultState();
    if(lRetVal < 0) {
      if (DEVICE_NOT_IN_STATION_MODE == lRetVal)
          UART_PRINT("Failed to configure the device in its default state \n\r");

      return lRetVal;
    }

    UART_PRINT("Device is configured in default state \n\r");

    CLR_STATUS_BIT_ALL(g_ulStatus);

    ///
    // Assumption is that the device is configured in station mode already
    // and it is in its default state
    //
    UART_PRINT("Opening sl_start\n\r");
    lRetVal = sl_Start(0, 0, 0);
    if (lRetVal < 0 || ROLE_STA != lRetVal) {
        UART_PRINT("Failed to start the device \n\r");
        return lRetVal;
    }

    UART_PRINT("Device started as STATION \n\r");

    //
    //Connecting to WLAN AP
    //
    lRetVal = WlanConnect();
    if(lRetVal < 0) {
        UART_PRINT("Failed to establish connection w/ an AP \n\r");
        return lRetVal;
    }

    UART_PRINT("Connection established w/ AP and IP is aquired \n\r");
    return 0;
}

//*****************************************************************************
//
// below is from heart rate detection
//
//*****************************************************************************

// Global variables used by the timer interrupt handler.
static volatile unsigned long g_ulBase;
unsigned long g_ulTimerInts;

// Flag that finishs the whole program after measuring
static unsigned short MEASURED = 1;

//
void create_square_wave(unsigned short period)
{
    unsigned short g;
    unsigned short print_num = 0;
    for(g = 0; g < sample_index; g++)
    {
        if(period < 2)
        {
            sqw_ms[g] = 1;
        }else
        {
            sqw_ms[g] = 0;
        }
        period++;
        if(period == 4)
        {
            period = 0;
        }
        if(print_num < 4)
        {
            Report("%d ", sqw_ms[g]);
            if(print_num == 3)
            {
                Report("\n\r");
            }
        }
        print_num++;
    }
}

//
int demodulate()
{
    unsigned short h;
    long sum = 0;
    for(h = 0; h < sample_index; h++)
    {
        sum += sqw_ms[h]*sample[h];
    }

    return (int) sum / sample_index;
}

// output
void create_marks_csv()
{

    Report("Sample_index = %d\n\r", sample_index);

    unsigned short i, j, k;

    // find the max sum of demodulated data
    unsigned short max = 0;
    unsigned short tmp = 0;
    unsigned short max_index = 0;
    for(i = 0; i < 4; i++)
    {
        create_square_wave(i);
        tmp = demodulate();
        Report("tmp = %d\n\r", tmp);
        if(tmp > max)
        {
            max = tmp;
            max_index = i;
        }
    }

    // create the phase-aligned square ware
    Report("Bits should shift = %d\n\r", max_index);

    create_square_wave(max_index);

    // demodulate
    Report("Demodulate...\n\r");
    for(j = 0; j < sample_index; j++)
    {
        sqw_ms[j] = sqw_ms[j]*sample[j];
    }

    // modify
    Report("Modify...\n\r");

    int max_value = 0;
    for(j = 0; j < sample_index; j++)
    {
        if(sample[j] > max_value)
        {
            max_value = sample[j];
        }
    }

    for(j = 0; j < sample_index; j++)
    {
        if(sqw_ms[j] < max_value - 100)
        {
            sqw_ms[j] = max_value - 100;
        }
    }

    // smooth
    Report("Smooth...\n\r");
    
    int sum = 0;
    for(j = 100; j < (sample_index - 100); j++)
    {
        for(k = 0; k < 201; k++)
        {
            sum = sum + sqw_ms[j - 100 + k];
        }
        smoothed_sample[j] = sum/201;
        sum = 0;
    }

    for(j = 0; j < 100; j++)
    {
        smoothed_sample[j] = smoothed_sample[100];
    }

    for(j = (sample_index - 100); j < sample_index; j++)
    {
        smoothed_sample[j] = smoothed_sample[(sample_index - 101)];
    }

    // calculate how many peeks we have
    Report("Calculate peeks...\n\r");
    short slope_num = 0;
    short slope_up_mark = 0;
    short slope_up_buffer = 0;
    short slope_down_buffer = 0;
    short up_index = 0, down_index = 0;
    short buffer_size = 4;
    for(i = 1; i < sample_index; i++)
    {
        if(smoothed_sample[i] - smoothed_sample[i - 1] > 0)
        {
            slope_up_buffer++;
            if(slope_up_buffer > buffer_size)
            {
                slope_up_buffer = buffer_size;
            }

            slope_down_buffer--;
            if(slope_down_buffer < 0)
            {
                slope_down_buffer = 0;
            }

            if(slope_up_buffer == buffer_size)
            {
                slope_up_mark = 1;
            }

        }else if(smoothed_sample[i] - smoothed_sample[i - 1] < 0)
        {
            slope_up_buffer--;
            if(slope_up_buffer < 0)
            {
                slope_up_buffer = 0;
            }

            slope_down_buffer++;
            if(slope_down_buffer > buffer_size)
            {
                slope_down_buffer = buffer_size;
            }

            if(slope_up_mark == 1 && slope_down_buffer == buffer_size)
            {
                slope_num = slope_num + 1;
                slope_up_mark = 0;

                if(slope_num == 2)
                {
                    up_index = i;
                }
                if(slope_num == 3)
                {
                    down_index = i;
                }
            }
        }
    }

    // calculate heart rate
    Report("Calculate heart rate...\n\r");
    float heart_rate;

    heart_rate = 60*((float)slope_num/5);
    HEART_RATE = (float)60*(((float)sample_index/(float)(down_index - up_index)))/5;
    
    Report("Heart Rate: %f\n\r", heart_rate);
    Report("Heart Rate (two peaks): %f\n\r", HEART_RATE);
    
    /*
    // output results to a .csv file
    Report("Ouput to output.csv file\n\r");

    FILE *fp;
    char *filename;
    filename = strcat("C:\\Users\\Jiuan\\Desktop\\output", ".csv");
    fp = fopen(filename, "w+");

    for(i = 0; i < sample_index; i++)
    {
        fprintf(fp, "%hu, %hu, %hu, %hu\n", (i + 1), sample[i], sqw_ms[i], smoothed_sample[i]);
    }

    fclose(fp);
    
    */
    Report("slope_num = %hu, up_index = %hu, down_index = %hu\n\r", slope_num, up_index, down_index);
}

// The interrupt handler for the first timer interrupt.
int TimerBaseIntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    Timer_IF_InterruptClear(g_ulBase);

    g_ulTimerInts++;
    
    // delay 1 sec
    MAP_UtilsDelay(16000000);
    //UTUtilsDelay(1000); // units: 1 micro sec
    
    Timer_IF_Stop(g_ulBase, TIMER_A);

    MEASURED = 0;
    flag_heart_rate = 1;

    return 0;
}

void MasterMain()
{
    //
    // Reset SPI
    //
    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVELOW |
                     SPI_WL_8));

    //
    // Enable SPI for communication
    //
    MAP_SPIEnable(GSPI_BASE);

    //
    // Initialize variable
    //
    sample_index = 0;
    unsigned short data; // modified (in while loop originally)
    
    // if SW2 pressed, then start to measure heart rate
    Report("Ready to measure heart rate.\n\r");
    while(flag_heart_rate == 0){
        if(GPIOPinRead(GPIOA2_BASE, 0x40)){
            //
            // Turn on the timers (mS)
            //
            Timer_IF_Start(g_ulBase, TIMER_A, 5000);

            while(MEASURED) {
                MAP_SPITransfer(GSPI_BASE, g_ucTxBuff, g_ucRxBuff, 4,
                                SPI_CS_ENABLE | SPI_CS_DISABLE);

                data = (unsigned short) ((g_ucRxBuff[0]<<8) | g_ucRxBuff[1]);
                data = data>>3;
                data = data & 0x3ff;

                sample[sample_index] = data;
                Report("%hu\n\r", sample[sample_index]);

                sample_index++;

                // delay 0.5ms
                // MAP_UtilsDelay spends 5 or 3 cycles (on RAM or ROM) if MAP_UtilsDelay(1)
                // clock_rate = 80000000
                // clock_rate / 5 = 1 sec (on RAM)
                MAP_UtilsDelay(6100);
                //UTUtilsDelay(1);
            }
        }
    }
}

// below is to convert heart rate in float to char

// Reverses a string 'str' of length 'len'
void reverse(char* str, int len)
{
    int i = 0, j = len - 1, temp;
    while (i < j) {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}
  
// Converts a given integer x to string str[].
// d is the number of digits required in the output.
// If d is more than the number of digits in x,
// then 0s are added at the beginning.
int intToStr(int x, char str[], int d)
{
    int i = 0;
    while (x) {
        str[i++] = (x % 10) + '0';
        x = x / 10;
    }
  
    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';
  
    reverse(str, i);
    str[i] = '\0';
    return i;
}
  
// Converts a floating-point/double number to a string.
void ftoa(float n, char* res, int afterpoint)
{
    // Extract integer part
    int ipart = (int)n;
  
    // Extract floating part
    float fpart = n - (float)ipart;
  
    // convert integer part to string
    int i = intToStr(ipart, res, 0);
  
    // check for display option after point
    if (afterpoint != 0) {
        res[i] = '.'; // add dot
  
        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter
        // is needed to handle cases like 233.007
        fpart = fpart * pow(10, afterpoint);
  
        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}

// boardInit
static void BoardInit(void) {
#ifndef USE_TIRTOS
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif

    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

//*****************************************************************************
// Main
//*****************************************************************************
int main() {
    long lRetVal = -1;
    long lRetVal_;
    BoardInit();

    PinMuxConfig();

    InitTerm();
    ClearTerm();
    UART_PRINT("It works!\n\r");

    lRetVal = connectToAccessPoint();
    lRetVal = set_time();
    if(lRetVal < 0) {
        UART_PRINT("Unable to set time in the device");
        LOOP_FOREVER();
    }
    lRetVal = tls_connect();
    if(lRetVal < 0) {
        ERR_PRINT(lRetVal);
    }
    
    lRetVal_ = lRetVal;
    // below is from heart rate detection
    //
    // Reset the peripheral
    //
    MAP_PRCMPeripheralReset(PRCM_GSPI);

    //
    // Base address for the timer
    //
    g_ulBase = TIMERA0_BASE;
    
    //
    // Configuring the timers
    //
    Timer_IF_Init(PRCM_TIMERA0, g_ulBase, TIMER_CFG_PERIODIC, TIMER_A, 0);

    //
    // Setup the interrupts for the timer timeouts.
    //
    Timer_IF_IntSetup(g_ulBase, TIMER_A, TimerBaseIntHandler);
    
#if MASTER_MODE
    
    MasterMain();
    Report("After MasterMain.\n\r");
#endif

    while(MEASURED){}
    Report("After MEASURED.\n\r");
    create_marks_csv();
    
    // above is from heart rate detection
    Timer_IF_Stop(g_ulBase, TIMER_A);
    http_post(lRetVal);

    // if SW3 pressed, then start to measure heart rate
    Report("Press SW3 to get from shadow.\n\r");
    while(flag_http_get == 0){
        if(GPIOPinRead(GPIOA1_BASE, 0x20)){
            http_get(lRetVal);
            flag_http_get = 1;
        }
    }
    sl_Stop(SL_STOP_TIMEOUT);
    LOOP_FOREVER();

    return 0;
}

// http post and get
static int http_post(int iTLSSockID){
    char acSendBuff[500];
    char cCLLength[200];
    char* pcBufHeaders;
    int lRetVal = 0;
    
    //maybe strcat?
    pcBufHeaders = acSendBuff;
    strcpy(pcBufHeaders, POSTHEADER);
    pcBufHeaders += strlen(POSTHEADER);
    strcpy(pcBufHeaders, HOSTHEADER);
    pcBufHeaders += strlen(HOSTHEADER);
    strcpy(pcBufHeaders, CHEADER);
    pcBufHeaders += strlen(CHEADER);
    strcpy(pcBufHeaders, "\r\n\r\n");
    
    char heart_rate_char[10];
    ftoa(HEART_RATE, heart_rate_char, 4);
    
    int dataLength = strlen(DATA1_L);
    dataLength += strlen(heart_rate_char);
    dataLength += strlen(DATA1_R);

    strcpy(pcBufHeaders, CTHEADER);
    pcBufHeaders += strlen(CTHEADER);
    strcpy(pcBufHeaders, CLHEADER1);

    pcBufHeaders += strlen(CLHEADER1);
    sprintf(cCLLength, "%d", dataLength);

    strcpy(pcBufHeaders, cCLLength);
    pcBufHeaders += strlen(cCLLength);
    strcpy(pcBufHeaders, CLHEADER2);
    pcBufHeaders += strlen(CLHEADER2);

    strcpy(pcBufHeaders, DATA1_L);
    pcBufHeaders += strlen(DATA1_L);
    strcpy(pcBufHeaders, heart_rate_char);
    pcBufHeaders += strlen(heart_rate_char);
    strcpy(pcBufHeaders, DATA1_R);
    pcBufHeaders += strlen(DATA1_R);

    //UART_PRINT(acSendBuff);
    
    //
    // Send the packet to the server */
    //
    
    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("POST failed. Error Number: %i\n\r",lRetVal);
        sl_Close(iTLSSockID);
        return lRetVal;
    }else {
        UART_PRINT("Post successfully sent.\n\r");
        UART_PRINT(acSendBuff);
        UART_PRINT("\n\r\n\r");
    }

    memset(acSendBuff, 0, sizeof(acSendBuff));
    lRetVal = sl_Recv(iTLSSockID, &acSendBuff[0], sizeof(acSendBuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("Post received failed. Error Number: %i\n\r",lRetVal);
        //sl_Close(iSSLSockID);
           return lRetVal;
    }else {
        UART_PRINT("Post successfully received.\n\r");
        acSendBuff[lRetVal+1] = '\0';
        UART_PRINT(acSendBuff);
        UART_PRINT("\n\r\n\r");
    }

    return 0;
}

static int http_get(int iTLSSockID){
    char get_acSendBuff[300];
    char get_acRecvbuff[500];
    char* pcBufHeaders;
    int lRetVal = 0;

    pcBufHeaders = get_acSendBuff;
    strcpy(pcBufHeaders, GETHEADER);
    pcBufHeaders += strlen(GETHEADER);
    strcpy(pcBufHeaders, HOSTHEADER);
    pcBufHeaders += strlen(HOSTHEADER);
    strcpy(pcBufHeaders, CTHEADER);
    pcBufHeaders += strlen(CTHEADER);
    strcpy(pcBufHeaders, CHEADER_GET);
    pcBufHeaders += strlen(CHEADER_GET);
    strcpy(pcBufHeaders, "\r\n\r\n");
    
    //
    // Send the packet to the server
    //
    UART_PRINT(get_acSendBuff);
    
    lRetVal = sl_Send(iTLSSockID, get_acSendBuff, strlen(get_acSendBuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("GET failed. Error Number: %i\n\r",lRetVal);
        sl_Close(iTLSSockID);
        return lRetVal;
    }else {
        UART_PRINT("GET successfully sent.\n\r");
    }
    
    lRetVal = sl_Recv(iTLSSockID, &get_acRecvbuff[0], sizeof(get_acRecvbuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("GET received failed. Error Number: %i\n\r",lRetVal);
        //sl_Close(iSSLSockID);
        return lRetVal;
    }
    else {
        UART_PRINT("GET successfully received.\n\r");
        get_acRecvbuff[lRetVal+1] = '\0';
        UART_PRINT(get_acRecvbuff);
        UART_PRINT("\n\r\n\r");
        }

    return 0;
    
}

