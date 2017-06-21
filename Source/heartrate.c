/**************************************************************************************************
  Filename:       heartrate.c
  Revised:        $Date $
  Revision:       $Revision $

  Description:    This file contains the heart rate sample application 
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

Copyright 2011 - 2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED ¡°AS IS¡± WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, 
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com. 
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OnBoard.h"
#include "hal_led.h"
#include "hal_key.h"
#include "linkdb.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "gatt_profile_uuid.h"
#include "heartrateservice.h"
#include "devinfoservice.h"
#include "battservice.h"
#include "peripheral.h"
#include "gapbondmgr.h"
#include "heartrate.h"

#include "hci.h"
#include "eegservice.h"
#include "hal_i2c.h"   

#include "TI_ADS1293.h"
#include "TI_CC254x.h"
#include "TI_CC254x_hardware_board.h"
#include "TI_CC254x_spi.h"
#include "TI_ADS1293_register_settings.h"

#include "ads1299.h"
#include "eegservice.h"

//haptic part
#include "Haptics_2605.h"
#include "DRV2605.h"

/*********************************************************************
 * MACROS
 */

// Convert BPM to RR-Interval for data simulation purposes
#define HEARTRATE_BPM_TO_RR(bpm)              ((uint16) 60 * 1024 / (uint16) (bpm))

/*********************************************************************
 * CONSTANTS
 */

// Fast advertising interval in 625us units
#define DEFAULT_FAST_ADV_INTERVAL             32

// Duration of fast advertising duration in ms
#define DEFAULT_FAST_ADV_DURATION             30000

// Slow advertising interval in 625us units
#define DEFAULT_SLOW_ADV_INTERVAL             1600

// Duration of slow advertising duration in ms (set to 0 for continuous advertising)
#define DEFAULT_SLOW_ADV_DURATION             0

// How often to perform heart rate periodic event
#define DEFAULT_HEARTRATE_PERIOD             100

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     200

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     1600

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         1

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Battery level is critical when it is less than this %
#define DEFAULT_BATT_CRITICAL_LEVEL           6 

// Battery measurement period in ms
#define DEFAULT_BATT_PERIOD                   15000

#define DEFAULT_ECG_PERIOD                    4

#define DEFAULT_HAPTIC_PERIOD                 200
   
// Some values used to simulate measurements
#define BPM_DEFAULT                           73
#define BPM_MAX                               80
#define ENERGY_INCREMENT                      10
#define FLAGS_IDX_MAX                         7

#define POWER_SAVING                          CC2540_PM2
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
#define BUF_SIZE 20   //12*3= 2*6*3 6*2ms *2 =24ms

uint8 dataBufX[BUF_SIZE];
uint8 dataBufY[BUF_SIZE];

uint8 rptr = 0;
uint8 wptr = 0;
uint8 *recv_buf = dataBufX;
uint8 *send_buf = dataBufX;
uint8 dataReadyFlag = 0;
int counter_ADS = 0;
int counter_BLE = 0;
int packet2enable = 0;

uint8 testspace[2];
uint8 status0;
uint8 status1;
uint8 status2; 
int status_value1 = 0;
int status_value2 = 0;

//HAPTIC PART
uint8 haptic_config = 0;
uint8 haptic_start = 0;
uint32 haptic_duration = 10; //10s
uint8 vibrate_period = 2; //200ms
extern uint8 vibration_amplitudeH; //0x10~0x30
/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 heartRate_TaskID;   // Task ID for internal task/event processing

static uint8 ecg_TaskID;

static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP Profile - Name attribute for SCAN RSP data
static uint8 scanData[] =
{
  0x0C,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'E',
  'E',
  'G',
  ' ',
  'M',
  'o',
  'n',
  'i',
  't',
  'o',
  'r'
};

static uint8 advertData[] = 
{ 
  // flags
  0x02,
  GAP_ADTYPE_FLAGS,
  GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
  // service UUIDs
  0x07,
  GAP_ADTYPE_16BIT_MORE,
  LO_UINT16(HEARTRATE_SERV_UUID),
  HI_UINT16(HEARTRATE_SERV_UUID),
  LO_UINT16(BATT_SERV_UUID),
  HI_UINT16(BATT_SERV_UUID),
  LO_UINT16(ECG_SERV_UUID),
  HI_UINT16(ECG_SERV_UUID),  
};

// Device name attribute value
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "EEG Monitor";

// GAP connection handle
static uint16 gapConnHandle;

// Heart rate measurement value stored in this structure
static attHandleValueNoti_t heartRateMeas;

static attHandleValueNoti_t ecgMeas;
// Components of heart rate measurement structure
//static uint8 heartRateBpm = BPM_DEFAULT;
static uint16 heartRateEnergy = 0;
//static uint16 heartRateRrInterval1 = HEARTRATE_BPM_TO_RR(BPM_DEFAULT);
//static uint16 heartRateRrInterval2 = HEARTRATE_BPM_TO_RR(BPM_DEFAULT);

// flags for simulated measurements
/*
static const uint8 heartRateFlags[FLAGS_IDX_MAX] =
{
  HEARTRATE_FLAGS_CONTACT_NOT_SUP,
  HEARTRATE_FLAGS_CONTACT_NOT_DET,
  HEARTRATE_FLAGS_CONTACT_DET | HEARTRATE_FLAGS_ENERGY_EXP,
  HEARTRATE_FLAGS_CONTACT_DET | HEARTRATE_FLAGS_RR,
  HEARTRATE_FLAGS_CONTACT_DET | HEARTRATE_FLAGS_ENERGY_EXP | HEARTRATE_FLAGS_RR,
  HEARTRATE_FLAGS_FORMAT_UINT16 | HEARTRATE_FLAGS_CONTACT_DET | HEARTRATE_FLAGS_ENERGY_EXP | HEARTRATE_FLAGS_RR,
  0x00
};
*/
static uint8 heartRateFlagsIdx = 0;

// Advertising user-cancelled state
static bool heartRateAdvCancelled = FALSE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void heartRate_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void HeartRateGapStateCB( gaprole_States_t newState );
static void heartRatePeriodicTask( void );
static void heartRateBattPeriodicTask( void );
static void heartRate_HandleKeys( uint8 shift, uint8 keys );
static void heartRateMeasNotify(void);
static void heartRateCB(uint8 event);
static void heartRateBattCB(uint8 event);

static void ecgCB(uint8 event);
static void ecgMeasNotify(void); 
static void ecgPeriodicTask( void );

static void hapticPeriodicTask();
void haptic_config_update(void);
/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t heartRatePeripheralCB =
{
  HeartRateGapStateCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller
};

// Bond Manager Callbacks
static const gapBondCBs_t heartRateBondCB =
{
  NULL,                   // Passcode callback
  NULL                    // Pairing state callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      HeartRate_Init
 *
 * @brief   Initialization function for the Heart Rate App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */

void HeartRate_Init( uint8 task_id )
{
  heartRate_TaskID = task_id;

  // Setup the GAP Peripheral Role Profile
  {
    // For the CC2540DK-MINI keyfob, device doesn't start advertising until button is pressed
    uint8 initial_advertising_enable = TRUE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;
      
    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );
    
    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanData ), scanData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
    
    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }
  
  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0; // passkey "000000"
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = FALSE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = TRUE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }  

  // Setup the Heart Rate Characteristic Values
  {
    uint8 sensLoc = HEARTRATE_SENS_LOC_WRIST;
    HeartRate_SetParameter( HEARTRATE_SENS_LOC, sizeof ( uint8 ), &sensLoc );
  }
  
  // Setup Battery Characteristic Values
  {
    uint8 critical = DEFAULT_BATT_CRITICAL_LEVEL;
    Batt_SetParameter( BATT_PARAM_CRITICAL_LEVEL, sizeof (uint8 ), &critical );
  }
  
  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );         // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes
  HeartRate_AddService( GATT_ALL_SERVICES );
  DevInfo_AddService( );
  Batt_AddService( );
  
  ecg_AddService( GATT_ALL_SERVICES );
    
  // Register for Heart Rate service callback
  HeartRate_Register( heartRateCB );
  
  ecg_Register(ecgCB);
  
  // Register for Battery service callback;
  Batt_Register ( heartRateBattCB );
  
  // Register for all key events - This app will handle all key events
  RegisterForKeys( heartRate_TaskID );
  
  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );
  
  // For keyfob board set GPIO pins into a power-optimized state
  // Note that there is still some leakage current from the buzzer,
  // accelerometer, LEDs, and buttons on the PCB.
  
  P0SEL = 0; // Configure Port 0 as GPIO
  P1SEL = 0; // Configure Port 1 as GPIO
  P2SEL = 0; // Configure Port 2 as GPIO

  P0DIR = 0xFC; // Port 0 pins P0.0 and P0.1 as input (buttons),
                // all others (P0.2-P0.7) as output
  P1DIR = 0xFF; // All port 1 pins (P1.0-P1.7) as output
  P2DIR = 0x1F; // All port 1 pins (P2.0-P2.4) as output
  P2DIR &= ~0x01; //P2.0 as input.
  
  P0 = 0x03; // All pins on port 0 to low except for P0.0 and P0.1 (buttons)
  P1 = 0;   // All pins on port 1 to low
  P2 = 0;   // All pins on port 2 to low  
  
  /* Enable ADC channel for battery measurement 
  APCFG = 0x80; // AIN7 as analog input
  P0DIR &= ~0x80; // force P0.7 to be input
  */
  //vishy
  P1DIR &= ~0x02; // force P1.1 as input: with Vlithium

//  Configure DRDYB (P1_7) from ADS1293 
  P1DIR &= ~0x80;                                                              // pin1.7 is input
  PICTL |= 0x04;                                                               // falling edge interrupt
  IRCON2 &= ~0x08;                                                             // clear Port 1 interrupt flag
  P1IFG &= ~0x80;                                                              // clear Port1.7 pin status flag
  P1IEN |= 0x80;                                                               // enable P1_7 interrupt
  IEN2  |= 0x10;                                                               // enable Port1 interrupt
/**/  
  delay_init();
  P1_0 = 1;
  ms_delay(2000);
  P1_0 = 0;
  ms_delay(2000);
  P1_0 = 1;

  //TI_ADS1293_SPISetup();                                                       // Initilaize CC254x SPI Block 
  //ads1299_set_up();      
  ads1299_powerdown();
  
  //haptic sensor
  //HalI2CInit( i2cClock_267KHZ );
  //Haptics_Init();
  
  
  hr_module_lowpower();
  //turn on overlapped processing
  HCI_EXT_HaltDuringRfCmd(HCI_EXT_HALT_DURING_RF_DISABLE);
  HCI_EXT_OverlappedProcessingCmd(HCI_EXT_ENABLE_OVERLAPPED_PROCESSING);
  
  // Setup a delayed profile startup
  osal_set_event( heartRate_TaskID, START_DEVICE_EVT );
  

}

/*********************************************************************
 * @fn      HeartRate_ProcessEvent
 *
 * @brief   Heart Rate Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 HeartRate_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function
  
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( heartRate_TaskID )) != NULL )
    {
      heartRate_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &heartRatePeripheralCB );  //did go

    // Register with bond manager after starting device
    GAPBondMgr_Register( (gapBondCBs_t *) &heartRateBondCB );
    
    return ( events ^ START_DEVICE_EVT );
  }

  if ( events & HEART_PERIODIC_EVT )
  {
    // Perform periodic heart rate task
    heartRatePeriodicTask();
    
    return (events ^ HEART_PERIODIC_EVT);
  }  

  if ( events & ECG_PERIODIC_EVT )
  {
    // Perform periodic heart rate task
      ecgPeriodicTask();
       
    return (events ^ ECG_PERIODIC_EVT);
  }  
  
  if ( events & BATT_PERIODIC_EVT )
  {
    // Perform periodic battery task
    heartRateBattPeriodicTask();
    
    return (events ^ BATT_PERIODIC_EVT);
  }  
  
  if ( events & HAPTIC_PERIODIC_EVT )
  {
    // Perform periodic battery task
    hapticPeriodicTask();
    
    return (events ^ HAPTIC_PERIODIC_EVT);
  }  
  
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      heartRate_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void heartRate_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case KEY_CHANGE:
      heartRate_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;
  }
}

/*********************************************************************
 * @fn      heartRate_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void heartRate_HandleKeys( uint8 shift, uint8 keys )
{
  if ( keys & HAL_KEY_SW_1 )
  {
    // set simulated measurement flag index
    if (++heartRateFlagsIdx == FLAGS_IDX_MAX)
    {
      heartRateFlagsIdx = 0;
    }
  }
  
  if ( keys & HAL_KEY_SW_2 )
  {
    // if not in a connection, toggle advertising on and off
    if( gapProfileState != GAPROLE_CONNECTED )
    {
      uint8 status;
      
      // Set fast advertising interval for user-initiated connections
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_FAST_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_FAST_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, DEFAULT_FAST_ADV_DURATION );

      // toggle GAP advertisement status
      GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &status );
      status = !status;
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &status );   
      
      // Set state variable
      if (status == FALSE)
      {
        heartRateAdvCancelled = TRUE;
      }
    }
  }
}

/*********************************************************************
 * @fn      heartRateMeasNotify
 *
 * @brief   Prepare and send a heart rate measurement notification
 *
 * @return  none
 */
static void heartRateMeasNotify(void)
{ 
  uint8 pointer_r; 
  uint8 pointer_w; 
  uint8 count = 9;      //to avoid wrong justification for ready data num.
  uint8 dataBuf_hr[64];
  uint8 dataBuf_hr2[16];
  uint8 buffer_w[2] = {0, 0};  //data to write in I2C
  
  buffer_w[0] = FIFO_W_POINTER;      
  HalI2CWrite(HR_ID, 1, buffer_w, 0);
  HalI2CRead(HR_ID, 1, &pointer_w);  
  buffer_w[0] = FIFO_R_POINTER;      
  HalI2CWrite(HR_ID, 1, buffer_w, 0);
  HalI2CRead(HR_ID, 1, &pointer_r);
  
  count = pointer_w - pointer_r;
  /*
  heartRateMeas.value[0] = 1;
  heartRateMeas.len = 16;
  HeartRate_MeasNotify( gapConnHandle, &heartRateMeas );
  
  */
  if(count < 8)     //Total buffer=16 samples. 8sample * 20ms = 160ms
  {  
    buffer_w[0] = FIFO_DATA;      
    HalI2CWrite(HR_ID, 1, buffer_w, 0);
    HalI2CRead(HR_ID, 32, &dataBuf_hr[0]);                                 // Read data   
    
    int i = 0;
    while(i < 16)
    {
      dataBuf_hr2[i] =  dataBuf_hr[2*i];
      dataBuf_hr2[i+1] =  dataBuf_hr[2*i + 1];
      i = i + 2;
    }
    osal_memcpy(&heartRateMeas.value[0], &dataBuf_hr2[0], 16);   //maximum size? 

    heartRateMeas.len = 16;
    HeartRate_MeasNotify( gapConnHandle, &heartRateMeas );
  }
  
}

/*********************************************************************
 * @fn      ecgMeasNotify
 *
 * @brief   Prepare and send a ecg measurement notification
 *
 * @return  none
 */

static void ecgMeasNotify(void)
{
  uint8 i;

  ecgMeas.len = 20;  //20 ,ark??
  /*
  ecgMeas.value[0] = 1;
  ecg_MeasNotify( gapConnHandle, &ecgMeas);
  */
    //----read data byte from spi
    
  /* 4 or 3 channel */
  
  if(dataReadyFlag == 1)
  {
    osal_memcpy(&ecgMeas.value[0], &send_buf[0], 20);
    if(ecg_MeasNotify( gapConnHandle, &ecgMeas) == SUCCESS)
    {
      dataReadyFlag = 0;           
      send_buf = NULL;      
    }
  }
   
/*   1 channel 
  if(dataReadyFlag == 1)
  {

    osal_memcpy(&ecgMeas.value[0], &send_buf[counter_BLE * 18], 18);   //maximum size?
     
    if(ecg_MeasNotify( gapConnHandle, &ecgMeas) == SUCCESS)
    {
      counter_BLE++;
      if(counter_BLE == 2)    //2*18=36Bytes  12 samples has been sent, then wait for new data filled
      {
        counter_BLE = 0;
        dataReadyFlag = 0;  
        send_buf = NULL;
      }
    }
  }
*/  
  
}

/*********************************************************************
 * @fn      HeartRateGapStateCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void HeartRateGapStateCB( gaprole_States_t newState )
{
  // if connected
  if (newState == GAPROLE_CONNECTED)
  {
    // get connection handle
    GAPRole_GetParameter(GAPROLE_CONNHANDLE, &gapConnHandle);
    
    //Start haptic sensor task.
    osal_start_timerEx( heartRate_TaskID, HAPTIC_PERIODIC_EVT, DEFAULT_HAPTIC_PERIOD );
    
    //ads start up and go to standby mode.
    TI_ADS1293_SPISetup();
    ads1299_set_up();
    ads1299_standby();
    
    //haptic sensor
    HalI2CInit( i2cClock_267KHZ );
    Haptics_Init();
  
    //Init the heart rate module
    // hr_module_init(); 
    //ads1299_set_up();
  }
  // if disconnected
  else if (gapProfileState == GAPROLE_CONNECTED && 
           newState != GAPROLE_CONNECTED)
  {
    uint8 advState = TRUE;

    // stop periodic measurement
    osal_stop_timerEx( heartRate_TaskID, HEART_PERIODIC_EVT );
    
    //stop haptic sensor task
    osal_stop_timerEx( heartRate_TaskID, HAPTIC_PERIODIC_EVT );
    
     //stop eeg task
    osal_stop_timerEx( heartRate_TaskID, ECG_PERIODIC_EVT );
    
     //stop eeg task
    osal_stop_timerEx( heartRate_TaskID, BATT_PERIODIC_EVT );
    
    //hr_module_stop();
    hr_module_lowpower();
    
    //power down ads
    TI_ADS1293_SPISetup();
    ads1299_biasoff();
    ads1299_standby();
    ads1299_powerdown();
    
    //stop haptic sensor.
    HalI2CInit( i2cClock_267KHZ );
    haptic_stop();
    haptic_shutdown();
    
    // reset client characteristic configuration descriptors
    HeartRate_HandleConnStatusCB( gapConnHandle, LINKDB_STATUS_UPDATE_REMOVED );
    Batt_HandleConnStatusCB( gapConnHandle, LINKDB_STATUS_UPDATE_REMOVED );

    if ( newState == GAPROLE_WAITING_AFTER_TIMEOUT )
    {
      // link loss timeout-- use fast advertising
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_FAST_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_FAST_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, DEFAULT_FAST_ADV_DURATION );
    }
    else
    {
      // Else use slow advertising
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_SLOW_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_SLOW_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, DEFAULT_SLOW_ADV_DURATION );
    }

    // Enable advertising
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advState );    
  }    
  // if advertising stopped
  else if ( gapProfileState == GAPROLE_ADVERTISING && 
            newState == GAPROLE_WAITING )
  {
    // if advertising stopped by user
    if ( heartRateAdvCancelled )
    {
      heartRateAdvCancelled = FALSE;
    }
    // if fast advertising switch to slow
    else if ( GAP_GetParamValue( TGAP_GEN_DISC_ADV_INT_MIN ) == DEFAULT_FAST_ADV_INTERVAL )
    {
      uint8 advState = TRUE;
      
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_SLOW_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_SLOW_ADV_INTERVAL );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, DEFAULT_SLOW_ADV_DURATION );
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advState );   
    }  
  }
  // if started
  else if (newState == GAPROLE_STARTED)
  {
    // Set the system ID from the bd addr
    uint8 systemId[DEVINFO_SYSTEM_ID_LEN];
    GAPRole_GetParameter(GAPROLE_BD_ADDR, systemId);
    
    // shift three bytes up
    systemId[7] = systemId[5];
    systemId[6] = systemId[4];
    systemId[5] = systemId[3];
    
    // set middle bytes to zero
    systemId[4] = 0;
    systemId[3] = 0;
    
    DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
  }
  
  gapProfileState = newState;
}

/*********************************************************************
 * @fn      heartRateCB
 *
 * @brief   Callback function for heart rate service.
 *
 * @param   event - service event
 *
 * @return  none
 */
static void heartRateCB(uint8 event)
{
  if (event == HEARTRATE_MEAS_NOTI_ENABLED)
  {

    if (gapProfileState == GAPROLE_CONNECTED)
    {
      //heart rate module initiation.
      // if connected start periodic measurement 
      hr_module_init();
         
      osal_start_timerEx( heartRate_TaskID, HEART_PERIODIC_EVT, DEFAULT_HEARTRATE_PERIOD );
    } 
  }
  else if (event == HEARTRATE_MEAS_NOTI_DISABLED)
  {
    //hr_module_stop();
    hr_module_lowpower();
    // stop periodic measurement
    osal_stop_timerEx( heartRate_TaskID, HEART_PERIODIC_EVT );
  }
  else if (event == HEARTRATE_COMMAND_SET)
  {
    // reset energy expended
    heartRateEnergy = 0;
  }
}

/*********************************************************************
 * @fn      eegCB
 *
 * @brief   Callback function for ecg service.
 *
 * @param   event - service event
 *
 * @return  none
 */
static void ecgCB(uint8 event)
{
  if (event == ECG_MEAS_NOTI_ENABLED)
  {
    // if connected start periodic measurement
    if (gapProfileState == GAPROLE_CONNECTED)
    {
      TI_ADS1293_SPISetup();                                                       // Initilaize CC254x SPI Block 
      ads1299_wakeup();
      us_delay(50);
      //ads1299_set_up();      //can't set up at here.
      osal_start_timerEx( heartRate_TaskID, ECG_PERIODIC_EVT, DEFAULT_ECG_PERIOD );
    } 
  }
  else if (event == ECG_MEAS_NOTI_DISABLED)
  {
    // stop periodic measurement
    osal_stop_timerEx( heartRate_TaskID, ECG_PERIODIC_EVT );
    ads1299_stop();    
    //ads1299_standby();
  }
  else if (event == ECG_COMMAND_SET)
  {
    // reset energy expended                   ???
    heartRateEnergy = 0;
  }
}

/*********************************************************************
 * @fn      heartRateBattCB
 *
 * @brief   Callback function for battery service.
 *
 * @param   event - service event
 *
 * @return  none
 */
static void heartRateBattCB(uint8 event)
{
  if (event == BATT_LEVEL_NOTI_ENABLED)
  {
    // if connected start periodic measurement
    if (gapProfileState == GAPROLE_CONNECTED)
    {
      osal_start_timerEx( heartRate_TaskID, BATT_PERIODIC_EVT, DEFAULT_BATT_PERIOD );
    } 
  }
  else if (event == BATT_LEVEL_NOTI_DISABLED)
  {
    // stop periodic measurement
    osal_stop_timerEx( heartRate_TaskID, BATT_PERIODIC_EVT );
  }
}

/*********************************************************************
 * @fn      heartRatePeriodicTask
 *
 * @brief   Perform a periodic heart rate application task.
 *
 * @param   none
 *
 * @return  none
 */
static void heartRatePeriodicTask( void )
{
  if (gapProfileState == GAPROLE_CONNECTED)
  {
    HalI2CInit( i2cClock_267KHZ );
    // send heart rate measurement notification
    heartRateMeasNotify();
    
    // Restart timer
    osal_start_timerEx( heartRate_TaskID, HEART_PERIODIC_EVT, DEFAULT_HEARTRATE_PERIOD );
  }
}

/*********************************************************************
 * @fn      eegPeriodicTask
 *
 * @brief   Perform a periodic ecg application task.
 *
 * @param   none
 *
 * @return  none
 */
static void ecgPeriodicTask( void )
{
  if (gapProfileState == GAPROLE_CONNECTED)
  {
    // send heart rate measurement notification
    ecgMeasNotify();
    
    // Restart timer
    osal_start_timerEx( heartRate_TaskID, ECG_PERIODIC_EVT, DEFAULT_ECG_PERIOD );
  }
}

/*********************************************************************
 * @fn      heartRateBattPeriodicTask
 *
 * @brief   Perform a periodic task for battery measurement.
 *
 * @param   none
 *
 * @return  none
 */
static void heartRateBattPeriodicTask( void )
{
  if (gapProfileState == GAPROLE_CONNECTED)
  {
    // perform battery level check
    Batt_MeasLevel( );
    
    // Restart timer
    osal_start_timerEx( heartRate_TaskID, BATT_PERIODIC_EVT, DEFAULT_BATT_PERIOD );
  }
}


void Haptic_Control(void)
{
  HeartRate_GetParameter(HEARTRATE_COMMAND, &haptic_config);
  if(haptic_config == 0x01)
    haptic_start = 0;  
  else if(haptic_config == 0x02)
    haptic_start = 1;
  
  if(haptic_start == 1)
  {
    if(P1_0 == 1)
      P1_0 = 0;
    else 
      P1_0 = 1;
  }
  else
    P1_0 = 0;
  //haptic_config = 2;
  //HeartRate_SetParameter(HEARTRATE_COMMAND, sizeof (uint8), &haptic_config);
  HalI2CInit( i2cClock_267KHZ );
  haptic_config_update();
  haptic_process();
}

static void hapticPeriodicTask()
{
  if (gapProfileState == GAPROLE_CONNECTED)
    {
      // perform battery level check
      Haptic_Control( );
      
      // Restart timer
      osal_start_timerEx( heartRate_TaskID, HAPTIC_PERIODIC_EVT, DEFAULT_HAPTIC_PERIOD );
    }  
}


void hr_module_init()
{
  //heart rate init.
  HalI2CInit( i2cClock_267KHZ );
  
  uint8 buffer_w[2] = {0, 0};  //data to write in I2C
  uint8 checkpoint1 = 0;       //data that read back
  
  buffer_w[0] = INT_STATUS;      //clear power up interrupt
  HalI2CWrite(HR_ID, 1, buffer_w, 0);
  HalI2CRead(HR_ID, 1, &checkpoint1);

  
  buffer_w[0] = LED_PWC;  //0X09
  buffer_w[1] = 0x88;         //set led current
  HalI2CWrite(HR_ID, 2, buffer_w, 1);

  buffer_w[0] = SPO2_CONFIG; //0X07
  buffer_w[1] = 0x03;         //set 50SPS, 16bits. 20ms/sample. 100ms/20ms=5samples 2B*5=10B
  HalI2CWrite(HR_ID, 2, buffer_w, 1);
  
  buffer_w[0] = MODE_CFR; //0X06
  buffer_w[1] = 0x02;         //set HR only mode 
  HalI2CWrite(HR_ID, 2, buffer_w, 1); //after set mode, it triggers power ready interrupt.

  buffer_w[0] = MODE_CFR;      //for verifying if HR module run or not.
  HalI2CWrite(HR_ID, 1, buffer_w, 0);
  HalI2CRead(HR_ID, 1, &checkpoint1);
  
  buffer_w[0] = INT_EN;  //0X01
  buffer_w[1] = 0x00;         // enable HR interrupt 
  HalI2CWrite(HR_ID, 2, buffer_w, 1);    
}

void hr_module_stop()
{
  HalI2CInit( i2cClock_267KHZ );
  
  uint8 buffer_w[2] = {0, 0};  //data to write in I2C
  
  buffer_w[0] = MODE_CFR; //0X06
  buffer_w[1] = 0x80;         // 0x00 as stop converting.
  HalI2CWrite(HR_ID, 2, buffer_w, 1); //after set mode, it triggers power ready interrupt.
  
}

void hr_module_lowpower(void)
{
  HalI2CInit( i2cClock_267KHZ );
  
  uint8 buffer_w[2] = {0, 0};  //data to write in I2C
  
  buffer_w[0] = MODE_CFR; //0X06
  buffer_w[1] = 0x80;         // power down.
  HalI2CWrite(HR_ID, 2, buffer_w, 1); //after set mode, it triggers power ready interrupt.
  
}

int num_counter = 0;

/******************************************************************************/
// TI_ADS1293_SPI_DRDYB_PIN interrupt service routine
#pragma vector = P1INT_VECTOR
__near_func __interrupt void TI_ADS1293_DRDY_PORTx(void)
{
//  static uint8 rd_count = 0;
//  static uint8 first_time = 1;
  uint8 *tmp_buf;
  uint8 blank_buf;
  EA = 0;
  IRCON2 &= ~0x08;    // clear Port 1 interrupt flag


  if (P1IFG & 0x80)
  {
    P1IFG &= ~0x80;               // clear interrupt status flag

    // Read ECG Data
  
    //spiWriteByte(_RDATA);
     
    tmp_buf = &recv_buf[0];
    
    //TI_ADS1293_SPIStreamRead((tmp_buf), BUF_SIZE);
    
    // test RDATA mode 

    CS_ADS = 0; 
    spiWriteByte(_RDATA);     //???
    us_delay(10);

    /* 1 channel  
  
    for(int i = 0; i<8; i++)
    {
      spiReadByte((&blank_buf), 0xFF);
      spiReadByte((&blank_buf), 0xFF);
      spiReadByte((&blank_buf), 0xFF);
    }    
    for(int i = 0; i < 3; i++)
    {
      spiReadByte((recv_buf+i+counter_ADS), 0xFF);                                             // Read data     
    }   
    counter_ADS = counter_ADS + 3; 
    CS_ADS = 1;
    */
    //*(recv_buf+counter_ADS) = 10;
    //*(recv_buf+counter_ADS + 1) = 10;
   
    
    /* 4 channels  
    //read status data to packet 1.
    //leave status to read.
    //set packet header
    if((counter_ADS == 0) && (packet2enable == 0))
    {
      *(recv_buf + counter_ADS) = 1;
      counter_ADS += 1;
      for(int i = 0; i < 2; i++)
      {
        spiReadByte((recv_buf+i+counter_ADS), 0xFF);                                             // Read data     
      }  
      counter_ADS = counter_ADS + 2;
      spiReadByte((&blank_buf), 0xFF); //need to change! deal with status.???
      //throw data from channel 1~4
      for(int i = 0; i<4; i++)
      {
        spiReadByte((&blank_buf), 0xFF);
        spiReadByte((&blank_buf), 0xFF);
        spiReadByte((&blank_buf), 0xFF);
      }   
      //read data from channel 5~8
      for(int j = 0; j < 4; j++)
      {
        for(int i = 0; i < 3; i++)
        {
          spiReadByte((recv_buf+i+counter_ADS), 0xFF);                                             // Read data     
        }  
        counter_ADS = counter_ADS + 3;    
      }
    }
    
    //packet 1's last 5 bytes- 2nd sample.
    else if((counter_ADS == 15) && (packet2enable == 0))
    {
      //miss first 4 channels' data
      for(int i = 0; i<5; i++)
      {
        spiReadByte((&blank_buf), 0xFF);
        spiReadByte((&blank_buf), 0xFF);
        spiReadByte((&blank_buf), 0xFF);
      } 
      
      for(int i = 0; i < 5; i++)
      {
        spiReadByte((recv_buf+i+counter_ADS), 0xFF);                                             // Read data     
      }  
      send_buf = dataBufX;   //changed.??
      dataReadyFlag = 1;
      packet2enable = 1;
      counter_ADS = 0;
      
      //keep sending to another buffer.
   
      recv_buf = dataBufY;
      
      
      //header for packet 2
      *(recv_buf + counter_ADS) = 2;
      counter_ADS = counter_ADS + 1;
      
      //read data to the other buffer for packet 2
      for(int i = 0; i < 7; i++)
      {
        spiReadByte((recv_buf+i+counter_ADS), 0xFF);                                             // Read data     
      }   
      counter_ADS = counter_ADS + 7;
    }
    
    if((packet2enable == 1) && (counter_ADS == 8))
    {
      //miss status 3 bytes and first 4 channel * 3B data
      for(int i = 0; i<5; i++)
      {
        spiReadByte((&blank_buf), 0xFF);
        spiReadByte((&blank_buf), 0xFF);
        spiReadByte((&blank_buf), 0xFF);
      } 
 
      //read all 12 bytes for last channel of 3rd sample
      for(int j = 0; j < 4; j++)
      {
        for(int i = 0; i < 3; i++)
        {
          spiReadByte((recv_buf+i+counter_ADS), 0xFF);                                             // Read data     
        }  
        counter_ADS = counter_ADS + 3;    
      }
      counter_ADS = 0;
      send_buf = dataBufY;  //??
      dataReadyFlag = 1;
      packet2enable = 0;
 
      recv_buf = dataBufX; //??
      
    }
    */


    /* 3 channels */
    if(counter_ADS == 0)
    {
      
      /*status.*/
      spiReadByte( &status0, 0xFF); 
      spiReadByte( &status1, 0xFF); 
      spiReadByte( &status2, 0xFF); 
      


      status_value1 = status1 >> 4;
      status0 = status0 << 4;
      *recv_buf = status0 + status_value1;
      status_value1 = status1 << 4;
      status_value2 = status2 >> 4;
      *(recv_buf + 1) = status_value1 + status_value2;
      
      counter_ADS = counter_ADS + 2;  
      //FOR TEST
      //counter_ADS = 0;
      //eeg data for 3 channels
      for(int i = 0; i < 3; i++)
      {
        spiReadByte((recv_buf+i+counter_ADS), 0xFF);                                             // Read data     
      }    
      counter_ADS = counter_ADS + 3;  
      for(int i = 0; i < 3; i++)
      {
        spiReadByte((recv_buf+i+counter_ADS), 0xFF);                                             // Read data     
      }    
      counter_ADS = counter_ADS + 3;  
      for(int i = 0; i < 3; i++)
      {
        spiReadByte((recv_buf+i+counter_ADS), 0xFF);                                             // Read data     
      }    
      counter_ADS = counter_ADS + 3; 
      
      for(int i = 0; i<5; i++)
      {
        spiReadByte((&blank_buf), 0xFF);
        spiReadByte((&blank_buf), 0xFF);
        spiReadByte((&blank_buf), 0xFF);
      }


    }
    else
    {
      
      spiReadByte((&blank_buf), 0xFF);
      spiReadByte((&blank_buf), 0xFF);
      spiReadByte((&blank_buf), 0xFF);
      
      for(int j = 0; j < 3; j++)
      {
        for(int i = 0; i < 3; i++)
        {
          spiReadByte((recv_buf+i+counter_ADS), 0xFF);                                             // Read data     
        }    
        counter_ADS = counter_ADS + 3; 
      }
      for(int i = 0; i<5; i++)
      {
        spiReadByte((&blank_buf), 0xFF);
        spiReadByte((&blank_buf), 0xFF);
        spiReadByte((&blank_buf), 0xFF);
      }       
    
      counter_ADS = 0;
      send_buf = recv_buf;
      dataReadyFlag = 1;
      if (recv_buf == dataBufX)  //keep sending to another buffer.
        recv_buf = dataBufY;
      else
        recv_buf = dataBufX;
      
    }
   
    CS_ADS = 1;
    
    
    /* 1 channel   
    //no flag contrain, no ADS waiting, keep data continuous.
    if(counter_ADS > 35)  //36bytes will takes 12*2ms = 24ms or 12*4ms= 48ms
    {      
      send_buf = recv_buf;
      if (recv_buf == dataBufX)  //keep sending to another buffer.
        recv_buf = dataBufY;
      else
        recv_buf = dataBufX;
      dataReadyFlag = 1; 
      counter_ADS = 0;
      
    }
  
     */

    
  }

  EA = 1;
}

void haptic_config_update(void)
{
  HeartRate_GetParameter(HEARTRATE_COMMAND, &haptic_config);
  //ecg_GetParameter(ECG_COMMAND, &haptic_config);
  if(haptic_config == 0x01)
    haptic_start = 1;              
  else if(haptic_config == 0x02)
  {
    haptic_start = 0;
    haptic_stop();
  }
  else if(haptic_config == 0x03)
    haptic_duration++;             //*m
  else if(haptic_config == 0x04)
    haptic_duration--;
  else if(haptic_config == 0x05)
    vibrate_period++;              //*100ms
  else if(haptic_config == 0x06)
    vibrate_period--;
  else if(haptic_config == 0x07)
    vibration_amplitudeH = vibration_amplitudeH + 0x05;
  else if(haptic_config == 0x08)
    vibration_amplitudeH = vibration_amplitudeH - 0x05;
  
  if(vibration_amplitudeH > 0x31)
    vibration_amplitudeH = 0x30;
  if(vibration_amplitudeH < 0x10)
    vibration_amplitudeH = 0x10;
  
  haptic_config = 0;
  ecg_SetParameter(ECG_COMMAND, sizeof (uint8), &haptic_config);
}

/*********************************************************************
*********************************************************************/