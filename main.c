/******************************************************************************
 *  Copyright (C) Cambridge Silicon Radio Limited, 2014
 *
 *  FILE
 *      beacon.c
 *
 *  DESCRIPTION
 *      This file defines an advertising node implementation
 *
 *****************************************************************************/

/*============================================================================*
 *  SDK Header Files
 *============================================================================*/

#include <main.h>
#include <gap_app_if.h>
#include <config_store.h>
#include <pio.h>
#include <random.h>
#include <timer.h>          /* Chip timer functions */
#include <panic.h>          /* Support for applications to panic */
#include <battery.h>
/*============================================================================*
 *  Local Header File
 *============================================================================*/

#include "main_config.h"

/*============================================================================*
 *  Private Definitions
 *============================================================================*/

/* Number of timers used in this application */
#define MAX_TIMERS 1

/* First timeout at which the timer has to fire a callback */
#define TIMER_TIMEOUT1 (200 * MILLISECOND)

/*============================================================================*
 *  Private Data
 *============================================================================*/

/* Declare timer buffer to be managed by firmware library */
static uint16 app_timers[SIZEOF_APP_TIMER * MAX_TIMERS];

static uint8 advData[MAX_ADVERT_PACKET_SIZE];

uint8 advPayloadSize;
ls_addr_type addressType = ls_addr_type_public;     /* use public address */

/*============================================================================*
 *  Private Function Prototypes
 *============================================================================*/

static void startAdvertising(void);

static void appSetRandomAddress(void);

/* Start timer */
static void startTimer(uint32 timeout, timer_callback_arg handler);

/* Callback after first timeout */
static void timerCallback1(timer_id const id);

static uint8 AppUpdatePressureLevel(void);

/*============================================================================*
 *  Private Function Implementations
 *============================================================================*/

/*----------------------------------------------------------------------------*
 *  NAME
 *      startTimer
 *
 *  DESCRIPTION
 *      Start a timer
 *
 * PARAMETERS
 *      timeout [in]    Timeout period in seconds
 *      handler [in]    Callback handler for when timer expires
 *
 * RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void startTimer(uint32 timeout, timer_callback_arg handler)
{
	/* Now starting a timer */
	const timer_id tId = TimerCreate(timeout, TRUE, handler);

	/* If a timer could not be created, panic to restart the app */
	if (tId == TIMER_INVALID)
	{
		/* Panic with panic code 0xfe */
		Panic(0xfe);
	}
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      timerCallback1
 *
 *  DESCRIPTION
 *      This function is called when the timer created by TimerCreate expires.
 *      It creates a new timer that will expire after the second timer interval.
 *
 * PARAMETERS
 *      id [in]     ID of timer that has expired
 *
 * RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void timerCallback1(timer_id const id)
{
	uint16 luw_BatteryLevel;

	/* Stop broadcasting */
	LsStartStopAdvertise(FALSE, whitelist_disabled, addressType);

	/* clear the existing advertisement data, if any */
	LsStoreAdvScanData(0, NULL, ad_src_advertise);

	advData[3] = 0x19u;

	/* MAC */
	advData[4] = 0x19u;
	advData[5] = 0x01u;
	advData[6] = 0x20u;
	advData[7] = 0x19u;

	/* Message Type */
	advData[8] = 0x00;

	/* Data */
	advData[9]  = AppUpdatePressureLevel();
	advData[10] = 0x00;

	luw_BatteryLevel = BatteryReadVoltage();

	advData[11] = (uint8)(luw_BatteryLevel >> 8u);
	advData[12] = (uint8)(luw_BatteryLevel);

	/* store the advertisement data */
	LsStoreAdvScanData(advPayloadSize + 3, advData, ad_src_advertise);

	/* Start broadcasting */
	LsStartStopAdvertise(TRUE, whitelist_disabled, addressType);

	/* Now start a new timer for second callback */
	startTimer((TIMER_TIMEOUT1), timerCallback1);
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      appSetRandomAddress
 *
 *  DESCRIPTION
 *      This function generates a non-resolvable private address and sets it
 *      to the firmware.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
static void appSetRandomAddress(void)
{
	BD_ADDR_T addr;

	/* "completely" random MAC addresses by default: */
	for(;;)
	{
		uint32 now = TimeGet32();
		/* Random32() is just two of them, no use */
		uint32 rnd = Random16();
		addr.uap = 0xff & (rnd ^ now);
		/* No sub-part may be zero or all-1s */
		if ( 0 == addr.uap || 0xff == addr.uap ) continue;
		addr.lap = 0xffffff & ((now >> 8) ^ (73 * rnd));
		if ( 0 == addr.lap || 0xffffff == addr.lap ) continue;
		addr.nap = 0x3fff & rnd;
		if ( 0 == addr.nap || 0x3fff == addr.nap ) continue;
		break;
	}

	/* Set it to actually be an acceptable random address */
	addr.nap &= ~BD_ADDR_NAP_RANDOM_TYPE_MASK;
	addr.nap |=  BD_ADDR_NAP_RANDOM_TYPE_NONRESOLV;
	GapSetRandomAddress(&addr);
}


/*----------------------------------------------------------------------------*
 *  NAME
 *      startAdvertising
 *
 *  DESCRIPTION
 *      This function is called to start advertisements.
 *
 *      Advertisement packet will contain Flags AD and Manufacturer-specific
 *      AD with Manufacturer id set to CSR and payload set to the value of
 *      the User Key 0. The payload size is set by the User Key 1.
 *
 *      +--------+-------------------------------------------------+
 *      |FLAGS AD|MANUFACTURER AD                                  |
 *      +--------+-------------------------------------------------+
 *       0      2 3
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
void startAdvertising(void)
{
	uint16 offset = 0;
	uint8 filler;
	uint16 advInterval;
	uint16 luw_BatteryLevel;

	/* initialise values from User CsKeys */

	/* read User key 0 for the payload filler */
	filler = (uint8)(CSReadUserKey(0) & 0x00FF);

	/* read User key 1 for the payload size */
	advPayloadSize = (uint8)(CSReadUserKey(1) & 0x00FF);

	/* range check */
	if((advPayloadSize < 1) || (advPayloadSize > MAX_ADVERT_PAYLOAD_SIZE))
	{
		/* revert to default payload size */
		advPayloadSize = DEFAULT_ADVERT_PAYLOAD_SIZE;
	}

	/* read User key 2 for the advertising interval */
	advInterval = CSReadUserKey(2);

	/* range check */
	if((advInterval < MIN_ADVERTISING_INTERVAL) ||
			(advInterval > MAX_ADVERTISING_INTERVAL))
	{
		/* revert to default advertising interval */
		advInterval = DEFAULT_ADVERTISING_INTERVAL;
	}

	/* read address type from User key 3 */
	if(CSReadUserKey(3))
	{
		/* use random address type */
		addressType = ls_addr_type_random;

		/* generate and set the random address */
		appSetRandomAddress();
	}

	/* set the GAP Broadcaster role */
	GapSetMode(gap_role_broadcaster,
			gap_mode_discover_no,
			gap_mode_connect_no,
			gap_mode_bond_no,
			gap_mode_security_none);

	/* clear the existing advertisement data, if any */
	LsStoreAdvScanData(0, NULL, ad_src_advertise);

	/* set the advertisement interval, API accepts the value in microseconds */
	GapSetAdvInterval(advInterval * MILLISECOND, advInterval * MILLISECOND);

	/* manufacturer-specific data */
	advData[0] = AD_TYPE_MANUF;

	/* CSR company code, little endian */
	advData[1] = 0x0A;
	advData[2] = 0x00;

	/* Counter */
	advData[3] = 0x00u;

	luw_BatteryLevel = BatteryReadVoltage();

	advData[11] = (uint8)(luw_BatteryLevel >> 8u);
	advData[12] = (uint8)(luw_BatteryLevel);



	/* fill in the rest of the advertisement */
	for(offset = 0; offset < (advPayloadSize - 3); offset++)
	{
		advData[6 + offset] = filler;
	}

	/* store the advertisement data */
	LsStoreAdvScanData(advPayloadSize + 3, advData, ad_src_advertise);

	/* Start broadcasting */
	LsStartStopAdvertise(TRUE, whitelist_disabled, addressType);
}


/*============================================================================*
 *  Public Function Implementations
 *============================================================================*/

/*----------------------------------------------------------------------------*
 *  NAME
 *      AppPowerOnReset
 *
 *  DESCRIPTION
 *      This function is called just after a power-on reset (including after
 *      a firmware panic).
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/

void AppPowerOnReset(void)
{
	/* empty */
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      AppInit
 *
 *  DESCRIPTION
 *      This function is called after a power-on reset (including after a
 *      firmware panic) or after an HCI Reset has been requested.
 *
 *      NOTE: In the case of a power-on reset, this function is called
 *      after AppPowerOnReset().
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/

void AppInit(sleep_state last_sleep_state)
{

	/* Timer Init */
	TimerInit(MAX_TIMERS, (void *)app_timers);

	/* set all PIOs to inputs and pull them down */
	PioSetModes(0xFFFFFFFFUL, pio_mode_user);
	PioSetDirs(0xFFFFFFFFUL, FALSE);
	PioSetPullModes(0xFFFFFFFFUL, pio_mode_strong_pull_down);

	/* disable wake up on UART RX */
	SleepWakeOnUartRX(FALSE);

	/* pull down the I2C lines */
	PioSetI2CPullMode(pio_i2c_pull_mode_strong_pull_down);

	/* Start advertising */
	startAdvertising();

	/* Start the first timer */
	startTimer(TIMER_TIMEOUT1, timerCallback1);
}


/*----------------------------------------------------------------------------*
 *  NAME
 *      AppProcessSystemEvent
 *
 *  DESCRIPTION
 *      This user application function is called whenever a system event, such
 *      as a battery low notification, is received by the system.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/

void AppProcessSystemEvent(sys_event_id id, void *data)
{
	/* empty */
}


/*----------------------------------------------------------------------------*
 *  NAME
 *      AppProcessLmEvent
 *
 *  DESCRIPTION
 *      This user application function is called whenever a LM-specific event is
 *      received by the system.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/

bool AppProcessLmEvent(lm_event_code event_code,
		LM_EVENT_T *p_event_data)
{
	return TRUE;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      AppUpdatePressureLevel
 *
 *  DESCRIPTION
 *      This user application function is called to update the pressure level according with the
 *      last valid data from sensor
 *
 *  RETURNS
 *      uint8 - % Pressure Level.
 *
 *---------------------------------------------------------------------------*/
static uint8 AppUpdatePressureLevel(void)
{
	static uint8 lub_Level;

	/* Check if Level is lower than max value */
	if(lub_Level < 100u)
	{
		/* Increase level value */
		lub_Level++;
	}
	else
	{
		/* Clear Level */
		lub_Level = 0;
	}
    
    return lub_Level;
}
