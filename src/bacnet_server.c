#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <pthread.h>
#include <string.h>
#include <modbus.h>
#include <errno.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <libbacnet/address.h>
#include <libbacnet/device.h>
#include <libbacnet/handlers.h>
#include <libbacnet/datalink.h>
#include <libbacnet/bvlc.h>
#include <libbacnet/client.h>
#include <libbacnet/txbuf.h>
#include <libbacnet/tsm.h>
#include <libbacnet/ai.h>
#include "bacnet_namespace.h"
#define BACNET_INSTANCE_NO	90
#define BACNET_PORT	0xBAC1
#define BACNET_INTERFACE	"lo"
#define BACNET_DATALINK_TYPE	"bvlc"
#define BACNET_SELECT_TIMEOUT_MS 1	/* ms */
#define RUN_AS_BBMD_CLIENT	1
#if RUN_AS_BBMD_CLIENT
#define BACNET_BBMD_PORT	0xBAC0
#define BACNET_BBMD_ADDRESS	"140.159.160.7"
#define BACNET_BBMD_TTL	90
#endif
#define lists 2
#define MTCP			    "140.159.153.159", 502
/*excessive libraries and #defines for now*/
/*---------------------------------------------------*/
/*-----------------linked lists----------------------*/
/*---------------------------------------------------*/
typedef struct sobj wobj;
struct sobj {
		char *word;
		wobj *next;
};


static wobj *list_get_first(wobj **lhead) { /* gets header sets it up for struct above ^^^ */
		wobj *fobj;
		fobj = *lhead; /* gets list header */
		*lhead = (*lhead)->next; /* moves list header to next */ 
		return fobj; /* returns header */
}

static wobj *lhead[lists];
static pthread_mutex_t llock = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t tlock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t lrdy = PTHREAD_COND_INITIALIZER;


static void add_to_list(wobj **lhead, char *word) {
		wobj *lobj, *tobj;
		char *tstring=strdup(word);
		tobj = malloc(sizeof(wobj)); /* new temp object with memory allocated to the size of w obj */
		tobj->word = tstring; 
		tobj->next = NULL;
		pthread_mutex_lock(&llock);
		if (*lhead == NULL) { /* start if */
			*lhead =tobj; /* makes list header = temp object */
		} else {
			lobj = *lhead; /* moves pointer to struct above ^^^ */
			while (lobj->next) { /*waits for change*/
				lobj = lobj->next; 
			}
			lobj->next = tobj; /*moves pointer to temp object for now */
			lobj=lobj->next;
		}
		pthread_mutex_unlock(&llock);
		pthread_cond_signal(&lrdy);
}


static int Update_Analog_Input_Read_Property(
		BACNET_READ_PROPERTY_DATA *rpdata) {

    static int index;
    int instance_no = bacnet_Analog_Input_Instance_To_Index(
			rpdata->object_instance);

    if (rpdata->object_property != bacnet_PROP_PRESENT_VALUE) goto not_pv;

    printf("AI_Present_Value request for instance %i\n", instance_no);
    /* Update the values to be sent to the BACnet client here.
     * The data should be read from the head of a linked list. You are required
     * to implement this list functionality.
     *
     * bacnet_Analog_Input_Present_Value_Set() 
     *     First argument: Instance No
     *     Second argument: data to be sent
     *
     * Without reconfiguring libbacnet, a maximum of 4 values may be sent */
    bacnet_Analog_Input_Present_Value_Set(0, test_data/*random data at start ( 0xA4EC, 0x6E39, 0x8740, 0x1065, 0x9134, 0xFC8C )obviously change this to data at IP address*/[index++]);
    /* bacnet_Analog_Input_Present_Value_Set(1, test_data[index++]); */
    /* bacnet_Analog_Input_Present_Value_Set(2, test_data[index++]); */
    
    if (index == NUM_TEST_DATA) index = 0;

not_pv:
    return bacnet_Analog_Input_Read_Property(rpdata);
}

static bacnet_object_functions_t server_objects[] = {
    {bacnet_OBJECT_DEVICE,
	    NULL,
	    bacnet_Device_Count,
	    bacnet_Device_Index_To_Instance,
	    bacnet_Device_Valid_Object_Instance_Number,
	    bacnet_Device_Object_Name,
	    bacnet_Device_Read_Property_Local,
	    bacnet_Device_Write_Property_Local,
	    bacnet_Device_Property_Lists,
	    bacnet_DeviceGetRRInfo,
	    NULL, /* Iterator */
	    NULL, /* Value_Lists */
	    NULL, /* COV */
	    NULL, /* COV Clear */
	    NULL  /* Intrinsic Reporting */
    },
    {bacnet_OBJECT_ANALOG_INPUT,
            bacnet_Analog_Input_Init,
            bacnet_Analog_Input_Count,
            bacnet_Analog_Input_Index_To_Instance,
            bacnet_Analog_Input_Valid_Instance,
            bacnet_Analog_Input_Object_Name,
            Update_Analog_Input_Read_Property,
            bacnet_Analog_Input_Write_Property,
            bacnet_Analog_Input_Property_Lists,
            NULL /* ReadRangeInfo */ ,
            NULL /* Iterator */ ,
            bacnet_Analog_Input_Encode_Value_List,
            bacnet_Analog_Input_Change_Of_Value,
            bacnet_Analog_Input_Change_Of_Value_Clear,
            bacnet_Analog_Input_Intrinsic_Reporting},
    {MAX_BACNET_OBJECT_TYPE}
};

static void register_with_bbmd(void) {
#if RUN_AS_BBMD_CLIENT
    /* Thread safety: Shares data with datalink_send_pdu */
    bacnet_bvlc_register_with_bbmd(
	    bacnet_bip_getaddrbyname(BACNET_BBMD_ADDRESS), 
	    htons(BACNET_BBMD_PORT),
	    BACNET_BBMD_TTL);
#endif
}

static void *minute_tick(void *arg) {
    while (1) {
	pthread_mutex_lock(&timer_lock);

	/* Expire addresses once the TTL has expired */
	bacnet_address_cache_timer(60);

	/* Re-register with BBMD once BBMD TTL has expired */
	register_with_bbmd();

	/* Update addresses for notification class recipient list 
	 * Requred for INTRINSIC_REPORTING
	 * bacnet_Notification_Class_find_recipient(); */
	
	/* Sleep for 1 minute */
	pthread_mutex_unlock(&timer_lock);
	sleep(60);
    }
    return arg;
}

static void *second_tick(void *arg) {
    while (1) {
	pthread_mutex_lock(&timer_lock);

	/* Invalidates stale BBMD foreign device table entries */
	bacnet_bvlc_maintenance_timer(1);

	/* Transaction state machine: Responsible for retransmissions and ack
	 * checking for confirmed services */
	bacnet_tsm_timer_milliseconds(1000);

	/* Re-enables communications after DCC_Time_Duration_Seconds
	 * Required for SERVICE_CONFIRMED_DEVICE_COMMUNICATION_CONTROL
	 * bacnet_dcc_timer_seconds(1); */

	/* State machine for load control object
	 * Required for OBJECT_LOAD_CONTROL
	 * bacnet_Load_Control_State_Machine_Handler(); */

	/* Expires any COV subscribers that have finite lifetimes
	 * Required for SERVICE_CONFIRMED_SUBSCRIBE_COV
	 * bacnet_handler_cov_timer_seconds(1); */

	/* Monitor Trend Log uLogIntervals and fetch properties
	 * Required for OBJECT_TRENDLOG
	 * bacnet_trend_log_timer(1); */
	
	/* Run [Object_Type]_Intrinsic_Reporting() for all objects in device
	 * Required for INTRINSIC_REPORTING
	 * bacnet_Device_local_reporting(); */
	
	/* Sleep for 1 second */
	pthread_mutex_unlock(&timer_lock);
	sleep(1);
    }
    return arg;
}

static void *modbus_contact(void *arg){
	int i; /*variable*/
	int mr; /*variable to track number of items received from modbus*/
	uint16_t tabr[128]; /* storage for data from modbus request*/
	char sending[64]; /*storage value for sending data*/
	modbus_t *ctx; /*reference to modbus connection, ctx is variable for addr as seen below */
	confailed:;
	ctx = modbus_new_tcp("140.159.153.159", 502); /*connects to server at uni */


	if (ctx == NULL) {
		fprintf(stderr, "Cannot allocate libmodbus context read manual\n");
		usleep(100000);/* waits .1 second to not DDOS */
		goto confailed; /* tries sequence again */
	}
	if (modbus_connect(ctx) == -1) {
		printf("Connection failure read manual");
		fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
		modbus_free(ctx);
		usleep(100000); /* waits .1 second to not DDOS */
		goto confailed; /* tries sequence again */
	}
while (1) {
	mr = modbus_read_registers(ctx, 90,4, tabr); /*reads the data from modbus server */

	if (mr == -1) {
		fprintf(stderr, "%s\n", modbus_strerror(errno));
		modbus_close(ctx);
		modbus_free(ctx);
		usleep(100000); /* waits .1 second to not DDOS */
		goto confailed; /* tries sequence again */
	}

	for (i=0; i < mr; i++) {
		sprintf(sending, "%x", tabr[i]);
		add_to_list(&lhead[i], sending);
	}
	usleep(100000); /* waits .1 second to not DDOS before next read */
	}
	return arg; /*removes warning*/
}
static void ms_tick(void) {
    /* Updates change of value COV subscribers.
     * Required for SERVICE_CONFIRMED_SUBSCRIBE_COV
     * bacnet_handler_cov_task(); */
}

#define BN_UNC(service, handler) \
    bacnet_apdu_set_unconfirmed_handler(		\
		    SERVICE_UNCONFIRMED_##service,	\
		    bacnet_handler_##handler)
#define BN_CON(service, handler) \
    bacnet_apdu_set_confirmed_handler(			\
		    SERVICE_CONFIRMED_##service,	\
		    bacnet_handler_##handler)

int main(int argc, char **argv) {
    uint8_t rx_buf[bacnet_MAX_MPDU];
    uint16_t pdu_len;
    BACNET_ADDRESS src;
    pthread_t minute_tick_id, second_tick_id, modbus_contact;

    bacnet_Device_Set_Object_Instance_Number(BACNET_INSTANCE_NO);
    bacnet_address_init();

    /* Setup device objects */
    bacnet_Device_Init(server_objects);
    BN_UNC(WHO_IS, who_is);
    BN_CON(READ_PROPERTY, read_property);

    bacnet_BIP_Debug = true;
    bacnet_bip_set_port(htons(BACNET_PORT));
    bacnet_datalink_set(BACNET_DATALINK_TYPE);
    bacnet_datalink_init(BACNET_INTERFACE);
    atexit(bacnet_datalink_cleanup);
    memset(&src, 0, sizeof(src));

    register_with_bbmd();

    bacnet_Send_I_Am(bacnet_Handler_Transmit_Buffer);

    pthread_create(&minute_tick_id, 0, minute_tick, NULL);
    pthread_create(&second_tick_id, 0, second_tick, NULL);
    
    pthread_create(&modbus_contact, 0, modbus_contact, NULL);
    /* Start another thread here to retrieve your allocated registers from the
     * modbus server. This thread should have the following structure (in a
     * separate function):
     *
     * Initialise:
     *	    Connect to the modbus server
     *
     * Loop:
     *	    Read the required number of registers from the modbus server
     *	    Store the register data into the tail of a linked list 
     */

    while (1) {
	pdu_len = bacnet_datalink_receive(
		    &src, rx_buf, bacnet_MAX_MPDU, BACNET_SELECT_TIMEOUT_MS);

	if (pdu_len) {
	    /* May call any registered handler.
	     * Thread safety: May block, however we still need to guarantee
	     * atomicity with the timers, so hold the lock anyway */
	    pthread_mutex_lock(&timer_lock);
	    bacnet_npdu_handler(&src, rx_buf, pdu_len);
	    pthread_mutex_unlock(&timer_lock);
	}

	ms_tick();
    }

    return 0;
}
