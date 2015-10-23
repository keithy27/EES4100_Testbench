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
#define NUM_LISTS 2
//excessive libraries and #defines for now*/

static void *modb(void *arg){
int i; //variable
int mr; //variable to track number of items received from modbus
uint16_t tab_reg[128]; // storage for data from modbus request
char sending[64]; //storage value for sending data
modbus_t *ctx; //reference to modbus connection, ctx is variable for addr as seen below 
confailed:;
ctx = modbus_new_tcp("140.159.153.159", 502);
//connects

if (ctx == NULL) {
fprintf(stderr, "Unable to allocate libmodbus context\n");
sleep(1);
goto confailed;
}
if (modbus_connect(ctx) == -1) {
printf("con failed");
fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
modbus_free(ctx);
sleep(1);
goto confailed;
}
while (1) {
mr = modbus_read_registers(ctx, 90,4, tab_reg); //reads the data from modbus server
/* if it failed close connection and then reconnect registers 90-93 */
if (mr == -1) {
fprintf(stderr, "%s\n", modbus_strerror(errno));
modbus_close(ctx);
modbus_free(ctx);
usleep(100000); //wait for 100ms before trying again
goto confailed;
}
/*add all items received in order to the head of the linked list*/
for (i=0; i < mr; i++) {
sprintf(sending, "%x", tab_reg[i]);
add_to_list(&list_heads[i], sending);
}
usleep(100000); //wait for 100ms before next read
}
return arg; //removes warning
}
