#include "config.h"

#if OPENWSN_USENSOR_NETWORK_C

#include "opendefs.h"
#include "usensor_network.h"
#include "sock.h"
#include "async.h"
#include "openserial.h"
#include "packetfunctions.h"
#include "scheduler.h"
#include "IEEE802154E.h"
#include "schedule.h"
#include "icmpv6rpl.h"
#include "idmanager.h"
#include "openrandom.h"
#include "uart_tx.h"
#include "msf.h"

//=========================== defines =========================================

#define USENSOR_NETWORK_TRAFFIC_RATE 1 ///> the value X indicates 1 packet/X minutes
#define PREAMBLE 's'

typedef struct {
    opentimers_id_t timerId;   ///< periodic timer which triggers transmission
    uint16_t counter;  ///< incrementing counter which is written into the packet
    uint16_t period;  ///< usensor_network packet sending period>
    bool busySendingUsensorNetwork;  ///< TRUE when busy sending an usensor_network
} usensor_network_vars_t;

static sock_udp_t _sock;
static usensor_network_vars_t usensor_network_vars;
static data_packet_t data_packet;

static const uint8_t usensor_network_dst_addr[] = {
        0xbb, 0xbb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x06, 0x06, 0x05, 0x05, 0x05, 0x05, 0x09
};

static uint16_t counter = 0;

//=========================== prototypes ======================================

void usensor_network_sock_handler(sock_udp_t *sock, sock_async_flags_t type, void *arg);

void _usensor_network_timer_cb(opentimers_id_t id);

void _usensor_network_task_cb(void);

//=========================== public ==========================================

void usensor_network_init(void) {
    uart_tx_init();

    // clear local variables
    memset(&_sock, 0, sizeof(sock_udp_t));
    memset(&usensor_network_vars, 0, sizeof(usensor_network_vars_t));

    sock_udp_ep_t local;
    local.family = AF_INET6;
    local.port = WKP_UDP_SENSOR_NETWORK;

    if (sock_udp_create(&_sock, &local, NULL, 0) < 0) {
        openserial_printf("Could not create socket\n");
        return;
    }

    openserial_printf("Created a UDP socket\n");

    sock_udp_set_cb(&_sock, usensor_network_sock_handler, NULL);

    // start periodic timer
    usensor_network_vars.period = USENSOR_NETWORK_PERIOD_MS;
    usensor_network_vars.timerId = opentimers_create(TIMER_GENERAL_PURPOSE, TASKPRIO_UDP);
    opentimers_scheduleIn(
            usensor_network_vars.timerId,
            USENSOR_NETWORK_PERIOD_MS,
            TIME_MS,
            TIMER_PERIODIC,
            _usensor_network_timer_cb
    );
}

//=========================== private =========================================

void usensor_network_sock_handler(sock_udp_t *sock, sock_async_flags_t type, void *arg) {
    (void) arg;

    char buf[50];

    if (type & SOCK_ASYNC_MSG_RECV) {
        sock_udp_ep_t remote;
        int16_t res;

        if ((res = sock_udp_recv(sock, buf, sizeof(buf), 0, &remote)) >= 0) {
            openserial_printf("Received %d bytes from remote endpoint:\n", res);
            openserial_printf(" - port: %d", remote.port);
            openserial_printf(" - addr: ", remote.port);
            for(int i=0; i < 16; i ++)
                openserial_printf("%x ", remote.addr.ipv6[i]);

            openserial_printf("\n\n");
            openserial_printf("Msg received: %s\n\n", buf);
        }
    }

    if (type & SOCK_ASYNC_MSG_SENT) {
        owerror_t error = *(uint8_t*)arg;
        if (error == E_FAIL) {
            LOG_ERROR(COMPONENT_USENSOR_NETWORK, ERR_MAXRETRIES_REACHED,
                    (errorparameter_t) usensor_network_vars.counter,
                    (errorparameter_t) 0);
        }
        // allow send next usensor_network packet
        usensor_network_vars.busySendingUsensorNetwork = FALSE;
    }
}


void _usensor_network_timer_cb(opentimers_id_t id) {
    // calling the task directly as the timer_cb function is executed in
    // task mode by opentimer already
    if (openrandom_get16b() < (0xffff / USENSOR_NETWORK_TRAFFIC_RATE)) {
        _usensor_network_task_cb();
    }
}

void _usensor_network_task_cb(void) {
    uart_tx_send_str("called1\n");
    uart_tx_send_str("called2\n");
    uart_tx_send_str("called3\n");
    open_addr_t parentNeighbor;
    bool foundNeighbor;

    // don't run if not synch
    if (ieee154e_isSynch() == FALSE) {
        return;
    }
    uart_tx_send_str("synchronized\n");

    // don't run on dagroot
    if (idmanager_getIsDAGroot()) {
        opentimers_destroy(usensor_network_vars.timerId);
        return;
    }
    uart_tx_send_str("not the root\n");

    foundNeighbor = icmpv6rpl_getPreferredParentEui64(&parentNeighbor);
    if (foundNeighbor == FALSE) {
        return;
    }
    uart_tx_send_str("neighbor\n");

    if (schedule_hasNegotiatedCellToNeighbor(&parentNeighbor, CELLTYPE_TX) == FALSE) {
        return;
    }
    uart_tx_send_str("schedule\n");

    if (usensor_network_vars.busySendingUsensorNetwork == TRUE) {
        // don't continue if I'm still sending a previous usensor_network packet
        return;
    }
    uart_tx_send_str("not busy\n");

    // if you get here, send a packet
    sock_udp_ep_t remote;
    remote.port = WKP_UDP_SENSOR_NETWORK;
    remote.family = AF_INET6;
    memcpy(remote.addr.ipv6, usensor_network_dst_addr, sizeof(usensor_network_dst_addr));
    memset(&data_packet, 0, sizeof(data_packet));
    data_packet.preamble = PREAMBLE;
    // add 16b addr
    memcpy(&data_packet.addr_16b, (idmanager_getMyID(ADDR_16B)->addr_16b), sizeof(uint16_t));
    // add sensor_payload
    data_packet.payload.sensor_type = SENSOR_POTENTIOMETRIC;
    data_packet.payload.data.potentiometric.data = counter;
    
    if (sock_udp_send(&_sock, &data_packet, sizeof(data_packet), &remote) > 0) {
        // set busySending to TRUE
        uart_tx_send_str("busy sending\n");
        usensor_network_vars.busySendingUsensorNetwork = TRUE;
    }
    ++counter;
    uart_tx_send_str("goodbye\n");
}

#endif /* OPENWSN_USENSOR_NETWORK_C */
