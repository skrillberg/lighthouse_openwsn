#include "opendefs.h"
#include "uinject.h"
#include "openqueue.h"
#include "openserial.h"
#include "packetfunctions.h"
#include "scheduler.h"
#include "IEEE802154E.h"
#include "idmanager.h"
//#include "accel_mimsy.h"
//#include "gpio.h"
//#include "headers/hw_gpio.h"
//#include <headers/hw_memmap.h>
//#include "flash_mimsy.h"
//#include "headers/hw_ints.h"
//#include "inv_mpu.h"
//=========================== variables =======================================

uinject_vars_t uinject_vars;
volatile uint8_t triggered;
volatile uint8_t debounce;
uint8_t me;
volatile 

static const uint8_t uinject_payload[]    = "uinject";
static const uint8_t uinject_dst_addr[]   = {
   0xbb, 0xbb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01
}; 

//=========================== prototypes ======================================

void uinject_timer_cb(opentimers_id_t id);
void uinject_task_cb(void);
void imu_int_cb(void);
void debounce_timer_cb(opentimers_id_t id);

//=========================== public ==========================================

void uinject_init(void) {
    me = (uint8_t)(idmanager_getMyID(ADDR_64B)->addr_64b[7]);	
    //mimsyIMUInit();
    /*
    struct int_param_s placeholder;
    mpu_init(&placeholder);
    //mpu_lp_accel_mode(1);
    mpu_lp_motion_interrupt(100, 300,20);

    triggered = 0;
    debounce = 0;
    volatile uint32_t i;

    //Delay to avoid pin floating problems
    for (i = 0xFFFF; i != 0; i--);

    // disable interrupts for PA7
    GPIOPinIntDisable(GPIO_A_BASE, GPIO_PIN_7);
    // clear the interrupt for PA7
    GPIOPinIntClear(GPIO_A_BASE, GPIO_PIN_7);

    // configures PA2 to be GPIO input
    GPIOPinTypeGPIOInput(GPIO_A_BASE, GPIO_PIN_7);

    // input GPIO on rising and falling edges
    GPIOIntTypeSet(GPIO_A_BASE, GPIO_PIN_7, GPIO_RISING_EDGE);

    // register the port level interrupt handler
    GPIOPortIntRegister(GPIO_A_BASE,imu_int_cb);

    // clear pin
    GPIOPinIntClear(GPIO_A_BASE, GPIO_PIN_7);
    IntPrioritySet(INT_GPIOA, 6<<5);
    // enable the interrupt (unmasks the interrupt bit)
    GPIOPinIntEnable(GPIO_A_BASE, GPIO_PIN_7);

    //ENABLE_INTERRUPTS();

    // 
    */
    // clear local variables
    memset(&uinject_vars,0,sizeof(uinject_vars_t));

    // register at UDP stack
    uinject_vars.desc.port              = WKP_UDP_INJECT;
    uinject_vars.desc.callbackReceive   = &uinject_receive;
    uinject_vars.desc.callbackSendDone  = &uinject_sendDone;
    openudp_register(&uinject_vars.desc);

    uinject_vars.period = UINJECT_PERIOD_MS;
    // start periodic timer
    uinject_vars.timerId = opentimers_create();

    opentimers_scheduleIn(
        uinject_vars.timerId,
        UINJECT_PERIOD_MS,
        TIME_MS,
        TIMER_PERIODIC,
        uinject_timer_cb
    );

    


    uinject_vars.debounceTimerId = opentimers_create();
/*
    opentimers_scheduleIn(
            uinject_vars.debounceTimerId,
            10000,
            TIME_MS,
            TIMER_PERIODIC,
            debounce_timer_cb
    );
*/
}

void uinject_sendDone(OpenQueueEntry_t* msg, owerror_t error) {
   openqueue_freePacketBuffer(msg);
}

void uinject_receive(OpenQueueEntry_t* pkt) {
   
   openqueue_freePacketBuffer(pkt);
   
   openserial_printError(
      COMPONENT_UINJECT,
      ERR_RCVD_ECHO_REPLY,
      (errorparameter_t)0,
      (errorparameter_t)0
   );
}

//=========================== private =========================================

/**
\note timer fired, but we don't want to execute task in ISR mode instead, push
   task to scheduler with CoAP priority, and let scheduler take care of it.
*/
void uinject_timer_cb(opentimers_id_t id){
 
   scheduler_push_task(uinject_task_cb,TASKPRIO_COAP);
}
/*
void imu_int_cb(void){
   GPIOPinIntClear(GPIO_A_BASE, GPIO_PIN_7);
   IMUData data;
   mimsyIMURead6Dof(&data);
   
   if(debounce == 0){
       debounce = 1;
       triggered = 1;
       scheduler_push_task(uinject_task_cb,TASKPRIO_COAP);
        
       opentimers_scheduleIn(
            uinject_vars.debounceTimerId,
            1500,
            TIME_MS,
            TIMER_ONESHOT,
            debounce_timer_cb
        );
       //SCHEDULER_WAKEUP();
    }
}
*/
void debounce_timer_cb(opentimers_id_t id){
    debounce = 0;
}

void uinject_task_cb(void) {
   OpenQueueEntry_t*    pkt;
   uint8_t              asnArray[5];
   
   // don't run if not synch
   if (ieee154e_isSynch() == FALSE) return;
   
   // don't run on dagroot
   if (idmanager_getIsDAGroot()) {
      opentimers_destroy(uinject_vars.timerId);
      return;
   }
   
   // if you get here, send a packet
   
   // get a free packet buffer
   pkt = openqueue_getFreePacketBuffer(COMPONENT_UINJECT);
   if (pkt==NULL) {
      openserial_printError(
         COMPONENT_UINJECT,
         ERR_NO_FREE_PACKET_BUFFER,
         (errorparameter_t)0,
         (errorparameter_t)0
      );
      return;
   }
   
   pkt->owner                         = COMPONENT_UINJECT;
   pkt->creator                       = COMPONENT_UINJECT;
   pkt->l4_protocol                   = IANA_UDP;
   pkt->l4_destination_port           = WKP_UDP_INJECT;
   pkt->l4_sourcePortORicmpv6Type     = WKP_UDP_INJECT;
   pkt->l3_destinationAdd.type        = ADDR_128B;
   memcpy(&pkt->l3_destinationAdd.addr_128b[0],uinject_dst_addr,16);
   
   // add payload
   packetfunctions_reserveHeaderSize(pkt,sizeof(uinject_payload)-1);
   memcpy(&pkt->payload[0],uinject_payload,sizeof(uinject_payload)-1);
   
   //packetfunctions_reserveHeaderSize(pkt,1);
   //pkt->payload[0] = triggered;

   packetfunctions_reserveHeaderSize(pkt,sizeof(uint16_t));
   pkt->payload[1] = me;
   pkt->payload[0] = triggered;
   uinject_vars.counter++;
   
   triggered = 0;

   packetfunctions_reserveHeaderSize(pkt,sizeof(asn_t));
   ieee154e_getAsn(asnArray);
   pkt->payload[0] = asnArray[0];
   pkt->payload[1] = asnArray[1];
   pkt->payload[2] = asnArray[2];
   pkt->payload[3] = asnArray[3];
   pkt->payload[4] = asnArray[4];
   
   if ((openudp_send(pkt))==E_FAIL) {
      openqueue_freePacketBuffer(pkt);
   }
}



