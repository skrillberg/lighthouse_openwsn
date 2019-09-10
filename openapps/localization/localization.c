#include "opendefs.h"
#include "localization.h"
#include "openqueue.h"
#include "openserial.h"
#include "packetfunctions.h"
#include "scheduler.h"
#include "IEEE802154E.h"
#include "idmanager.h"
#include "board.h"
#include "radio.h"
#include "leds.h"
#include "sctimer.h"
#include "source/gpio.h"
#include "headers/hw_rfcore_sfr.h"
#include "headers/hw_rfcore_sfr.h"
#include "headers/hw_rfcore_xreg.h"
#include <math.h>
#include <stdio.h>
#include "gptimer.h"
#include "sys_ctrl.h"
#include "headers/hw_gptimer.h"
#include "headers/hw_ints.h"
#include "gpio.h"
#include "interrupt.h"
#include "headers/hw_memmap.h"
#include "headers/hw_gpio.h"
#include "headers/hw_ioc.h"
#include "ioc.h"
#include "sixtop.h"
#include "schedule.h"
#include "servo.c"
#include "uart_mimsy.h"
#include "IEEE802154.h"
#include "IEEE802154_security.h"

//mimsy only
#include "accel_mimsy.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "ml_math_func.h"
#include <math.h>

//=========================== variables =======================================

volatile localization_vars_t localization_vars;

static const uint8_t localization_payload[]    = "localization";
static const uint8_t localization_dst_addr[]   = {
   0xbb, 0xbb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01
};

static const uint32_t gptmEdgeTimerBase = GPTIMER3_BASE;
static const uint32_t gptmFallingEdgeInt = INT_TIMER3B;
static const uint32_t gptmFallingEdgeEvent = GPTIMER_CAPB_EVENT;

static const uint32_t gptmTimer3AReg = 0x40033048;
static const uint32_t gptmTimer3BReg = 0x4003304C;

static const uint32_t timer_cnt_32 = 0xFFFFFFFF;
static const uint32_t timer_cnt_16 = 0xFFFF;
static const uint32_t timer_cnt_24 = 0xFFFFFF;

static const float sweep_velocity = PI / SWEEP_PERIOD_US;

volatile pulse_t valid_pulses[PULSE_TRACK_COUNT][PULSE_TRACK_COUNT];
volatile pulse_t pulses[PULSE_TRACK_COUNT];
volatile pulse_t asn_pulses[PULSE_TRACK_COUNT];
volatile uint8_t modular_ptr;
volatile uint32_t pulse_count;
volatile uint32_t count;
volatile uint32_t wsn_count;

volatile bool testRan;
volatile extern sixtop_vars_t sixtop_vars;
extern ieee154e_vars_t ieee154e_vars;
//=========================== prototypes ======================================
void localization_offset_timer_cb(opentimers_id_t id);
void localization_timer_cb(opentimers_id_t id);
void localization_task_cb(void);
void open_timer_init(void);
void precision_timers_init(void);
void input_edge_timers_init(void);
void mimsy_GPIO_falling_edge_handler(void);
void calc_eulers(float * quats, euler_t * roll, euler_t * pitch, euler_t * yaw);
location_t localize_mimsy(pulse_t *pulses_local, pulse_t *asn_pulses_local);
#define GYRO_FSR			2000 //gyro full scale range in deg/s
void orientation_sendEB(void);
void anchor_sendEB(int phi, uint32_t time, int16_t x, int16_t y);
void orientation_lookup_task(void);
void orientation_timer_cb(opentimers_id_t id);
void compass_cal_lookup(void);
void receive_cf_packet(void);

//=========================== private vars ====================================


typedef struct{
    int16_t x;
    int16_t y;
    int16_t z;
} mag_t;

int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
uint8_t mag_max_idx[3];
uint8_t mag_min_idx[3];
mag_t mag_max[MAG_CAL_SAMPLES];
mag_t mag_min[MAG_CAL_SAMPLES];
float heading; //heading in radians
int32_t hard_mag_bias[3];
//=========================== public ==========================================

void localization_init(void) {

    // clear local variables
    memset(&localization_vars,0,sizeof(localization_vars_t));
    
    testRan = false;
    count = 0; modular_ptr = 0; pulse_count = 0; wsn_count = 0;
    // initialize edges
    unsigned short int i;
    for (i = 0; i < PULSE_TRACK_COUNT; i++) {
        pulses[i] = (pulse_t){.rise = 0, .fall = 0, .type = -1};
        asn_pulses[i] = (pulse_t){.rise = 0, .fall = 0, .type = -1};
    }
    unsigned short int j;
    for (i = 0; i < PULSE_TRACK_COUNT; i++) {
        for (j = 0; j < PULSE_TRACK_COUNT; j++) {
            valid_pulses[i][j].rise = 0; valid_pulses[i][j].fall = 0; valid_pulses[i][j].type = -1;
        }
    }

    // register at UDP stack
    localization_vars.desc.port              = WKP_UDP_LOCALIZATION;
    localization_vars.desc.callbackReceive   = &localization_receive;
    localization_vars.desc.callbackSendDone  = &localization_sendDone;
    openudp_register(&localization_vars.desc);

    volatile uint32_t _i;

    //Delay to avoid pin floating problems
    for (_i = 0xFFFF; _i != 0; _i--);

    // configure_pins();
    precision_timers_init();
    open_timer_init();
    //motor_init(0,1,25,GPIO_D_BASE,GPIO_PIN_1);

    //mimsy specific imu code
    struct int_param_s placeholder;
    mpu_init(&placeholder);
    //mimsyIMUInit();
    mpu_set_sensors(INV_XYZ_ACCEL|INV_XYZ_GYRO|INV_XYZ_COMPASS); //turn on sensor
    mpu_set_compass_sample_rate(50);
    //mpu_configure_fifo(INV_XYZ_ACCEL | INV_XYZ_GYRO);
    mpu_set_accel_fsr(8); //set fsr for accel
    mpu_set_gyro_fsr(2000); //set fsr for accel
    //mimsyDmpBegin(); //this will cause a crash if the delay in i2c reads is too long, 1 ms was too long

    //mpu_lp_accel_mode(1);
    //mpu_lp_motion_interrupt(100, 300,20);
	float fquats[4] = {0,0,0,0};
    long quat[4];
	short gyro[3] = {0,0,0};
	short accel[3] = {0,0,0};
    short sensors;
    unsigned char more;
    unsigned long timestamp;
    uint16_t dur = ieee154e_getSlotDuration();
    uartMimsyInit(); //initial mimsy printf
    //while(dmp_read_fifo((gyro), (accel), (quat),&(timestamp), &sensors, &more)!=0){
    //mimsyPrintf(" hello world \n");
	//}
	//*********************euler angle conversion*****************************************************************
   //pitch control
   fquats[0]=(float)quat[0]/(float)0x40000000;
   fquats[1]=(float)quat[1]/(float)0x40000000;
   fquats[2]=(float)quat[2]/(float)0x40000000;
   fquats[3]=(float)quat[3]/(float)0x40000000;
   /*
   //mag = sqrtf( fquats[1] * fquats[1] + fquats[2] * fquats[2] + fquats[3] * fquats[3]);
   inv_q_norm4(fquats);
   urocket_vars.pitch_last = urocket_vars.pitch.flt; //save previous state
   urocket_vars.pitch.flt = asinf( 2*(fquats[0]*fquats[2]-fquats[3]*fquats[1])); //computes sin of pitch

   //gyro yaw
   urocket_vars.yaw_last = urocket_vars.yaw.flt;
   urocket_vars.yaw.flt = atan2f(2*(fquats[0] * fquats[3] + fquats[1] * fquats[2]),1 - 2*(fquats[2]*fquats[2] + fquats[3]*fquats[3]));

   //roll control
   urocket_vars.roll_last = urocket_vars.roll.flt;
   urocket_vars.roll.flt=  atan2f(2 * (fquats[0]*fquats[1] + fquats[2] * fquats[3]) ,(1 -2*(fquats[1] * fquats[1] +fquats[2]*fquats[2])));
   mimsyPrintf("\n Roll: %d. Pitch: %d, Yaw: %d",(int)(roll*100),(int)(pitch*100), (int)(yaw*100));
    */

   for(i = 0 ; i< MAG_CAL_SAMPLES; i++){
        mag_max[i].x = -32767;
        mag_max[i].y = -32767;
        mag_max[i].z = -32767;
        mag_min[i].x = 32767;
        mag_min[i].y = 32767;
        mag_min[i].z = 32767;
   }
    for (i=0;i<3;i++){
        mag_max_idx[i] = 0;
        mag_min_idx[i] = 0;
    }
    compass_cal_lookup();

    //set constant location if nonmobile anchor mote
    if (ANCHOR_MOTE && CRAZYFLIE == 0){
    	sixtop_vars.location.x = ANCHOR_X;
    	sixtop_vars.location.y = ANCHOR_Y;
    }

}

void compass_cal_lookup(void){
    uint8_t mote;
    open_addr_t * addr = idmanager_getMyID(ADDR_16B);
    mote = addr->addr_16b[1];
    switch(mote){

        case(48):
            hard_mag_bias[0] = -41;
            hard_mag_bias[1] = 236;
            hard_mag_bias[2] = -507;

            if(LEGO_DRONE_MOUNT){
                //hard_mag_bias[0] = 460;
                //hard_mag_bias[1] = -83;
                //hard_mag_bias[2] = -1025;
                hard_mag_bias[0] = 455;
                hard_mag_bias[1] = -40;
                hard_mag_bias[2] = -1025;
            }

            break;
        case(58):
            hard_mag_bias[0] = -23;
            hard_mag_bias[1] = -156;
            hard_mag_bias[2] = -111;
            break;
            

        
    }
}

void calc_eulers(float * fquats, euler_t* roll, euler_t * pitch, euler_t* yaw){
   inv_q_norm4(fquats);

       uint8_t i;
   pitch->flt = asinf( 2*(fquats[0]*fquats[2]-fquats[3]*fquats[1])); //computes sin of pitch

   //gyro yaw

   yaw->flt = atan2f(2*(fquats[0] * fquats[3] + fquats[1] * fquats[2]),1 - 2*(fquats[2]*fquats[2] + fquats[3]*fquats[3]));

   //roll control

   roll->flt=  atan2f(2 * (fquats[0]*fquats[1] + fquats[2] * fquats[3]) ,(1 -2*(fquats[1] * fquats[1] +fquats[2]*fquats[2])));
   //mimsyPrintf("Roll: %d. Pitch: %d, Yaw: %d \n",(int)(roll->flt*1000),(int)(pitch->flt*1000), (int)(yaw->flt*1000));
	// get data
    short compass[3];
    
	if (mpu_get_compass_reg(compass, NULL) != 0 && PRINT) {
		mimsyPrintf("Failed to read compass data\n");
	}else{
        for (i=0;i<3;i++){
            mag_max_idx[i] = 0;
            mag_min_idx[i] = 0;
        }
        if(compass[0] > mag_max[mag_max_idx[0]].x){
            mag_max[mag_max_idx[0]].x = compass[0];
            mag_max_idx[0] = (mag_max_idx[0]+1) % MAG_CAL_SAMPLES;
        }
        if(compass[1] > mag_max[mag_max_idx[1]].y){
            mag_max[mag_max_idx[1]].y = compass[1];
            mag_max_idx[1] = (mag_max_idx[1]+1) % MAG_CAL_SAMPLES;
        }
        if(compass[2] > mag_max[mag_max_idx[2]].z){
            mag_max[mag_max_idx[2]].z = compass[2];
            mag_max_idx[2] = (mag_max_idx[2]+1) % MAG_CAL_SAMPLES;
        }
        if(compass[0] < mag_min[mag_min_idx[0]].x){
            mag_min[mag_min_idx[0]].x = compass[0];
            mag_min_idx[0] = (mag_min_idx[0]+1) % MAG_CAL_SAMPLES;
        }
        if(compass[1] < mag_min[mag_min_idx[1]].y){
            mag_min[mag_min_idx[1]].y = compass[1];
            mag_min_idx[1] = (mag_min_idx[1]+1) % MAG_CAL_SAMPLES;
        }
        if(compass[2] < mag_min[mag_min_idx[2]].z){
            mag_min[mag_min_idx[2]].z = compass[2];
            mag_min_idx[2] = (mag_min_idx[2]+1) % MAG_CAL_SAMPLES;
        }
    }
    
    mag_bias[0] = (mag_max[0].x + mag_min[0].x)/2;
    mag_bias[1] = (mag_max[0].y + mag_min[0].y)/2;
    mag_bias[2] = (mag_max[0].z + mag_min[0].z)/2;
	// convert from hardware units to uT
	float mag_sens = 0.5859;
	float mag[3];
	mag[0] = ((float)compass[0]-hard_mag_bias[0]) * mag_sens;
	mag[1] = ((float)compass[1]-hard_mag_bias[1]) * mag_sens;
	mag[2] = ((float)compass[2]-hard_mag_bias[2]) * mag_sens;
    heading = atan2f(mag[0],mag[1]);
    if(heading < 0){
     heading = (2*PI + heading);
    }
    //mimsyPrintf("Compass Data: %d, %d, %d \n",(int) mag[0],(int)mag[1],(int)mag[2]);
    //

    /*
    mimsyPrintf("Compass Maxes z:");
    for(i=0;i<MAG_CAL_SAMPLES; i++){
        mimsyPrintf("%d, ",mag_min[i].x);
    }*/
    open_addr_t * addr = idmanager_getMyID(ADDR_16B);
    
    //mimsyPrintf("Mote %d, Compass Bias: %d, %d, %d; ",addr->addr_16b[1],mag_bias[0], mag_bias[1], mag_bias[2]);
    //mimsyPrintf("Compass Data: %d, %d, %d",(int) compass[0],(int)compass[1],(int)compass[2]);
    //mimsyPrintf("\n");
    //mimsyPrintf("Compass Data: %d, %d, %d, Heading: %d \n",(int) compass[0],(int)compass[1],(int)compass[2], (int)(heading*1000));
   
}

void open_timer_init(void){
    localization_vars.period = LOCALIZATION_PERIOD_MS;
    // start periodic timer
    localization_vars.timerId = opentimers_create();
    opentimers_scheduleIn(
        localization_vars.timerId,
        LOCALIZATION_PERIOD_MS,
        TIME_MS,
        TIMER_PERIODIC,
        localization_timer_cb
    );

    localization_vars.orientationTimerId = opentimers_create();

    opentimers_scheduleIn(
        localization_vars.orientationTimerId,
        ORIENTATION_PERIOD_MS,
        TIME_MS,
        TIMER_PERIODIC,
        orientation_timer_cb
    );


    

}

void precision_timers_init(void){
   // SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_GPT2); // enables timer1 module
    SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_GPT3); // enables timer3 module
    if(LIGHTHOUSE_MOTE == 0){
        input_edge_timers_init();
    }
    // TimerConfigure(gptmPeriodTimerBase, GPTIMER_CFG_PERIODIC_UP);
    // TimerLoadSet(gptmPeriodTimerBase,GPTIMER_A,timer_cnt_32);
    // TimerEnable(gptmPeriodTimerBase,GPTIMER_A);
}

// NOTE: route single gpio pin to both timers and see if that still works
void input_edge_timers_init(void) {
    GPIOPinTypeTimer(GPIO_A_BASE,GPIO_PIN_2); // enables hw muxing of pin inputs
    GPIOPinTypeTimer(GPIO_A_BASE,GPIO_PIN_5); // enables hw muxing of pin inputs

    TimerConfigure(gptmEdgeTimerBase, GPTIMER_CFG_SPLIT_PAIR |
          GPTIMER_CFG_A_CAP_TIME_UP | GPTIMER_CFG_B_CAP_TIME_UP); // configures timer3a/b as 16-bit edge timers

    ///TimerConfigure(GPTIMER2_BASE, GPTIMER_CFG_A_PERIODIC_UP ); // configures timer1a/b as 16-bit edge timers

    TimerPrescaleSet(gptmEdgeTimerBase,GPTIMER_A,0); // add prescaler to timer3a (24-bit)
    TimerPrescaleSet(gptmEdgeTimerBase,GPTIMER_B,0); // add prescaler to timer3b (24-bit)

    //TimerPrescaleSet(GPTIMER2_BASE,GPTIMER_A,0); // add prescaler to timer2a (24-bit)


    TimerLoadSet(gptmEdgeTimerBase,GPTIMER_A,timer_cnt_24);
    TimerLoadSet(gptmEdgeTimerBase,GPTIMER_B,timer_cnt_24);

    //TimerLoadSet(GPTIMER2_BASE,GPTIMER_A,timer_cnt_24);


    // FIXME: can we use the same gpio pin for both??
    IOCPinConfigPeriphInput(GPIO_A_BASE, GPIO_PIN_2, IOC_GPT3OCP1); // map gpio pin output to timer3a
    IOCPinConfigPeriphInput(GPIO_A_BASE, GPIO_PIN_5, IOC_GPT3OCP2); // map gpio pin output to timer3b

    // NOTE: the TS3633-CM1 Prototyping Module inverts rising and falling edges when light pulses are received,
    // so negative edges correspond to rising edges, and positive edges correspond to falling edges
    TimerControlEvent(gptmEdgeTimerBase, GPTIMER_A, GPTIMER_EVENT_POS_EDGE); // set timer3a to capture rising edges (inverted by PCB)
    TimerControlEvent(gptmEdgeTimerBase, GPTIMER_B, GPTIMER_EVENT_NEG_EDGE); // set timer3b to capture falling edges (inverted by PCB)

    TimerIntDisable(gptmEdgeTimerBase, gptmFallingEdgeEvent);
    TimerIntClear(gptmEdgeTimerBase, gptmFallingEdgeEvent);

    // set up interrupt for falling edge timer
    TimerIntRegister(gptmEdgeTimerBase, GPTIMER_B, mimsy_GPIO_falling_edge_handler);
    TimerIntEnable(gptmEdgeTimerBase, gptmFallingEdgeEvent);
    // IntPrioritySet(gptmFallingEdgeInt, 7<<5);
    IntEnable(gptmFallingEdgeInt);

    ENABLE_INTERRUPTS();

    TimerEnable(gptmEdgeTimerBase,GPTIMER_BOTH);
    //TimerEnable(GPTIMER2_BASE,GPTIMER_A);
   // TimerSynchronize(GPTIMER0_BASE, GPTIMER_3A_SYNC | GPTIMER_3B_SYNC  | GPTIMER_2A_SYNC);

}

void localization_sendDone(OpenQueueEntry_t* msg, owerror_t error) {
   openqueue_freePacketBuffer(msg);
}

void localization_receive(OpenQueueEntry_t* pkt) {

   openqueue_freePacketBuffer(pkt);

   openserial_printError(
      COMPONENT_localization,
      ERR_RCVD_ECHO_REPLY,
      (errorparameter_t)0,
      (errorparameter_t)0
   );
}

//record current time of start of tsch slot in localization ticks
void loc_record_start_of_slot(void){
    localization_vars.start_of_slot = TimerValueGet(GPTIMER2_BASE, GPTIMER_A);
    TimerSynchronize(GPTIMER0_BASE, GPTIMER_3A_SYNC | GPTIMER_3B_SYNC  | GPTIMER_2A_SYNC);
}

//=========================== private =========================================

float get_period_us(uint32_t start, uint32_t end) {
    if (start > end) {
        // do overflow arithmetic
        return ((float) (end + (timer_cnt_24 - start))) / CLOCK_SPEED_MHZ;
    } else {
        return ((float) (end - start)) / CLOCK_SPEED_MHZ;
    }
}

/** Returns a number defining our 3 information bits: skip, data, axis.
  Given by our pulse length in microseconds (us). */
unsigned short int sync_bits(float duration) {
  return (unsigned short int) (48*duration - 2501) / 500;
}

void mimsy_GPIO_falling_edge_handler(void) {
    TimerIntClear(gptmEdgeTimerBase, gptmFallingEdgeEvent);

    // uint32_t time = TimerValueGet(gptmPeriodTimerBase, GPTIMER_A);
    // uint32_t rise = HWREG(gptmTimer3AReg) & 0xFFFF;
    // uint32_t fall = HWREG(gptmTimer3BReg) & 0xFFFF;

    // shift previous pulses and write to struct
    pulses[modular_ptr].rise = (uint32_t)(HWREG(gptmTimer3AReg)); // TimerValueGet(gptmEdgeTimerBase, GPTIMER_A);
    pulses[modular_ptr].fall = (uint32_t)(HWREG(gptmTimer3BReg)); // TimerValueGet(gptmEdgeTimerBase, GPTIMER_B);
    
    

    //ADDED BY KILBERG: classify for sync pulse so we can capture asn and timer offset. This should only be done if 
    //this mote is the sync mote, but this needs to be implemented
    float period = get_period_us(pulses[modular_ptr].rise, pulses[modular_ptr].fall);
    


    ieee154e_getAsn(sixtop_vars.sync_pulse_asn);


	//calculate offset from asn to current time using 32 mhz timer. This var name is misleading
	sixtop_vars.sync_pulse_timer_offset =  opentimers_getValue()-ieee154e_vars.startOfSlotReference;

	//calculate current time in 32 mHz ticks: asn*slot_duration + intraslot_offest
	uint32_t curr_time = sixtop_vars.sync_pulse_asn[3] * 16777216 * ieee154e_vars.slotDuration
                    + sixtop_vars.sync_pulse_asn[2] * 65536 * ieee154e_vars.slotDuration 
                    + sixtop_vars.sync_pulse_asn[1] * 256 * ieee154e_vars.slotDuration 
                    + sixtop_vars.sync_pulse_asn[0] * ieee154e_vars.slotDuration
                    + sixtop_vars.sync_pulse_timer_offset; 

	//detect pulse, disregard pulses < 60 us because those are glitches or optitrak pulses, and robolighthouse scans
	//should be slower than 60 us
	if(localization_vars.orientation_received == 0 && localization_vars.pulse_detected ==0 && period > 200.0){
		localization_vars.orientation_pulse_time = curr_time;
		localization_vars.pulse_detected = 1;
		if(PRINT){
			//mimsyPrintf("Pulse Detected \n");
		}
	}
}

void localization_offset_timer_cb(opentimers_id_t id){

    scheduler_push_task(localization_task_cb,TASKPRIO_COAP);
}
/**
\note timer fired, but we don't want to execute task in ISR mode instead, push
   task to scheduler with CoAP priority, and let scheduler take care of it.
*/
void localization_timer_cb(opentimers_id_t id){
    // count += 1;
    /*
    scheduleEntry_t* slot = schedule_getCurrentScheduleEntry();

    if((slot->type != CELLTYPE_SERIALRX) && (schedule_getNumOfSlotsByType(CELLTYPE_TXRX) != 0)){
        uint8_t slots_from_rx = schedule_getNumOfSlotsByType(CELLTYPE_TXRX) - slot->slotOffset;
        opentimers_scheduleIn(
            localization_vars.offsetTimerId,
            17 + slots_from_rx * ieee154e_vars.slotDuration,
            TIME_MS,
            TIMER_ONESHOT,
            localization_offset_timer_cb
        ); 
       openserial_printError(
         COMPONENT_localization,
         ERR_NO_FREE_PACKET_BUFFER,
         (errorparameter_t)(uint16_t) ieee154e_vars.slotDuration,
         (errorparameter_t)0
       );

       openserial_printError(
         COMPONENT_localization,
         ERR_NO_FREE_PACKET_BUFFER,
         (errorparameter_t)(uint16_t) schedule_getNumberOfFreeEntries(),
         (errorparameter_t)0
       );
       openserial_printError(
         COMPONENT_localization,
         ERR_NO_FREE_PACKET_BUFFER,
         (errorparameter_t)(uint16_t) 7777,
         (errorparameter_t)0
       );
       
    }else{
    scheduler_push_task(localization_task_cb,TASKPRIO_COAP);
    // SCHEDULER_WAKEUP();
    }
    */if(LIGHTHOUSE_MOTE == 1){
        scheduler_push_task(localization_task_cb,TASKPRIO_COAP);
    }
}
void orientation_timer_cb(opentimers_id_t id){
    scheduler_push_task(orientation_lookup_task,TASKPRIO_COAP);
}
void orientation_lookup_task(void){
    //look for uart packets if connected to crazyflie
	if(CRAZYFLIE){
		receive_cf_packet();
	}
    if(localization_vars.pulse_detected && localization_vars.orientation_received){
        uint8_t idx;
        uint8_t found;
        if(PRINT){
        	mimsyPrintf("Pulse detected and eb received \n");
        }
        found = 0;
        for(idx = 0; idx < ORIENTATION_SAMPLE_N-1; idx++ ){
        	if(LIGHTHOUSE_MOTE == 0 && PRINT){
        		//mimsyPrintf("Orientation No Pulse: %d, %d \n",localization_vars.orientations_tmp[idx].fields.time,
                //                                localization_vars.orientations_tmp[idx].fields.orientation);
        	}
            //search for the nearest time in the table
            if(localization_vars.orientation_pulse_time > localization_vars.orientations_tmp[idx].fields.time && localization_vars.orientation_pulse_time < localization_vars.orientations_tmp[idx+1].fields.time){
                if(idx < ORIENTATION_SAMPLE_N -1){
                	int next = localization_vars.orientations_tmp[idx+1].fields.orientation;
                	int prev = localization_vars.orientations_tmp[idx].fields.orientation;
                	int32_t orientation_diff = next - prev;
                    //calc angle wrap if angle diff magnitude>PI and orientations are different signs
                    if(orientation_diff > PI){

                    	orientation_diff =  orientation_diff - 2*PI;
                    }
					else if(orientation_diff < -PI){
						orientation_diff = 2*PI + orientation_diff;
					}

                	uint32_t time_diff = (localization_vars.orientations_tmp[idx+1].fields.time -  localization_vars.orientations_tmp[idx].fields.time +1);
                    float interp_slope = ((float)orientation_diff)/(1+time_diff);
                    int32_t orientation =   localization_vars.orientations_tmp[idx].fields.orientation + 
                                            interp_slope*(localization_vars.orientation_pulse_time - localization_vars.orientations_tmp[idx].fields.time) ;
                    char crazypacket[15];
                    uint8_t synch_packet[4]  = {0xde,0xad,0xbe,0xef};
                    int i_print = 0;

                    if(orientation > 6283){
                    	orientation = (orientation-6283);
                    }else if(orientation < 0){
                    	orientation = 6283 + orientation;
                    }
                    //orientation =  localization_vars.orientations_tmp[idx].fields.orientation;
                    found = 1;
                    if(PRINT){
                    	mimsyPrintf("Orientation: %d, %d \n", localization_vars.orientation_pulse_time, orientation);
                    }
                    if (CRAZYFLIE){
                        //create crazyflie lh packet
                        crazypacket[0] = 100; //preamble for lh packet type

                        //random number, i'll put length for now
                        crazypacket[1] = 'a';
                        //x location of measurer
                        crazypacket[2] = (localization_vars.orientation_x & 0xff00) >> 8;
                        crazypacket[3] = localization_vars.orientation_x & 0x00ff;

                        //y location of measurer
                        crazypacket[4] = (localization_vars.orientation_y & 0xff00) >> 8;
                        crazypacket[5] = localization_vars.orientation_y & 0x00ff;

                        //orientation: milliradians in int32
                        crazypacket[6] = (orientation & 0xFF000000) >> 24;
                        crazypacket[7] = (orientation & 0x00FF0000)>> 16;
                        crazypacket[8] = (orientation & 0x0000FF00) >> 8;
                        crazypacket[9] = orientation & 0xFF;
                        
                        //time of pulse in openwsn asn time
                        crazypacket[10] = (localization_vars.orientation_pulse_time & 0xFF000000) >> 24;
                        crazypacket[11] = (localization_vars.orientation_pulse_time & 0xFF0000) >> 16;
                        crazypacket[12] = (localization_vars.orientation_pulse_time & 0xFF00) >> 8;
                        crazypacket[13] = localization_vars.orientation_pulse_time & 0xFF;

                        //end of frame byte
                        crazypacket[14] = 122;

                        
                        for(int i = 0; i <15; i++){
                        	uart_mimsy_writeByte(crazypacket[i]);
                        }

                        
                       //print orientation and location in crazyflie lighthouse measurement format
                          
                      	//mimsyPrintf("Orientation: %d, %d \n", localization_vars.orientation_pulse_time, orientation);
                    }
                    //if anchor, send eb to ligthouse robot
                    if(ANCHOR_MOTE){
                    	anchor_sendEB(orientation,localization_vars.orientation_pulse_time,sixtop_vars.location.x,sixtop_vars.location.y);
                    	if(PRINT){
                    		mimsyPrintf("Anchor EB Sent \n");
                    	}
                    }

                }
            }
        }
        if(LIGHTHOUSE_MOTE == 0 && PRINT){
        mimsyPrintf("Orientation Not Found: %d, %d \n",localization_vars.orientations_tmp[ORIENTATION_SAMPLE_N-1].fields.time,
                                            localization_vars.orientations_tmp[ORIENTATION_SAMPLE_N-1].fields.orientation);
        }
        localization_vars.pulse_detected = 0;
        localization_vars.orientation_received = 0;
        if(PRINT){
        	//mimsyPrintf("Calculation State Exited; Found: %d \n",found);
        }
        found = 0;
    }
   /* else if(localization_vars.pulse_detected){
        if(PRINT){
        	mimsyPrintf("Pulse detected \n");
        }
    }*/
    else if(!localization_vars.pulse_detected && localization_vars.orientation_received){
    	localization_vars.orientation_received = 0; //clear flag since we got an orientation before we got a laser pulse
    	if(PRINT){
    		//mimsyPrintf("Orientation no Pulse \n");
            int edx;
            for(edx = 0; edx < ORIENTATION_SAMPLE_N; edx++ ){
                //mimsyPrintf("Orientation No Pulse: %d, %d \n",localization_vars.orientations_tmp[edx].fields.time,
                //                                    localization_vars.orientations_tmp[edx].fields.orientation);
            }
    	}
    }

}
void localization_task_cb(void) {

	//check for anchor measurements
	if(localization_vars.anchor_received){

		//an orientation packet from anchor mote was received
		uint32_t time = localization_vars.anchor_measurement_time;
		int32_t phi = localization_vars.anchor_measurement_phi;
		localization_vars.anchor_received = 0;
		//mimsyPrintf("anchor measurement received, x: %d, y: %d, phi: %d \n",localization_vars.anchor_measurement_x,localization_vars.anchor_measurement_y,localization_vars.anchor_measurement_phi);

        //create crazyflie lh packet
		uint8_t crazypacket[15];
        crazypacket[0] = 100; //preamble for lh packet type

        //random number, i'll put length for now
        crazypacket[1] = 'a';
        //x location of measurer
        crazypacket[2] = (localization_vars.anchor_measurement_x & 0xff00) >> 8;
        crazypacket[3] = localization_vars.anchor_measurement_x & 0x00ff;

        //y location of measurer
        crazypacket[4] = (localization_vars.anchor_measurement_y & 0xff00) >> 8;
        crazypacket[5] = localization_vars.anchor_measurement_y & 0x00ff;

        //orientation: milliradians in int32
        crazypacket[6] = (localization_vars.anchor_measurement_phi & 0xFF000000) >> 24;
        crazypacket[7] = (localization_vars.anchor_measurement_phi & 0x00FF0000)>> 16;
        crazypacket[8] = (localization_vars.anchor_measurement_phi & 0x0000FF00) >> 8;
        crazypacket[9] = localization_vars.anchor_measurement_phi & 0xFF;

        //time of pulse in openwsn asn time
        crazypacket[10] = (localization_vars.anchor_measurement_time & 0xFF000000) >> 24;
        crazypacket[11] = (localization_vars.anchor_measurement_time & 0xFF0000) >> 16;
        crazypacket[12] = (localization_vars.anchor_measurement_time & 0xFF00) >> 8;
        crazypacket[13] = localization_vars.anchor_measurement_time & 0xFF;

        //end of frame byte
        crazypacket[14] = 122;

        for(int i = 0; i <15; i++){
        	uart_mimsy_writeByte(crazypacket[i]);
        }


	}
    scheduleEntry_t* slot = schedule_getCurrentScheduleEntry();

	float fquats[4] = {0,0,0,0};
    long quat[4];
	short gyro[3] = {0,0,0};
	short accel[3] = {0,0,0};
    short sensors;
    unsigned char more;
    unsigned long timestamp;
   // int code = dmp_read_fifo((gyro), (accel), (quat),&(timestamp), &sensors, &more);
   // mpu_get_fifo_config(&sensors);
    uint8_t asn[5];
    ieee154e_getAsn(asn);      
    uint32_t asn_offset =  opentimers_getValue()-ieee154e_vars.startOfSlotReference;   

    uint32_t curr_time = asn[3] * 16777216 * ieee154e_vars.slotDuration 
                + asn[2] * 65536 * ieee154e_vars.slotDuration 
                + asn[1] * 256 * ieee154e_vars.slotDuration 
                + asn[0] * ieee154e_vars.slotDuration
                + asn_offset; 


    //while(dmp_read_fifo((gyro), (accel), (quat),&(timestamp), &sensors, &more)!=0){
    //mimsyPrintf(" dmp fifo error\n");
	//}
    //convert quats from hardware units to float
    fquats[0]=(float)quat[0]/(float)0x40000000;
    fquats[1]=(float)quat[1]/(float)0x40000000;
    fquats[2]=(float)quat[2]/(float)0x40000000;
    fquats[3]=(float)quat[3]/(float)0x40000000;

    euler_t roll;
    euler_t pitch;
    euler_t yaw;
    //if(code == 0 && sensors != 0){
    	
        //save orientations and timestamps

    //}
    calc_eulers(fquats, &roll, &pitch , &yaw);

    //if attached to crazyflie, use crazyflie estimation of heading
    if(CRAZYFLIE == 1){
    	localization_vars.orientations[localization_vars.orientation_idx].fields.orientation = localization_vars.crazyflie_heading;
    }else{
    	localization_vars.orientations[localization_vars.orientation_idx].fields.orientation = (int32_t)(heading*1000); //in milliradians
    }
    localization_vars.orientations[localization_vars.orientation_idx].fields.time = curr_time; //save time
    localization_vars.orientation_idx++;
    if(localization_vars.orientation_idx >= ORIENTATION_SAMPLE_N && LIGHTHOUSE_MOTE == 1){
       localization_vars.orientation_idx = 0;
        orientation_sendEB();
    }

 

    IMUData data;
    //mimsyIMURead6Dof(&data);

    //mimsyPrintf("dmp data: ");
    /*
    openserial_printError(
     COMPONENT_localization,
     ERR_NO_FREE_PACKET_BUFFER,
     (errorparameter_t)(uint16_t) ( 11111),
     (errorparameter_t)0
   );
   openserial_printError(
     COMPONENT_localization,
     ERR_NO_FREE_PACKET_BUFFER,
     (errorparameter_t)(uint16_t) ( slot->type),
     (errorparameter_t)0
   );
   openserial_printError(
     COMPONENT_localization,
     ERR_NO_FREE_PACKET_BUFFER,
     (errorparameter_t)(uint16_t) ( schedule_getMaxActiveSlots()),
     (errorparameter_t)0
   );
    */


   OpenQueueEntry_t*    pkt;
   // uint8_t              asnArray[5];

   // don't run if not synch
   if (ieee154e_isSynch() == FALSE) return;

   // don't run on dagroot
   if (idmanager_getIsDAGroot()) {
      opentimers_destroy(localization_vars.timerId);
      return;
   }

   union {
      float flt;
      unsigned char bytes[4];
   } x;

   union {
      float flt;
      unsigned char bytes[4];
   } y;

   union {
      float flt;
      unsigned char bytes[4];
   } z;

   pulse_t pulses_local[PULSE_TRACK_COUNT];
   pulse_t asn_pulses_local[PULSE_TRACK_COUNT];
   uint8_t ptr = modular_ptr;

   wsn_count += 1;

   unsigned short int i;
   for (i = ptr; i < ptr + PULSE_TRACK_COUNT; i++) {
     pulses_local[i-ptr].rise = pulses[i%PULSE_TRACK_COUNT].rise;
     pulses_local[i-ptr].fall = pulses[i%PULSE_TRACK_COUNT].fall;
     pulses_local[i-ptr].type = pulses[i%PULSE_TRACK_COUNT].type;
   }

   for (i = ptr; i < ptr + PULSE_TRACK_COUNT; i++) {
     asn_pulses_local[i-ptr].rise = asn_pulses[i%PULSE_TRACK_COUNT].rise;
     asn_pulses_local[i-ptr].fall = asn_pulses[i%PULSE_TRACK_COUNT].fall;
     asn_pulses_local[i-ptr].type = asn_pulses[i%PULSE_TRACK_COUNT].type;
   }
   // perform localization calculations

    location_t loc = localize_mimsy(pulses_local, asn_pulses_local);
   //sixtop_vars.location.x = (uint16_t) pulses_local[0].fall;
    /*
  openserial_printError(
     COMPONENT_localization,
     ERR_NO_FREE_PACKET_BUFFER,
     (errorparameter_t)(uint16_t) opentimers_getValue()-ieee154e_vars.startOfSlotReference,
     (errorparameter_t)0
  );
  openserial_printError(
     COMPONENT_localization,
     ERR_NO_FREE_PACKET_BUFFER,
     (errorparameter_t)(uint16_t)ieee154e_vars.slotDuration,
     (errorparameter_t)0
  );*/

   if (!loc.valid) return;

   //x.flt = *r * sinf(*theta) * cosf(*phi);
   //y.flt = *r * sinf(*theta) * sinf(*phi);
   //z.flt = *r * cosf(*theta);

   // x.flt = *phi; y.flt = *theta; z.flt = *r;
   x.flt = (float) pulse_count; y.flt = (float) wsn_count; z.flt = (float) count;

   // if you get here, send a packet

   // get a free packet buffer
    /*
   pkt = openqueue_getFreePacketBuffer(COMPONENT_localization);
   if (pkt==NULL) {
      openserial_printError(
         COMPONENT_localization,
         ERR_NO_FREE_PACKET_BUFFER,
         (errorparameter_t)0,
         (errorparameter_t)0
      );
      return;
   }

   pkt->owner                         = COMPONENT_localization;
   pkt->creator                       = COMPONENT_localization;
   pkt->l4_protocol                   = IANA_UDP;
   pkt->l4_destination_port           = WKP_UDP_LOCALIZATION;
   pkt->l4_sourcePortORicmpv6Type     = WKP_UDP_LOCALIZATION;
   pkt->l3_destinationAdd.type        = ADDR_128B;
   memcpy(&pkt->l3_destinationAdd.addr_128b[0],localization_dst_addr,16);

   // add payload
   packetfunctions_reserveHeaderSize(pkt,sizeof(localization_payload)-1);
   memcpy(&pkt->payload[0],localization_payload,sizeof(localization_payload)-1);

   // add payload
   packetfunctions_reserveHeaderSize(pkt,3*sizeof(float));
   pkt->payload[0] = x.bytes[0];
   pkt->payload[1] = x.bytes[1];
   pkt->payload[2] = x.bytes[2];
   pkt->payload[3] = x.bytes[3];
   pkt->payload[4] = y.bytes[0];
   pkt->payload[5] = y.bytes[1];
   pkt->payload[6] = y.bytes[2];
   pkt->payload[7] = y.bytes[3];
   pkt->payload[8] = z.bytes[0];
   pkt->payload[9] = z.bytes[1];
   pkt->payload[10] = z.bytes[2];
   pkt->payload[11] = z.bytes[3];

   packetfunctions_reserveHeaderSize(pkt,4*sizeof(float));
	union {
		float _flt;
		unsigned char bytes[4];
	} phi;

	union {
		float _flt;
		unsigned char bytes[4];
	} theta;

	union {
		float _flt;
		unsigned char bytes[4];
	} r_vert;

	union {
		float _flt;
		unsigned char bytes[4];
	} r_horiz;

	phi._flt = loc.phi; theta._flt = loc.theta; r_vert._flt = loc.r_vert; r_horiz._flt = loc.r_horiz;

	pkt->payload[0] = phi.bytes[0];
   pkt->payload[1] = phi.bytes[1];
   pkt->payload[2] = phi.bytes[2];
   pkt->payload[3] = phi.bytes[3];
   pkt->payload[4] = theta.bytes[0];
   pkt->payload[5] = theta.bytes[1];
   pkt->payload[6] = theta.bytes[2];
   pkt->payload[7] = theta.bytes[3];
   pkt->payload[8] = r_vert.bytes[0];
   pkt->payload[9] = r_vert.bytes[1];
   pkt->payload[10] = r_vert.bytes[2];
   pkt->payload[11] = r_vert.bytes[3];
   pkt->payload[12] = r_horiz.bytes[0];
   pkt->payload[13] = r_horiz.bytes[1];
   pkt->payload[14] = r_horiz.bytes[2];
   pkt->payload[15] = r_horiz.bytes[3];

   packetfunctions_reserveHeaderSize(pkt,sizeof(asn_t));
   pkt->payload[0] = loc.asn[0];
   pkt->payload[1] = loc.asn[1];
   pkt->payload[2] = loc.asn[2];
   pkt->payload[3] = loc.asn[3];
   pkt->payload[4] = loc.asn[4];

   if ((openudp_send(pkt))==E_FAIL) {
      openqueue_freePacketBuffer(pkt);
   } else {
      count += 1;
   }
    */
}

void receive_cf_packet(void){

    union{
        int16_t val;
        uint8_t bytes[2];
    } x;

    union{
        int16_t val;
        uint8_t bytes[2];
    } y;

    union{
        int32_t val;
        uint8_t bytes[4];
    } phi;
   
    uint8_t packet[10];
    uint8_t byte = 0;
    byte = uart_mimsy_readByte();
    
    //check for correct start flag from crazyflie
    if(byte != 's'){
        return;
    }

    //receive the 10 uart bytes from packet
    packet[0] = byte;
    uint8_t count = 1;
    while( count <10){
        //mimsyPrintf("%c",byte);
        byte = uart_mimsy_readByte();
        packet[count] = byte;
        count++;
    }
   // mimsyPrintf("%c \n",byte);
    
    //check for correct end flag from crazyflie
    if(byte!= 'z'){
        return;
    }else{
        //convert byte to location and heading
        x.bytes[0] = packet[1];
        x.bytes[1] = packet[2];

        y.bytes[0] = packet[3];
        y.bytes[1] = packet[4];

        phi.bytes[0] = packet[5];
        phi.bytes[1] = packet[6];
        phi.bytes[2] = packet[7];
        phi.bytes[3] = packet[8];

        //update mimsy state
        localization_vars.crazyflie_heading = phi.val;
        sixtop_vars.location.x = x.val;
        sixtop_vars.location.y = y.val;

        //update locations with crazyflie state
        //mimsyPrintf("Valid State Packet! x: %d, y: %d, phi: %d \n",x.val,y.val,phi.val);
    } 

}

float get_period_us_32(uint32_t start, uint32_t end) {
    if (start > end) {
        // do overflow arithmetic
        return ((float) (end + (timer_cnt_32 - start))) / CLOCK_SPEED_MHZ;
    } else {
        return ((float) (end - start)) / CLOCK_SPEED_MHZ;
    }
}




float distance_fit_horiz(float time_us) {
  float E = 0.2218; float c0 = -0.3024; float c1 = 18.2991;
  return E + c1 / (time_us - c0 * sweep_velocity);
}

float distance_fit_vert(float time_us) {
  float E = 0.3074; float c0 = 0.9001; float c1 = 16.1908;
  return E + c1 / (time_us - c0 * sweep_velocity);
}

location_t localize_mimsy(pulse_t *pulses_local, pulse_t *asn_pulses_local) {
    location_t loc = (location_t){.phi = 0, .theta = 0,
											.r_vert = 0, .r_horiz = 0,
											.asn =  {0, 0, 0, 0, 0}, .valid = 0};
    ieee154e_getAsn(loc.asn);

    uint8_t init_sync_index = PULSE_TRACK_COUNT;
    
    
    
    // loop through and classify our pulses
    Pulses valid_seq_a[4] = { Sync, Horiz, Sync, Vert };
    Pulses valid_seq_b[4] = { Sync, Vert, Sync, Horiz };
    uint8_t sweep_axes_check = 0; uint8_t i;
    for (i = 0; i < PULSE_TRACK_COUNT; i++) {
        float period = get_period_us(pulses_local[i].rise, pulses_local[i].fall);
        if (period < MIN_SYNC_PERIOD_US) { // sweep pulse
            if (init_sync_index != PULSE_TRACK_COUNT) {
                float parent_period = get_period_us(pulses_local[i-1].rise, pulses_local[i-1].fall);
                int axis = (sync_bits(parent_period) & 0b001) + 1;
                pulses_local[i].type = axis; // 1 if horizontal, 2 if vertical

                int ind = i - init_sync_index;
                if (axis == ((int) valid_seq_a[ind]) || axis == ((int) valid_seq_b[ind])) {
                    sweep_axes_check += axis; // check for 1 horizontal, 1 vertical sweep
                } else {
                openserial_printError(
                     COMPONENT_localization,
                     ERR_NO_FREE_PACKET_BUFFER,
                     (errorparameter_t)(uint16_t) 60,
                     (errorparameter_t)0
                  );
                    return loc;
                }
            }
        } else if (period < MAX_SYNC_PERIOD_US) { // sync pulse
            if (init_sync_index == PULSE_TRACK_COUNT) {
            	init_sync_index = i; // set initial valid sync pulse index
            }
            
            pulses_local[i].type = (int) Sync;
        } else { // neither
            pulses_local[i].type = -1;
            openserial_printError(
                 COMPONENT_localization,
                 ERR_NO_FREE_PACKET_BUFFER,
                 (errorparameter_t)(uint16_t) 61,
                 (errorparameter_t)0
              );
            return loc;
        }
    }

    if (init_sync_index == PULSE_TRACK_COUNT || sweep_axes_check != 3) return loc;

    for (i = init_sync_index; i < PULSE_TRACK_COUNT-1; i++) {
        pulse_t curr_pulse = pulses_local[i];
        pulse_t next_pulse = pulses_local[i+1];

        switch(next_pulse.type) {
            case ((int) Sync):
                break;
            case ((int) Horiz):
                loc.phi = get_period_us(curr_pulse.fall, next_pulse.rise) * sweep_velocity;
                loc.r_horiz = distance_fit_horiz(get_period_us(next_pulse.rise, next_pulse.fall));
                //code for computing asn based stuff, sometimes it is off by 16.6 ms, or one cycle at 60hz
                uint32_t periods_from_ref = (uint32_t)((float)((asn_pulses_local[i+1].rise*100 - sixtop_vars.ref_sync_pulse)) / (sixtop_vars.sync_pulse_period*100));
                uint32_t proj_sync_fall = sixtop_vars.ref_sync_pulse/100 + sixtop_vars.sync_pulse_period*periods_from_ref;


                uint32_t desync;
                int32_t difference;
                difference = -proj_sync_fall + asn_pulses_local[i].fall; //difference is projected - actual
                float time_since_sync = asn_pulses_local[i].fall - sixtop_vars.ref_sync_pulse/100; //need to divide by 100x 
                uint32_t corrected_proj_sync = (uint32_t)(proj_sync_fall +  localization_vars.slope_sum/32768/30.51 * time_since_sync/localization_vars.sync_cycle_count);
                 
                float slope = difference * 30.51 /(time_since_sync)*32768; //in us/s

                if(periods_from_ref > 60 && (slope >20 ) && slope <40){
                    localization_vars.slope_sum += slope;
                    localization_vars.sync_cycle_count += 1;
                }
                /*
                openserial_printError(
                     COMPONENT_localization,
                     ERR_NO_FREE_PACKET_BUFFER,
                     (errorparameter_t)(uint16_t) (loc.phi * 1000),
                     (errorparameter_t)0
                );

                openserial_printError(
                     COMPONENT_localization,
                     ERR_NO_FREE_PACKET_BUFFER,
                     (errorparameter_t)(uint16_t)(localization_vars.slope_sum/32768/30.51 * time_since_sync/localization_vars.sync_cycle_count ),
                     (errorparameter_t)0
                );

                openserial_printError(
                     COMPONENT_localization,
                     ERR_NO_FREE_PACKET_BUFFER,
                     (errorparameter_t)(uint16_t)(localization_vars.slope_sum/localization_vars.sync_cycle_count ),
                     (errorparameter_t)0
                );
                openserial_printError(
                     COMPONENT_localization,
                     ERR_NO_FREE_PACKET_BUFFER,
                     (errorparameter_t)(uint16_t)(corrected_proj_sync ),
                     (errorparameter_t)0
                );
                openserial_printError(
                     COMPONENT_localization,
                     ERR_NO_FREE_PACKET_BUFFER,
                     (errorparameter_t)(uint16_t) (proj_sync_fall),
                     (errorparameter_t)0
                );


                openserial_printError(
                     COMPONENT_localization,
                     ERR_NO_FREE_PACKET_BUFFER,
                     (errorparameter_t)(uint16_t)(asn_pulses_local[i].fall),
                     (errorparameter_t)0
                );
                openserial_printError(
                     COMPONENT_localization,
                     ERR_NO_FREE_PACKET_BUFFER,
                     (errorparameter_t)(uint16_t)(22222),
                     (errorparameter_t)0
                );*/
                if ((next_pulse.rise - curr_pulse.fall)/GPT_TICKS_PER_SC_TICK > (asn_pulses_local[i+1].rise - proj_sync_fall)){
                    desync = (next_pulse.rise - curr_pulse.fall)/GPT_TICKS_PER_SC_TICK -  (asn_pulses_local[i+1].rise - proj_sync_fall);
                } else{
                    desync = (asn_pulses_local[i+1].rise - proj_sync_fall)*GPT_TICKS_PER_SC_TICK - (next_pulse.rise - curr_pulse.fall);
                }

                if(periods_from_ref < LIGHTHOUSE_KA_PERIODS){
                    union {
                        uint32_t data[8];
                        uint8_t bytes[32];
                    } sync_data;
                    union { 
                        uint8_t bytes[4];
                        float    float_rep;
                    } float_un;

                    sync_data.data [0] = periods_from_ref;
                    sync_data.data[1] = proj_sync_fall;
                    sync_data.data[2] =  asn_pulses_local[i].fall;
                    sync_data.data[3] = (uint32_t)(loc.phi*1000);
                    sync_data.data[4] = (uint32_t) (((asn_pulses_local[i+1].rise - proj_sync_fall)*GPT_TICKS_PER_SC_TICK) * sweep_velocity/32.0*1000);  
                    sync_data.data[5] = (uint32_t) localization_vars.slope_sum/localization_vars.sync_cycle_count; 
                    sync_data.data[6] = (uint32_t) corrected_proj_sync;
                    sync_data.data[7] = (uint32_t) (((asn_pulses_local[i+1].rise - corrected_proj_sync)*GPT_TICKS_PER_SC_TICK) * sweep_velocity/32.0*1000);  

                }

                break;
            case ((int) Vert):
                loc.theta = get_period_us(curr_pulse.fall, next_pulse.rise) * sweep_velocity;
                loc.r_vert = distance_fit_vert(get_period_us(next_pulse.rise, next_pulse.fall));
                break;
            default:
                openserial_printError(
                     COMPONENT_localization,
                     ERR_NO_FREE_PACKET_BUFFER,
                     (errorparameter_t)(uint16_t) 63,
                     (errorparameter_t)0
                  );
                return loc;
                break;
        }
    }

    loc.valid = true;
    //sixtop_vars.location.x = loc.phi*1000;
    //sixtop_vars.location.y = loc.theta*1000;
    //sixtop_vars.location.z = loc.r_horiz*1000;

        uint16_t diff;
        if((uint16_t)sixtop_vars.ref_sync_pulse > (uint16_t)(sixtop_vars.current_sync_pulse_time*100)){

            diff = (uint16_t) sixtop_vars.ref_sync_pulse - (uint16_t)(sixtop_vars.current_sync_pulse_time*100);

        }else{
            diff = (uint16_t)(sixtop_vars.current_sync_pulse_time *100) - (uint16_t) sixtop_vars.ref_sync_pulse ; 
        }
          /*
          openserial_printError(
             COMPONENT_localization,
             ERR_NO_FREE_PACKET_BUFFER,
             (errorparameter_t)diff,
             (errorparameter_t)0
          );  
        openserial_printError(
             COMPONENT_localization,
             ERR_NO_FREE_PACKET_BUFFER,
             (errorparameter_t)(uint16_t) (sixtop_vars.sync_pulse_period*100),
             (errorparameter_t)0
          );*/

    return loc;
}

/**
\brief Send an EB.

This is one of the MAC management tasks. This function inlines in the
timers_res_fired() function, but is declared as a separate function for better
readability of the code.
*/
port_INLINE void orientation_sendEB() {
    OpenQueueEntry_t* eb;
    uint8_t     i;
    uint8_t     eb_len;
    uint16_t    temp16b;
   
   
    // if I get here, I will send an EB
    
    // get a free packet buffer
    eb = openqueue_getFreePacketBuffer(COMPONENT_localization);
    if (eb==NULL) {
        openserial_printError(
            COMPONENT_SIXTOP,
            ERR_NO_FREE_PACKET_BUFFER,
            (errorparameter_t)0,
            (errorparameter_t)0
        );
        return;
    }
   
    // declare ownership over that packet
    eb->creator = COMPONENT_SIXTOP;
    eb->owner   = COMPONENT_SIXTOP;
    

    //reserve space for location IE. This should be before all other reserve
    //header operation because otherwise it will not be appended to the end of the IEs

    uint16_t loc_len = 88;   //length is payload plus two bytes of descriptor stuff
    packetfunctions_reserveHeaderSize(eb,loc_len); //undid to get working
    temp16b = IEEE802154E_DESC_TYPE_SHORT | 
              (0x49 << IEEE802154E_DESC_SUBID_SHORT_MLME_IE_SHIFT) | 
               (loc_len-2) ;

  //printf("%d \n",temp16b);
    //sixtop_vars.location.x = 15;
    /*openserial_printError(
    COMPONENT_SIXTOP,
    ERR_NO_FREE_PACKET_BUFFER,
    (errorparameter_t)((uint8_t)(sixtop_vars.location.x & 0x00ff)),
    (errorparameter_t)0
);*/

    uint8_t loc_stream[88]; //6 bytes in the beginning for asn

    loc_stream[0] = (uint8_t)(temp16b & 0x00ff);
    loc_stream[1] = (uint8_t)((temp16b & 0xff00)>>8);
    int tuple = 0;
    for(tuple = 0; tuple < (loc_len-8)/sizeof(localization_vars.orientations[0].bytes);tuple++){
        int byte = 0;
        int size = sizeof(localization_vars.orientations[0].bytes);
        for(byte = 0; byte<sizeof(localization_vars.orientations[0].bytes); byte++){
            loc_stream[8+tuple*size+byte] = localization_vars.orientations[tuple].bytes[byte];
        }
    }
    

    //load location header
    for (i=0;i<loc_len;i++){
        eb->payload[i]   = loc_stream[i]; //undid to get working
	  //printf("%x\n",eb->payload[i]);
    }

    packetfunctions_reserveHeaderSize(eb,2); //2 bytes for header payload descripter and 6 empty bytes for asn
    temp16b = loc_len | IEEE802154E_PAYLOAD_DESC_GROUP_ID_MLME | IEEE802154E_PAYLOAD_DESC_TYPE_MLME;
    eb->payload[0] = (uint8_t)(temp16b & 0x00ff);
    eb->payload[1] = (uint8_t)((temp16b & 0xff00)>>8);


    // Keep a pointer to where the ASN will be
    // Note: the actual value of the current ASN and JP will be written by the
    //    IEEE802.15.4e when transmitting
    eb->l2_ASNpayload               = &eb->payload[EB_ASN0_OFFSET]; 
  
    // some l2 information about this packet
    eb->l2_frameType                     = IEEE154_TYPE_BEACON;
    eb->l2_nextORpreviousHop.type        = ADDR_16B;
    eb->l2_nextORpreviousHop.addr_16b[0] = 0xff;
    eb->l2_nextORpreviousHop.addr_16b[1] = 0xff;
    
    //I has an IE in my payload
    eb->l2_payloadIEpresent = TRUE;

    // set l2-security attributes
    eb->l2_securityLevel   = IEEE802154_SECURITY_LEVEL_BEACON;
    eb->l2_keyIdMode       = IEEE802154_SECURITY_KEYIDMODE;
    eb->l2_keyIndex        = IEEE802154_security_getBeaconKeyIndex();

    // put in queue for MAC to handle
    sixtop_send_internal(eb,eb->l2_payloadIEpresent);
   
    // I'm now busy sending an EB
    sixtop_vars.busySendingEB = TRUE;
    
}

/**
\brief Send an EB.

This is one of the MAC management tasks. This function inlines in the
timers_res_fired() function, but is declared as a separate function for better
readability of the code.
*/
port_INLINE void anchor_sendEB(int phi, uint32_t time, int16_t x, int16_t y) {
    OpenQueueEntry_t* eb;
    uint8_t     i;
    uint8_t     eb_len;
    uint16_t    temp16b;


    // if I get here, I will send an EB

    // get a free packet buffer
    eb = openqueue_getFreePacketBuffer(COMPONENT_localization);
    if (eb==NULL) {
        openserial_printError(
            COMPONENT_SIXTOP,
            ERR_NO_FREE_PACKET_BUFFER,
            (errorparameter_t)0,
            (errorparameter_t)0
        );
        return;
    }

    // declare ownership over that packet
    eb->creator = COMPONENT_SIXTOP;
    eb->owner   = COMPONENT_SIXTOP;


    //reserve space for location IE. This should be before all other reserve
    //header operation because otherwise it will not be appended to the end of the IEs

    uint16_t loc_len = 88;   //length is payload plus two bytes of descriptor stuff
    packetfunctions_reserveHeaderSize(eb,loc_len); //undid to get working
    temp16b = IEEE802154E_DESC_TYPE_SHORT |
              (0x50 << IEEE802154E_DESC_SUBID_SHORT_MLME_IE_SHIFT) |
               (loc_len-2) ;

  //printf("%d \n",temp16b);
    //sixtop_vars.location.x = 15;
    /*openserial_printError(
    COMPONENT_SIXTOP,
    ERR_NO_FREE_PACKET_BUFFER,
    (errorparameter_t)((uint8_t)(sixtop_vars.location.x & 0x00ff)),
    (errorparameter_t)0
);*/

    uint8_t loc_stream[88]; //6 bytes in the beginning for asn

    loc_stream[0] = (uint8_t)(temp16b & 0x00ff);
    loc_stream[1] = (uint8_t)((temp16b & 0xff00)>>8);
    int tuple = 0;
    for(tuple = 0; tuple < (loc_len-8)/sizeof(localization_vars.orientations[0].bytes);tuple++){
        int byte = 0;
        int size = sizeof(localization_vars.orientations[0].bytes);
        for(byte = 0; byte<sizeof(localization_vars.orientations[0].bytes); byte++){
            loc_stream[8+tuple*size+byte] = localization_vars.orientations[tuple].bytes[byte];
        }
    }

    //x location
    loc_stream[8] = (uint8_t)((x & 0xff00) >> 8);
    loc_stream[9] = (uint8_t)(x & 0x00ff);

    //y location
    loc_stream[10] = (y & 0xff00) >> 8;
    loc_stream[11] = y & 0x00ff;

    //orientation: milliradians in int32
    loc_stream[12] = (phi & 0xFF000000) >> 24;
    loc_stream[13] = (phi & 0x00FF0000) >> 16;
    loc_stream[14] = (phi & 0x0000FF00) >> 8;
    loc_stream[15] = (phi & 0xFF);

    //time of pulse in openwsn asn time
    loc_stream[16] = (time & 0xFF000000) >> 24;
    loc_stream[17] = (time & 0xFF0000) >> 16;
    loc_stream[18] = (time & 0xFF00) >> 8;
    loc_stream[19] = (time & 0xFF);


    //load location header
    for (i=0;i<loc_len;i++){
        eb->payload[i]   = loc_stream[i]; //undid to get working
	  //printf("%x\n",eb->payload[i]);
    }

    packetfunctions_reserveHeaderSize(eb,2); //2 bytes for header payload descripter and 6 empty bytes for asn
    temp16b = loc_len | IEEE802154E_PAYLOAD_DESC_GROUP_ID_MLME | IEEE802154E_PAYLOAD_DESC_TYPE_MLME;
    eb->payload[0] = (uint8_t)(temp16b & 0x00ff);
    eb->payload[1] = (uint8_t)((temp16b & 0xff00)>>8);


    // Keep a pointer to where the ASN will be
    // Note: the actual value of the current ASN and JP will be written by the
    //    IEEE802.15.4e when transmitting
    eb->l2_ASNpayload               = &eb->payload[EB_ASN0_OFFSET];

    // some l2 information about this packet
    eb->l2_frameType                     = IEEE154_TYPE_BEACON;
    eb->l2_nextORpreviousHop.type        = ADDR_16B;
    eb->l2_nextORpreviousHop.addr_16b[0] = 0xff;
    eb->l2_nextORpreviousHop.addr_16b[1] = 0xff;

    //I has an IE in my payload
    eb->l2_payloadIEpresent = TRUE;

    // set l2-security attributes
    eb->l2_securityLevel   = IEEE802154_SECURITY_LEVEL_BEACON;
    eb->l2_keyIdMode       = IEEE802154_SECURITY_KEYIDMODE;
    eb->l2_keyIndex        = IEEE802154_security_getBeaconKeyIndex();

    // put in queue for MAC to handle
    sixtop_send_internal(eb,eb->l2_payloadIEpresent);

    // I'm now busy sending an EB
    sixtop_vars.busySendingEB = TRUE;

}
