#ifndef __LOCALIZATION_H
#define __LOCALIZATION_H

/**
\addtogroup AppUdp
\{
\addtogroup localization
\{
*/

#include "opentimers.h"
#include "openudp.h"

//=========================== define ==========================================

#define LOCALIZATION_PERIOD_MS 200
#define PULSE_TRACK_COUNT 5

#define MIN_SYNC_PERIOD_US 52
#define MAX_SYNC_PERIOD_US 138

#define PI 3.14159265f
#define SWEEP_PERIOD_US 8333.333333f
#define DIODE_WIDTH_CM 0.45f // FIXME: measure for PCB

#define LIGHTHOUSE_HORIZONTAL_SYNC_PERIOD 546.13333333f  //in 32khz ticks
#define LIGHTHOUSE_KA_PERIODS 3600
#define CLOCK_SPEED_MHZ 32.0f
#define GPT_TICKS_PER_SC_TICK 976.5625f
#define ORIENTATION_SAMPLE_N  10
#define ORIENTATION_SIZE 8
#define ORIENTATION_PERIOD_MS 100
//=========================== typedef =========================================

//=========================== variables =======================================
typedef union  {
    struct {
        uint32_t time;
        int32_t orientation;
    } fields;

    uint8_t bytes[8];
}orientation_map_euler_t;

typedef struct {
    uint32_t time;
    float quats[4];
} orientation_map_quaternion_t;

typedef struct {
   opentimers_id_t      timerId;  ///< periodic timer which triggers transmission
   uint16_t             counter;  ///< incrementing counter which is written into the packet
   uint16_t              period;  ///< localization packet sending period>
   udp_resource_desc_t     desc;  ///< resource descriptor for this module, used to register at UDP stack
   uint32_t          sync_count;
   uint32_t          sync_cycle_count;
   uint32_t          desync_slope;
   uint32_t          start_of_slot; //records start of current slot with respect to localization timer 
   uint32_t          last_sync_time; //time of last horiz sync pulse
   uint32_t          sync_cycle_start; // time when the sync cycle started, used to calculate drift
   opentimers_id_t      offsetTimerId;  ///< periodic timer which triggers transmission
   float             slope_sum; 
   orientation_map_euler_t   orientations[ORIENTATION_SAMPLE_N];
   orientation_map_euler_t   orientations_tmp[ORIENTATION_SAMPLE_N];
   uint8_t          orientation_idx;
   uint8_t          pulse_detected;     //state variable: have i seen a laser pulse
   uint8_t          orientation_received;   //state variable: have i received an orientation EB
   uint32_t         orientation_pulse_time;
   opentimers_id_t      orientationTimerId;  ///< periodic timer which triggers transmission
       
} localization_vars_t;

typedef struct {
   uint32_t                rise;
   uint32_t                fall;
   int                     type; // -1 for unclassified, 0 for Sync, 1 for Horiz, 2 for Vert
} pulse_t;

 
typedef union {
     float flt;
     unsigned char bytes[4];
   } euler_t;



typedef struct {
	float                    phi;
	float                  theta;
	float                 r_vert;
	float				      r_horiz;
	uint8_t               asn[5];
	int						  valid;
} location_t;

typedef enum {
   Sync, Horiz, Vert,
} Pulses;

//=========================== prototypes ======================================

void localization_init(void);
void localization_sendDone(OpenQueueEntry_t* msg, owerror_t error);
void localization_receive(OpenQueueEntry_t* msg);
void loc_record_start_of_slot(void); 
/**
\}
\}
*/

#endif
