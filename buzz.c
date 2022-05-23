#include <stdio.h>

#include "contiki.h"
#include "sys/rtimer.h"
#include "sys/etimer.h"

#include "board-peripherals.h"

#include <math.h>
#include <stdlib.h>
// processes
PROCESS(mpu_reading_proc, "Reading MPU data");
PROCESS(opt_reading_proc, "Reading light sensod");
PROCESS(wait_3_sec_proc, "Wait 3 sec");
PROCESS(main_process, "Main process");
AUTOSTART_PROCESSES(&main_process);
// events
static process_event_t imu_significant_event;
static process_event_t opt_significant_event;
static process_event_t waiting_end_event;
// states
typedef enum State {IDLE, BUZZ, WAIT} State;
static State STATE = IDLE;
// IMU reading
static struct rtimer imu_rtimer;
static int gyro_value;
static bool imu_significant = false;
// OPT reading
static struct rtimer opt_rtimer;
static int opt_value;
static bool opt_significant = false;
static bool opt_reading_error = false;

/*---------------------------------------------------------------------------*/
//                              SENSORS
/*---------------------------------------------------------------------------*/
//                              activate

static void init_opt_activate(void)
{
  SENSORS_ACTIVATE(opt_3001_sensor);
}

static void init_mpu_activate(void)
{
  mpu_9250_sensor.configure(SENSORS_ACTIVE, MPU_9250_SENSOR_TYPE_ALL);
}
//                              deactivate

static void init_opt_deactivate(void)
{
  SENSORS_DEACTIVATE(opt_3001_sensor);
}

static void init_mpu_deactivate(void)
{
  SENSORS_DEACTIVATE(mpu_9250_sensor);
}

/*---------------------------------------------------------------------------*/
//                         STATE IDLE : motion detection
/*---------------------------------------------------------------------------*/
// I implemented for one GYRO value only. I think it's way enough to decide a sig mo

int gyro_moving_average_calc(int window[], int size)
{
  static int target, congestion = 0, sum = 0;
  window[target] = gyro_value;
  // congestion to make difference btwn retrun values
  if(congestion % (size / 2) == 0){
    sum = 0;
    for (int i = 0; i < size; ++i) {
      sum += window[i]; 
    }
  }
  congestion++; target++;
  target %= size;
  return sum / size;
}

// if significant change happened in GYRO_X --> true
bool gyro_data_processing()
{
  bool gyro_sig = false;
  static int gyro_moving_avg, gyro_prev_moving_avg;
  // moving average window size
  // ADJUST to set response time, higher val means shorter response time and less accuracy caused by old data.
  static int gyro_window[12];
  // calculating the moving average
  // ADJUST to the above defined window's size
  gyro_moving_avg = gyro_moving_average_calc(gyro_window, 12);

  if(gyro_prev_moving_avg > 0){
    int diff = abs(gyro_moving_avg - gyro_prev_moving_avg);
    // significance decision; unit is in degrees, 180 to prevent dump values
    // ADJUST to set sensitivity, higher value means less
    if(diff > 17 && diff < 180)
      gyro_sig = true; // false by defaule
  }
  printf("ln 100: gyr ma = %d, gyr pma = %d\n", gyro_moving_avg, gyro_prev_moving_avg);
  gyro_prev_moving_avg = gyro_moving_avg;
  return gyro_sig;
}
  
// reading IMU sensors
static void get_mpu_reading_proc()
{
  int value;
  value = abs(mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Z)) / 100;
  if (value < 180)
    gyro_value = value;
  else  // prevent incorrect values
    gyro_value = 0;
}

void do_mpu_rtimer_timeout(struct rtimer *timer, void *ptr)
{
  if(!imu_significant){
    // rtimer period 50ms = 20Hz
    rtimer_set(&imu_rtimer, RTIMER_NOW() + RTIMER_SECOND / 20, 0, do_mpu_rtimer_timeout, NULL);
    get_mpu_reading_proc();
    // this may change imu_significant to true 
    imu_significant = gyro_data_processing();
  }
  if (imu_significant){
    init_mpu_deactivate(); 
    imu_significant_event = process_alloc_event();
    // post imu_significant_event exit event
    process_post(&main_process, imu_significant_event, NULL);
  }
  // else: end of recursion >> end of process
}


/*---------------------------------------------------------------------------*/
//                      ACTIVE STATES : light detection
/*---------------------------------------------------------------------------*/

int opt_weighted_moving_avg_calc(int window[], int size)
{
  static int target = 0; int wmavg = 0;
  window[target] = opt_value;
  
  for (int i = 0; i < size; ++i) {
    int weight = i + target + 1;
    weight %= size + 1;
    wmavg += window[i] * weight;
  }
  if(!opt_reading_error){
    target++; target %= size;
  }
  return wmavg / ((size * (size + 1)) / 2);
}

bool opt_sign_data_processing()
{
  static int opt_wght_mvng_avg, opt_prev_wght_mvng_avg;
  bool opt_sig = false;
  // moving average window size.
  static int opt_window[5] = {400, 400, 400, 400, 400};
  // calculating the moving average
  // ADJUST to the same value above line defined
  opt_wght_mvng_avg = opt_weighted_moving_avg_calc(opt_window, 5);

  // first run
  if(opt_prev_wght_mvng_avg > 0){
    // significance decision; unit is LUX.
    // ADJUST to set sensitivity, higher value means less
    int diff = abs(opt_wght_mvng_avg - opt_prev_wght_mvng_avg); 
    opt_sig = diff > 90;
  }
  printf("ln 172: ma = %d, pma = %d\n", opt_wght_mvng_avg, opt_prev_wght_mvng_avg);
  opt_prev_wght_mvng_avg = opt_wght_mvng_avg;

  return opt_sig;
}

static void get_light_reading()
{
  int value;
  value = opt_3001_sensor.value(0);
  if(value != CC26XX_SENSOR_READING_ERROR){
    opt_value = value / 100;
    opt_reading_error = false;
  }
  else
    opt_reading_error = true;

  init_opt_activate();
}

void do_opt_rtimer_timeout(struct rtimer *timer, void *ptr)
{
  if(!opt_significant){
    rtimer_set(&opt_rtimer, RTIMER_NOW() + RTIMER_SECOND / 4, 0, do_opt_rtimer_timeout, NULL);
    get_light_reading();
    opt_significant = (opt_sign_data_processing() && !opt_reading_error);
  }
  if(opt_significant){
    init_opt_deactivate(); 
    opt_significant_event = process_alloc_event();
    // post opt significant exit event
    process_post(&main_process, opt_significant_event, NULL);
  } 
  // else: end of recursion >> end of process
}


/*---------------------------------------------------------------------------*/
//                            MPU READING PROCESS
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(mpu_reading_proc, ev, data)
{ 
  static struct etimer timer_etimer;

  PROCESS_BEGIN();

  init_mpu_activate();
  // wait for the activation
  etimer_set(&timer_etimer, CLOCK_SECOND / 2);
  PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
  // recursive
  rtimer_set(&imu_rtimer, RTIMER_NOW() + RTIMER_SECOND / 20, 0,  do_mpu_rtimer_timeout, NULL);

  PROCESS_END();
}


/*---------------------------------------------------------------------------*/
//                            LIGHT SENSOR PROCESS
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(opt_reading_proc, ev, data)
{  
  PROCESS_BEGIN();

  init_opt_activate();
  rtimer_set(&opt_rtimer, RTIMER_NOW() + RTIMER_SECOND / 4, 0,  do_opt_rtimer_timeout, NULL);

  PROCESS_END();
}


/*---------------------------------------------------------------------------*/
//                            WAIT 3 SEC PROCESS
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(wait_3_sec_proc, ev, data)
{ 
  static struct etimer timer_etimer;

  PROCESS_BEGIN();

  etimer_set(&timer_etimer, CLOCK_SECOND * 3);
  PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
  waiting_end_event = process_alloc_event();
  process_post(&main_process, waiting_end_event, NULL);
    
  PROCESS_END();
}


/*---------------------------------------------------------------------------*/
//                         MAIN PROCESS : state machine
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(main_process, ev, data)
{  
  static struct etimer timer_etimer;

  PROCESS_BEGIN();

  STATE = IDLE;

  while(1)
  {
    if (STATE == IDLE) 
    {
      process_start(&mpu_reading_proc, NULL);
      PROCESS_WAIT_EVENT_UNTIL(ev==imu_significant_event);
      process_exit(&mpu_reading_proc);
      imu_significant = false;
      printf("idle >> buzz\n");
      STATE = BUZZ;
    }

    else if(STATE == BUZZ)
    {
      // start light sensoring
      process_start(&opt_reading_proc, NULL);
      process_start(&wait_3_sec_proc, NULL);   
      // buzzing
      buzzer_init();
      buzzer_start(2349);
      PROCESS_WAIT_EVENT_UNTIL(ev==opt_significant_event || ev == waiting_end_event);
      process_exit(&opt_reading_proc); process_exit(&wait_3_sec_proc);
      buzzer_stop();
      // change in light or not; thats the question
      if(opt_significant){
        opt_significant = false;
        // preventing unintentional light sensor activation in conseq of signif motion
        // opt_reading_proc still running for a very short time and picking up values
        etimer_set(&timer_etimer, CLOCK_SECOND);
        PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
        STATE = IDLE;
        printf("buzz >> idle\n");
      }
      else{
        printf("buzz >> wait\n");
        STATE = WAIT;
      }
    }

    else if(STATE == WAIT)
    {
      process_start(&opt_reading_proc, NULL);
      process_start(&wait_3_sec_proc, NULL);     
      PROCESS_WAIT_EVENT_UNTIL(ev==opt_significant_event || ev == waiting_end_event);
      process_exit(&opt_reading_proc); process_exit(&wait_3_sec_proc);
      // change in light or not; that's the question
      if(opt_significant){
        opt_significant = false;
        etimer_set(&timer_etimer, CLOCK_SECOND);
        PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
        printf("buzz >> idle\n");
        STATE = IDLE;
      }
      else{
        printf("wait >> buzz\n");
        STATE = BUZZ;
      }
    }
  }

  PROCESS_END();
}
