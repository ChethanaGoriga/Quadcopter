#include <AP_HAL.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_InertialSensor.h>
#include <AP_ADC.h>
#include <AP_GPS.h>
#include <AP_AHRS.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <AP_Airspeed.h>
#include <AP_Baro.h>
//#include <AP_Declination.h>
//#include <GCS_MAVLink.h>
#include <Filter.h>
//#include <SITL.h>
#include <AP_Buffer.h>
#include<PID.h>
#include <APM_PI.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Empty.h>
#include <AP_InertialNav.h>
#include <APM_PI.h>
#include <AP_Common.h>




const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
AP_InertialSensor_MPU6000 ins;
AP_Baro_MS5611 bmp085(&AP_Baro_MS5611::spi);

#elif CONFIG_HAL_BOARD == HAL_BOARD_APM1
AP_ADC_ADS7844 adc;
AP_InertialSensor_Oilpan ins( &adc );
AP_Baro_BMP085 bmp085;
#else
AP_InertialSensor_Stub ins;
#endif

AP_Compass_HMC5843 compass;

GPS *g_gps;

AP_GPS_Auto GPS(&g_gps);

// choose which AHRS system to use
AP_AHRS_DCM  ahrs(&ins, g_gps);
//AP_AHRS_MPU6000  ahrs(&ins, g_gps);		// only works with APM2

//AP_Baro_BMP085_HIL barometer;


//Navigation nav((GPS) *g_gps);



#define HIGH 1
#define LOW 0

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
 # define A_LED_PIN        27
 # define C_LED_PIN        25
 # define LED_ON           LOW
 # define LED_OFF          HIGH
 # define MAG_ORIENTATION  AP_COMPASS_APM2_SHIELD
#else
 # define A_LED_PIN        37
 # define C_LED_PIN        35
 # define LED_ON           HIGH
 # define LED_OFF          LOW
 # define MAG_ORIENTATION  AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD
#endif

AP_InertialNav inertialnav(&ahrs, &ins, &bmp085, &g_gps);

static void flash_leds(bool on)
{
    hal.gpio->write(A_LED_PIN, on ? LED_OFF : LED_ON);
    hal.gpio->write(C_LED_PIN, on ? LED_ON : LED_OFF);
}

// Radio min/max values for each stick for my radio (worked out at beginning of article)
#define RC_THR_MIN   990
#define RC_YAW_MIN   1008
#define RC_YAW_MAX   1985
#define RC_PIT_MIN   1007
#define RC_PIT_MAX   1985
#define RC_ROL_MIN   1008
#define RC_ROL_MAX   1983

// Motor numbers definitions
#define MOTOR_F   2    // Front left   == front 
#define MOTOR_R   0    // Front right ==right
#define MOTOR_L   1    // back left == left
#define MOTOR_B   3    // back right == back

// Arduino map function
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))

// PID array (6 pids, two for each axis)
PID pids[12];
#define PID_PITCH_RATE 0
#define PID_ROLL_RATE 1
#define PID_PITCH_STAB 2
#define PID_ROLL_STAB 3
#define PID_YAW_RATE 4
#define PID_YAW_STAB 5
#define PID_ALT_STAB 6
#define PID_ALT_RATE 7
#define PID_LAT_STAB 8
#define PID_LON_STAB 9
#define PID_LAT_RATE 10
#define PID_LON_RATE 11


float roll,pitch,yaw;
float gyroRoll,gyroPitch,gyroYaw;
float dx,dy,velx,vely;
float pos_x0=0,pos_y0=0;
float gps_alt,gps_spd;
uint32_t last_update;

float alt=0;
float zoff;
float altf=0,altf0=0;
float velz=0;
float count=0;

uint32_t timer;
float home_lat,home_lon;
float lat,lon;

void setup() 
{ 
  timer = hal.scheduler->micros();
  hal.uartC->begin(9600,45,1);
  //hal.uartA->begin(9600);
  
  // PID Configuration
  pids[PID_PITCH_RATE].kP(2.45);
  pids[PID_PITCH_RATE].kI(0.5);
  pids[PID_PITCH_RATE].imax(50);
  
  pids[PID_ROLL_RATE].kP(2.45);
  pids[PID_ROLL_RATE].kI(0.5);
  pids[PID_ROLL_RATE].imax(50);
  
  pids[PID_YAW_RATE].kP(2.45);//0.7
  pids[PID_YAW_RATE].kI(0);//1
  pids[PID_YAW_RATE].imax(50);
  
  pids[PID_PITCH_STAB].kP(1.81);
  pids[PID_ROLL_STAB].kP(1.81);
  pids[PID_YAW_STAB].kP(1.81);//5
  
  pids[PID_ALT_STAB].kP(2);
  pids[PID_ALT_STAB].kI(0);
  pids[PID_ALT_STAB].imax(50);
  
  pids[PID_ALT_RATE].kP(0.62);
  pids[PID_ALT_RATE].kI(1);
  pids[PID_ALT_RATE].imax(50);
  
  pids[PID_LAT_STAB].kP(1);
  pids[PID_LON_STAB].kP(1);
  pids[PID_LAT_STAB].kD(0.1);
  pids[PID_LON_STAB].kD(0.1);
  pids[PID_LAT_STAB].imax(25);
  pids[PID_LON_STAB].imax(25);
  pids[PID_LAT_STAB].kI(0);
  pids[PID_LON_STAB].kI(0);
  
  pids[PID_LAT_RATE].kP(1);
  pids[PID_LON_RATE].kP(1);
  pids[PID_LAT_RATE].kD(0);
  pids[PID_LON_RATE].kD(0);
  pids[PID_LAT_RATE].imax(25);
  pids[PID_LON_RATE].imax(25);
  pids[PID_LAT_RATE].kI(0.1);
  pids[PID_LON_RATE].kI(0.1);
  
  
  
 #ifdef APM2_HARDWARE
    hal.gpio->pinMode(40, GPIO_OUTPUT);
    hal.gpio->write(40, HIGH);
#endif

    ins.init(AP_InertialSensor::COLD_START, 
			 AP_InertialSensor::RATE_100HZ,
			 flash_leds);
    ins.init_accel(flash_leds);

    compass.set_orientation(MAG_ORIENTATION);
    ahrs.init();
    hal.scheduler->delay(1000);

    if(compass.init() ) {
        hal.console->printf("Enabling compass\n");
        ahrs.set_compass(&compass);
    } else {
        hal.console->printf("No compass detected\n");
    }
    
    
    g_gps = &GPS;

    g_gps->init(hal.uartB,GPS::GPS_ENGINE_AIRBORNE_2G);
    hal.console->print("GPS");

  last_update = hal.scheduler->millis();
  
  hal.console->println("Initialising barometer...");

    hal.scheduler->delay(1000);

    if (!bmp085.init()) {
        hal.console->println("Barometer initialisation FAILED\n");
    }
    hal.console->println("initialisation complete."); 
    
  hal.scheduler->delay(500);
  bmp085.calibrate();
  hal.scheduler->delay(1000);
  inertialnav.init();

  while((g_gps->fix)!=3){
  g_gps->update();
  hal.console->println(g_gps->fix);
  }
   for(int i =1 ;i < 1000 ;i++){
     g_gps->update();
      home_lat = g_gps->latitude;
      home_lon = g_gps->longitude;
      hal.scheduler->delay(10);
    }

    g_gps->update();
  
  inertialnav.set_velocity_xy(0,0);
  inertialnav.set_current_position(home_lon,home_lat);
  hal.console->print("home: ");
  hal.console->print(home_lat);
  hal.console->print(" ");
  hal.console->println(home_lon);
  
  
  hal.rcout->set_freq(0xF, 490);
  hal.rcout->enable_mask(0xFF);
  hal.scheduler->delay(5000);
 

}


char buf[255];
int buf_offset = 0;

void loop() 
{

    uint32_t currtime = hal.scheduler->millis();
    float dt = (currtime - last_update) / 1000.0f;
    last_update = currtime;
    
    g_gps->update();
    if(g_gps->fix){
    lon = g_gps->longitude;
    lat = g_gps->latitude; 
    }

    
    inertialnav.correct_with_gps(lon,lat,dt);
    inertialnav.update(dt);
    float yaw_target =0;
    float tmp_float;
    static uint32_t last_print;

    // accumulate values at 100Hz
    if ((hal.scheduler->micros()- timer) > 20000L) {
	    bmp085.accumulate();
	    timer = hal.scheduler->micros();
    }

    // print at 100Hz
    if ((hal.scheduler->millis()- last_print) >= 100) {
	uint32_t start = hal.scheduler->micros();
        last_print = hal.scheduler->millis();
        bmp085.read();
        uint32_t read_time = hal.scheduler->micros() - start;
        if (! bmp085.healthy) {
            hal.console->println("not healthy");
            return;
        }
    }
   
float z_target = 200; 

static uint16_t counter;
    static uint32_t last_t,  last_compass;
    uint32_t now = hal.scheduler->micros();
    float heading = 0;

    if (last_t == 0) {
        last_t = now;
        return;
    }
    last_t = now;
        compass.read() ;
        heading = compass.calculate_heading(ahrs.get_dcm_matrix());
        last_compass = now;
    ahrs.update();
    counter++;
    Vector3f gyro;
    Vector3f accel;
    Vector3f velocity;
    Vector3f positions;
    gyro = ins.get_gyro();
    accel = ins.get_accel();
    velocity = inertialnav.get_velocity();
    positions = inertialnav.get_position();
    velz = velocity.z;
    altf = inertialnav.get_altitude();
    roll      = ToDeg(ahrs.roll);
    pitch     = ToDeg(ahrs.pitch);
    yaw       = ToDeg(heading);
    gyroRoll  = degrees(gyro.x);
    gyroPitch = degrees(gyro.y);
    gyroYaw   = degrees(gyro.z);
   
  uint16_t channels[8];
  static uint32_t lastPkt = 0;
  static int32_t radio[6] = {0,0,0,0,0,0};
  
// serial bytes available?

  int bytesAvail = hal.uartC->available();
  
  if(bytesAvail > 0) {
   // hal.console->println("1");
    while(bytesAvail > 0) {  //yes
        
        char c = (char)hal.uartC->read();//read next byte
//        hal.console->print("in");
//        hal.console->print(" ");
//        hal.console->println(c);
        if(c =='\n'){
        buf[buf_offset] = '\0';
        
        char *str = strtok(buf,"*");
        char *ch = strtok(str,",");
        radio[0] = strtol(ch, NULL, 10);
        ch = strtok(NULL,",");
        radio[1] = strtol(ch, NULL, 10);
        buf_offset = 0;
     //   hal.console->print(radio[0]);
     //   hal.console->print(" ");
     //   hal.console->println(radio[1]);
      //  float x= strtof(ch);
  } 
     else if(c != '\r') {
         buf[buf_offset++] = c;   // store in buffer and continue until newline
       }
       bytesAvail --;
    }
  } 
  
  float mob_lat = radio[0];
  float mob_lon = radio[1];
  // Read RC transmitter and map to sensible values  
  hal.rcin->read(channels, 8);
  
  float rcthr, rcyaw, rcpit, rcroll,tune1,tune2;  // Variables to store radio in
  rcthr = channels[2];
  rcyaw = map(channels[3], RC_YAW_MIN, RC_YAW_MAX, -180, 180);
  rcpit = map(channels[1], RC_PIT_MIN, RC_PIT_MAX, -20, 20);
  rcroll = map(channels[0], RC_ROL_MIN, RC_ROL_MAX, -20, 20);
  
  tune1 = map(channels[4], 994, 2000, 0, 1000);
  tune2 = map(channels[5], 994, 2000, 0, 1000);
  

  float lat_f=0, lon_f=0;
  if(tune1>500){
  lat_f=mob_lat;
  }
  else lat_f=home_lat;
  
  if(tune2>500){
    lon_f = mob_lat;
  }
  else lon_f=home_lon;
   
  float  dlat = lat_f - lat;
  float  dlon = lon_f - lon;
    
    dlat = round(dlat/100);
    dlon = round(dlon/100);
    
    
float corr = 1 * (abs(sin(ToRad(roll))) + abs(sin(ToRad(pitch))));
 
 rcthr = rcthr + corr;
 
  pids[PID_LAT_STAB].kP(2.31);
  pids[PID_LON_STAB].kP(2.31);
//  float dx= inertialnav.get_latitude_diff();
//  float dy= inertialnav.get_longitude_diff();
//  
//  dx=dx/100;
//  dy=dy/100;
  
    float lat_corr = -1*constrain(pids[PID_LAT_STAB].get_pid(dlat,1),-10,10);
    float lon_corr = constrain(pids[PID_LON_STAB].get_pid(dlon,1),-10,10);
    
//    velx = velocity.x;
//    vely = velocity.y;
//    
//    lat_corr = constrain(pids[PID_LAT_RATE].get_pid(lat_corr-velx,1),-10,10);
//    lon_corr = constrain(pids[PID_LON_RATE].get_pid(lon_corr-vely,1),-10,10);
//    
  // Do the magic
  if(rcthr > RC_THR_MIN + 150) {  // Throttle raised, turn on stablisation.
  // Stablise PIDS    
    float pitch_stab_output = constrain(pids[PID_PITCH_STAB].get_pid((lat_corr + rcpit ) - pitch, 1), -250, 250); //     0 and 1 are the direct or reverse logic of controller # signs
    float roll_stab_output = constrain(pids[PID_ROLL_STAB].get_pid((lon_corr + rcroll ) - roll, 1), -250, 250);
    float yaw_stab_output = constrain(pids[PID_YAW_STAB].get_pid(wrap_180(yaw_target - yaw), 1), -360, 360);
    float alt_stab_output = constrain(pids[PID_ALT_STAB].get_pid((z_target - altf), 1), -15, 15);
    
    // is pilot asking for yaw change - if so feed directly to rate pid (overwriting yaw stab output)
    if(abs(rcyaw ) > 15) {
      yaw_stab_output = rcyaw;
      yaw_target = yaw;   // remember this yaw for when pilot stops
    }
    
    // rate PIDS
    float pitch_output =   constrain(pids[PID_PITCH_RATE].get_pid(pitch_stab_output - gyroPitch, 1), - 500, 500);  
    float roll_output =   constrain(pids[PID_ROLL_RATE].get_pid(roll_stab_output - gyroRoll, 1), -500, 500);  
    float yaw_output =   constrain(pids[PID_YAW_RATE].get_pid(yaw_stab_output - gyroYaw, 1), -500, 500); 
    float alt_output =   constrain(pids[PID_ALT_RATE].get_pid(alt_stab_output - velz, 1), -100, 100); 
    
  // mix pid outputs and send to the motors.
    //alt__output = 0;
    hal.rcout->write(MOTOR_F, rcthr + alt_output + pitch_output - yaw_output);
    hal.rcout->write(MOTOR_L, rcthr + alt_output + roll_output  + yaw_output);
    hal.rcout->write(MOTOR_R, rcthr + alt_output - roll_output +  yaw_output);
    hal.rcout->write(MOTOR_B, rcthr + alt_output - pitch_output - yaw_output);
    
 } else {
    // motors off
    
    hal.rcout->write(MOTOR_F, 900);
    hal.rcout->write(MOTOR_L, 900);
    hal.rcout->write(MOTOR_R, 900);
    hal.rcout->write(MOTOR_B, 900);
       
    // reset yaw target so we maintain this on takeoff
    yaw_target = yaw;
    
    // reset PID integrals whilst on the ground
    for(int i=0; i<10; i++)
      pids[i].reset_I();
 }
}

AP_HAL_MAIN();
