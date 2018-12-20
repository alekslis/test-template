#include <khepera/khepera.h>
#include <signal.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
//testgita
static knet_dev_t * dsPic; // robot pic microcontroller access
int maxsp,accinc,accdiv,minspacc, minspdec; // for speed profile
static int quitReq = 0; // quit variable for loop
/*--------------------------------------------------------------------*/
/*!
 * Make sure the program terminate properly on a ctrl-c
 */
static void ctrlc_handler( int sig )
{
  quitReq = 1;
  kh4_set_speed(0 ,0 ,dsPic); // stop robot
  kh4_SetMode( kh4RegIdle,dsPic );
  kh4_SetRGBLeds(0,0,0,0,0,0,0,0,0,dsPic); // clear rgb leds because consumes energy
  kb_change_term_mode(0); // revert to original terminal if called
  exit(0);
}
#define fwSpeed 150//150
#define BMIN_SPEED 10
#define MAX_DIST 500
#define MIN_DIST 80 // 70
#define PI 3.14159265358979
#define M_PI 3.14159265358979323846
#define deg_PI 180.0
#define PLIK_XY "pozycjaxy.txt"
#define PLIK_US "us_readings.txt"
int sl, sr; //predkosci lewego i prawego
//zmienne do odometrii
int pos_left_prev;
int pos_right_prev;
float result_x;
float result_y;
float result_theta;
int pos_left,pos_right;
float wheel_distance;
float wheel_conversion_left;
float wheel_conversion_right;
int acceleration_step;
float speed_max;
float speed_min;
float gotox,gotoy;
int speed_left_internal;
int speed_right_internal;
int r_speed_left;
int r_speed_right;
int closetogoal;
int veryclosetogoal;
int atgoal;

float cel_kat;
void odometria_init()
{
	//Odometria - domyslne dane
	wheel_distance = 0.10470;
	wheel_conversion_left = 0.13194 / 19456.0; // perimeter over pulses per rev
	wheel_conversion_right = 0.13194 / 19456.0;
    acceleration_step = 2.;
    speed_min = 10.;
    speed_max = 90.;
    gotox = 0.;
    gotoy = 0.;
    speed_left_internal = 0;
    speed_right_internal = 0;
    r_speed_left = 0;
    r_speed_right = 0;
    atgoal=0;
    veryclosetogoal=0;
    closetogoal=0;
}

/*void odo_start(float gotoxx,float gotoyy)
{
    gotox=gotoxx;
    gotoyy
}*/
void odometria()
{
	long delta_pos_left, delta_pos_right;
	float delta_left, delta_right, delta_theta, theta2;
	float delta_x, delta_y;

		kh4_get_position(&pos_left,&pos_right,dsPic);
		delta_pos_left = pos_left - pos_left_prev;
		delta_pos_right = pos_right - pos_right_prev;
		delta_left = delta_pos_left * wheel_conversion_left;
		delta_right = delta_pos_right * wheel_conversion_right;
		delta_theta = (delta_left - delta_right) / wheel_distance;
		theta2 = result_theta + delta_theta * 0.5;
		delta_x = (delta_left + delta_right) * 0.5 * cosf(theta2);
		delta_y = (delta_left + delta_right) * 0.5 * sinf(theta2);
		result_x += delta_x;
		result_y += delta_y;
		result_theta += delta_theta;
		//timestamp = khepera4_current_time();
		pos_left_prev = pos_left;
		pos_right_prev = pos_right;
}
/*float skret(float goalangle)
{
    float alpha;
    //goalangle = 1.571
    //int poz_l,poz_r;
    //float deg_res_th_f=floorf(deg_res_th);
    //float deg_goalangle_f=floorf(deg_goalangle);
    float deg_alpha=0;
    //float deg_alpha_f;

    //deg_alpha=alpha * 180.0 / PI;
    float deg_res_th=result_theta * 180.0 / PI;
    float deg_goalangle=goalangle * 180.0 / PI;
    //while((goalangle-result_theta)<-0.1 || (goalangle-result_theta)>0.1){
    //    kb_clrscr();
        alpha = goalangle - result_theta;
        printf("goalangle: %f\nresult_theta: %f\n",goalangle,result_theta);
	    while (alpha > PI)
	    {
	    	alpha -= 2 * PI;
	    }
	    while (alpha < -PI)
	    {
	        alpha += 2 * PI;
	    }
        //printf("Alfa %f\n",alpha);

        //printf("Deg_alfa %f\n",deg_alpha);
        //deg_alpha_f=floorf(deg_alpha);
        //printf("Deg_alfa %f\n",deg_alpha_f);
        cel_kat=alpha;

	       if (result_theta<goalangle)
           {
               kh4_get_position(&poz_l,&poz_r,dsPic);
               kh4_SetMode(kh4RegPosition,dsPic );
               kh4_set_position(poz_l+9456,poz_r-9456,dsPic);
               //kh4_set_speed(50,-50,dsPic); //prawo
               kh4_SetRGBLeds(8,0,0,0,0,0,0,0,0,dsPic);
               usleep(200000);
               kh4_SetMode(kh4RegSpeedProfile,dsPic);
               //kh4_set_speed(0,0,dsPic);
               //sleep(2);

           }
           else if (result_theta>goalangle)
           {

               kh4_get_position(&poz_l,&poz_r,dsPic);
               kh4_SetMode(kh4RegPosition,dsPic );
               kh4_set_position(poz_l-9456,poz_r+9456,dsPic);
               //kh4_set_speed(-50,50,dsPic); //lewo
               kh4_SetRGBLeds(0,0,0,8,0,0,0,0,0,dsPic);
                usleep(200000);
                kh4_SetMode(kh4RegSpeedProfile,dsPic);
               //kh4_set_speed(0,0,dsPic);
               //sleep(2);
           }
           else
           {

               return;
           }
   // }
return 0;
}*/
void odometry_goto(float goal_x,float goal_y)
{
	       //STEP
	       float dx, dy;
	       float distance, goalangle, alpha;
	       float speedfactor;
	       long speed_left_wish, speed_right_wish;
	       int atmaxspeed = 0;
	       // Do nothing if we are at goal
	       if (atgoal != 0)
	       {
	    	   kh4_SetRGBLeds(0,5,0,0,5,0,0,5,0,dsPic);
	    	   printf("U CELU \n");
	    	   return;
	       }
	       // Calculate new wish speeds
	       dx = goal_x - result_x;
	       dy = goal_y - result_y;
	       distance = sqrt(dx * dx + dy * dy);
	       goalangle = atan2(dy, dx);
	       alpha = goalangle - result_theta;
	       while (alpha > PI)
	       {
	    	   alpha -= 2 * PI;
	       }
	       while (alpha < -PI)
	       {
	           alpha += 2 * PI;
	       }

	       // Calculate the speed factor
	       speedfactor = (distance + 0.05) * 10. * speed_max;
	       if (speedfactor > speed_max)
	       {
	           speedfactor = speed_max;
	           atmaxspeed = 1;
	       }
	        // Calculate the theoretical speed
	        //printf("dist %f - goalangle %f - alpha %f \n", distance, goalangle, alpha);
	        speed_left_wish = speedfactor * (PI - 2 * abs(alpha) + alpha) / PI + 0.5;
	        speed_right_wish = speedfactor * (PI - 2 * abs(alpha) - alpha) / PI + 0.5;

	        // Close to termination condition: just stop
	        if (veryclosetogoal)
	        {
	        	speed_left_wish = 0;
	            speed_right_wish = 0;
	        }

	        // Limit acceleration
	        if (speed_left_wish > speed_left_internal)
	        {
	        	speed_left_internal += acceleration_step;
	        }
	        if (speed_left_wish < speed_left_internal)
	        {
	            speed_left_internal -= acceleration_step;
	        }
	        if (speed_right_wish > speed_right_internal)
	        {
	            speed_right_internal += acceleration_step;
	        }
	        if (speed_right_wish < speed_right_internal)
	        {
	            speed_right_internal -= acceleration_step;
	        }

	        // Don't set speeds < MIN_SPEED (for accuracy reasons)
	        r_speed_left = speed_left_internal;
	        r_speed_right = speed_right_internal;
	        if (abs(r_speed_left) < speed_min)
	        {
	        	r_speed_left = 0;
	        }
	        if (abs(r_speed_right) < speed_min)
	        {
	            r_speed_right = 0;
	        }
	        // Termination condition
	        if (atmaxspeed == 0)
	        {
	            closetogoal = 1;
	            if ((r_speed_left == 0) || (r_speed_right == 0))
	            {
	            	veryclosetogoal = 1;
	            	   //atgoal = 1;
	            }
	            if ((r_speed_left == 0) && (r_speed_right == 0))
	            {
	            	atgoal = 1;
	            }
	        }
}

void run_goto_heading(float goal_theta) {
    float diff_theta;

    // Move until we have reached the target position
    while (1) {
        // Update position and calculate new speeds
        odometria();

        // Calculate the current heading error
        diff_theta = goal_theta - result_theta;
        while (diff_theta > PI) {
            diff_theta -= 2 * PI;
        }
        while (diff_theta < -PI) {
            diff_theta += 2 * PI;
        }

        // Termination condition
        if (fabs(diff_theta) < 0.01) {
            break;
        }

        if (diff_theta>0)
        {
            kh4_set_speed(50,-50,dsPic);
            kh4_SetRGBLeds(0,0,0,0,5,0,0,0,0,dsPic);
        }
        else if (diff_theta<0)
        {
            kh4_set_speed(-50,50,dsPic);
            kh4_SetRGBLeds(0,5,0,0,0,0,0,0,0,dsPic);
        }
        //khepera4_drive_set_speed_differential_bounded(og.configuration.speed_max, 0, 0, diff_theta * 8., 1);

    }
    kh4_SetRGBLeds(0,5,0,0,5,0,0,5,0,dsPic);
    kh4_set_speed(0,0,dsPic);
    sleep(2);
    return;
    // Stop the motors
    //khepera4_drive_set_speed(0, 0);

}



int test()
{

    kh4_activate_us(31,dsPic); //wl. ultradzwiekowe
    int sl, sr;
    int i;
    char Buffer[256];
    short usvalues[5];
    int sensors[12];
    FILE *xytxt;
	xytxt = fopen(PLIK_XY,"w");
    FILE *ustxt;
	ustxt = fopen(PLIK_US,"w");
	accinc=10;//3;
	accdiv=0;
	minspacc=20;
	minspdec=1;
	maxsp=400;
    int ii,jj;
    int mapka[2][3];
    int max_us=0;
    int maxi;
    int poz_l,poz_r;
	kh4_SetSpeedProfile(accinc,accdiv,minspacc, minspdec,maxsp,dsPic ); // Acceleration increment ,  Acceleration divider, Minimum speed acc, Minimum speed dec, maximum speed
	kh4_SetMode(kh4RegSpeedProfile,dsPic);
	//inicjalizacja odometrii
	kh4_get_position(&pos_left,&pos_right,dsPic);

	result_x = 0;
	result_y = 0;
	result_theta = 0;
	//timestamp = 0;

	pos_left_prev = pos_left;
	pos_right_prev = pos_right;
	//kh4_set_speed(sl, sr, dsPic);
    odometria_init();
    printf("Predkosci kol\n");
    printf("Lewe:");
    scanf("%d", &sl);
    printf("Prawe:");
    scanf("%d", &sr);

    for(ii=0;ii<2;ii++)
	{
		for(jj=0;jj<3;jj++)
		{
			mapka[ii][jj]=0;
		}
	}
	for(ii=0;ii<2;ii++)
	{
		for(jj=0;jj<3;jj++)
		{
			printf("%d  ",mapka[ii][jj]);
		}
		printf("\n");
	}
    /*
    kh4_measure_us(Buffer,dsPic);
    for (i=0;i<5;i++)
        {
        	usvalues[i] = (short)(Buffer[i*2] | Buffer[i*2+1]<<8);
            if (usvalues[i]>max_us)
            {
                max_us=usvalues[i];
                maxi=i;
            }
        }
        /*
        kh4_SetMode(kh4RegPosition,dsPic);


	    if (maxi==0){
            kh4_set_position(poz_l-10000,poz_r+10000,dsPic);
            sleep(2);
        }
        else if(maxi==1){
            kh4_set_position(poz_l-5000,poz_r+5000,dsPic);
            sleep(2);
        }
        else if(maxi==2){
            kh4_set_position(5000,5000,dsPic);
            sleep(2);
        }
        else if(maxi==3){
            kh4_set_position(poz_l+5000,poz_r-5000,dsPic);
            sleep(2);
        }
        else if(maxi==4){
            kh4_set_position(poz_l+10000,poz_r-10000,dsPic);
            sleep(2);
        }
        kh4_SetMode(kh4RegSpeedProfile,dsPic);
        */

	while(!kb_kbhit())
    {

        kb_clrscr();
        //kh4_get_position(&poz_l,&poz_r,dsPic);

        kh4_proximity_ir(Buffer, dsPic);
        for (i=0;i<12;i++)
		{
			sensors[i]=(Buffer[i*2] | Buffer[i*2+1]<<8);
        }
        kh4_measure_us(Buffer,dsPic);
        for (i=0;i<5;i++)
        {
        	usvalues[i] = (short)(Buffer[i*2] | Buffer[i*2+1]<<8);
            /*if (usvalues[i]>max_us)
            {
                max_us=usvalues[i];
                maxi=i;
            }*/
        }

            //float gotox,gotoy;


    odometria();
    if (usvalues[2]>30 && usvalues[2]<1000)
    {
        //gotox=result_x+0.1;
        //gotoy=result_y;
        //usleep(200000);
        //kh4_SetMode(kh4RegSpeedProfile,dsPic);
        kh4_set_speed(sl,sr,dsPic);
        kh4_SetRGBLeds(0,0,7,0,0,7,0,5,0,dsPic);

    }
    else if((usvalues[0]>25 && usvalues[0]<1000) || (usvalues[1]>25 && usvalues[1]<1000) || sensors [3] > 150)
    {
        //kh4_SetMode(kh4RegPosition,dsPic);
        //kh4_set_position(poz_l-10000,poz_r+10000,dsPic);
        //usleep(200000);
        //kh4_SetMode(kh4RegSpeedProfile,dsPic);
        //gotox=result_x;
        //gotoy=result_y-0.1;
        run_goto_heading(-1.57);
    }
    else if(usvalues[4]>30 && usvalues[0]<1000 || sensors [3] > 150)
    {
        //kh4_SetMode(kh4RegPosition,dsPic);
        //kh4_set_position(poz_l+10000,poz_r-10000,dsPic);
        //usleep(200000);
        //kh4_SetMode(kh4RegSpeedProfile,dsPic);
        //gotox=result_x;
        //gotoy=result_y+0.1;

        run_goto_heading(1.57);
    }
    kh4_set_speed(sl,sr,dsPic);
        //while(atgoal==0)
        //{
        //odometria();
        //odometry_goto(gotox,gotoy);
        //kh4_set_speed(r_speed_left,r_speed_right,dsPic);
        printf("Czujniki us\nl90: %4d\nfl45: %4d\nf0: %4d\nfr45: %4d\nr90: %4d\n", usvalues[0], usvalues[1], usvalues[2], usvalues[3], usvalues[4]);
        fprintf(ustxt,"%d %d %d %d %d\n",usvalues[0],usvalues[1],usvalues[2],usvalues[3],usvalues[4]);

		printf("Aktualna pozycja robota: x: %.3f  y: %.3f  kat: %.4f\n",result_x, result_y, result_theta);
		fprintf(xytxt,"%.4f %.4f %.4f\n", result_x, result_y, result_theta);
        usleep(20000);
       // }
       // odometria_init();
        /*
        for(i=0;i<5;i++)
        {
        	if (i==0) { ii=1; jj=0;}
        	else if (i==1) {ii=0; jj=0;}
        	else if (i==2) {ii=0; jj=1;}
        	else if (i==3) {ii=0; jj=2;}
        	else if (i==4) {ii=1; jj=2;}

        	if (usvalues[i]>25)
        	{
        		mapka[ii][jj]=usvalues[i];
        	}
        	else{
        		mapka[ii][jj]=0;
        	}
        }
        for(ii=0;ii<2;ii++)
        {
        	for(jj=0;jj<3;jj++)
        	{
        		printf("%d    ",mapka[ii][jj]);
        	}
        	printf("\n");
        }
        */

        //printf("Alfa %f\n",cel_kat);
        float kat_skret_p=1.571;
        float kat_skret_l=-1.571;
        int poz_l,poz_r;
        /*
        if (sensors[2] > 150 || sensors[1]>150) //przeszkoda lewo i przod
        	        {

        	        //skret(kat_skret_p);

               kh4_get_position(&poz_l,&poz_r,dsPic);
               kh4_SetMode(kh4RegPosition,dsPic );
               kh4_set_position(poz_l+9456,poz_r-9456,dsPic);
               //kh4_set_speed(50,-50,dsPic); //prawo
               kh4_SetRGBLeds(8,0,0,0,0,0,0,0,0,dsPic);
               usleep(200000);
               kh4_SetMode(kh4RegSpeedProfile,dsPic);
               //kh4_set_speed(0,0,dsPic);
               //sleep(2);
        	        	//kh4_set_speed(100,-100, dsPic);
        	        	//kh4_SetRGBLeds(1,0,0,0,0,0,0,0,0,dsPic);
        	        }
        else if (sensors[4] > 150 || sensors[5]>150) //przeszkoda prawo i przod
        	        {

        	        //skret(kat_skret_l);

        	   kh4_get_position(&poz_l,&poz_r,dsPic);
               kh4_SetMode(kh4RegPosition,dsPic );
               kh4_set_position(poz_l-9456,poz_r+9456,dsPic);
               //kh4_set_speed(-50,50,dsPic); //lewo
               kh4_SetRGBLeds(0,0,0,8,0,0,0,0,0,dsPic);
                usleep(200000);
                kh4_SetMode(kh4RegSpeedProfile,dsPic);
        	        }
        else
        {
            kh4_set_speed(sl,sr,dsPic);
        }
        */
        printf("\nNacisnij klawisz aby zatrzymac\n");
        usleep(100);
    }
    //tcflush(0, TCIFLUSH); // flush input
    kh4_set_speed(0,0,dsPic ); // stop robot
    kh4_SetMode( kh4RegIdle,dsPic ); // set motors to idle
    accinc=3;//3;
    accdiv=0;
    minspacc=20;
    minspdec=1;
    maxsp=400;
    // configure acceleration slope
    kh4_SetSpeedProfile(accinc,accdiv,minspacc, minspdec,maxsp,dsPic ); // Acceleration increment ,  Acceleration divider, Minimum speed acc, Minimum speed dec, maximum speed
    fclose(xytxt);
    fclose(ustxt);
    kh4_activate_us(0,dsPic); //wyl. ultradzwiekowe
    return 0;
}

int main(int argc, char * argv[])
{

#define MAX_G 2 		// max acceleration in g
  char Bufer[100];
  char revision,version;
  int kp,ki,kd;
  int pmarg;
// initiate libkhepera and robot access
 if ( kh4_init(argc ,argv)!=0)
 {
	 printf("\nERROR: could not initiate the libkhepera!\n\n");
	 return -1;
 }
/* open robot socket and store the handle in their respective pointers */
 dsPic  = knet_open( "Khepera4:dsPic" , KNET_BUS_I2C , 0 , NULL );
 if ( dsPic==NULL)
  {
  	printf("\nERROR: could not initiate communication with Kh4 dsPic\n\n");
  	return -2;
  }
  /* initialize the motors controlers*/
  /* tuned parameters */
  pmarg=20;
  kh4_SetPositionMargin(pmarg,dsPic ); 				// position control margin
  kp=10;
  ki=5;
  kd=1;
  kh4_ConfigurePID( kp , ki , kd,dsPic  ); 		// configure P,I,D
  accinc=3;//3;
  accdiv=0;
  minspacc=20;
  minspdec=1;
  maxsp=400;
  // configure acceleration slope
  kh4_SetSpeedProfile(accinc,accdiv,minspacc, minspdec,maxsp,dsPic ); // Acceleration increment ,  Acceleration divider, Minimum speed acc, Minimum speed dec, maximum speed
  kh4_ResetEncoders(dsPic);
  kh4_SetMode( kh4RegIdle,dsPic );  // Put in idle mode (no control)

  // get revision
  if(kh4_revision(Bufer, dsPic)==0){
   	version=(Bufer[0]>>4) +'A';
  	revision=Bufer[0] & 0x0F;
    printf("\r\nVersion = %c, Revision = %u\r\n",version,revision);
  }

  signal( SIGINT , ctrlc_handler ); // set signal for catching ctrl-c
  /* For ever loop until ctrl-c key */
  while(quitReq==0)
  {
      test(); //funkcja glowna(?)
    //tcflush(0, TCIFLUSH); //?????
    break;
  }
//zakończenie działania
  kh4_set_speed(0 ,0 ,dsPic); // stop robot
  kh4_SetMode( kh4RegIdle,dsPic ); // set motors to idle
  kh4_SetRGBLeds(0,0,0,0,0,0,0,0,0,dsPic); // clear rgb leds because consumes energy

  return 0;
}
