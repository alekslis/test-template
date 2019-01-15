#include <khepera/khepera.h>
#include <signal.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
//chybasieudalo
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
#define fwSpeed 100//150
#define BMIN_SPEED 10
#define MAX_DIST 500
#define MIN_DIST 80 // 70
#define PI 3.14159265358979
#define R_45 0.785398 //radiany 45 st.
#define R_90 1.570796 //radiany 90 st.
#define F_01 14745 //0.1 m w przód
#define DEG_PI 180.0
#define PLIK_XY "pozycjaxy.txt"
#define PLIK_US "data.txt"
#define MAX_US_DIST 250 //maksymalny zasieg US
#define MAX_US_DIST2 100//ZASIEG 1 M
#define MAX_US_INT 10
#define MIN_US_DIST 25 //minimalny zasieg US
#define A_US_0 -1.570796
#define A_US_1 -0.785398
#define A_US_2 0
#define A_US_3 0.785398
#define A_US_4 1.570796
#define IR_OBST_TRSH 250
#define SIZE 10000
#define HEADING_0 -2.356194
#define HEADING_1 -1.570796
#define HEADING_2 -0.785398
#define HEADING_3 0
#define HEADING_4 0.785398
#define HEADING_5 1.570796
#define HEADING_6 2.356194
#define HEADING_7 3.141593
#define CELL_0 -100-1
#define CELL_1 -1
#define CELL_2 100-1
#define CELL_3 100
#define CELL_4 100+1
#define CELL_5 1
#define CELL_6 -100+1
#define CELL_7 -100
int hasz_key;
int current_angle;
float wish_angle;
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
float golx,goly;
float r_theta;
float cel_kat;
float del_theta;
int atheading;
char Buffer[256];
short usvalues[5];
int sensors[12];
float skret;
float maxus;
int map[22][22];
int cur_x,cur_y;
float avoid_past_theta;
float dist,goaltheta;
float correct_angle;
float check_space_angle;
int ava_tab[100][100];
int mapx[8];
int mapy[8];
FILE *xytxt;
FILE *datatxt;
struct DataItem {
   int data;   
   int key;
};

struct DataItem* hashArray[SIZE]; 
struct DataItem* dummyItem;
struct DataItem* item;

/**********************************************************************************************
 *********************************************************************************************/
void create_map()
 {
   int j,i;
   
    for (j=0;j<22;j++)
    {
        for(i=0;i<22;i++)
        {
            map[j][i]=0;
        }
    }
    for (j=0;j<22;j++)
    {
        map[j][0]=2;
        map[j][21]=2;
    }
    
    for (i=0;i<22;i++)
    {
        map[0][i]=2;
        map[21][i]=2;
    }
}

/**********************************************************************************************
 * *******************************************************************************************/
void odometria_init()
{
	//Odometria - domyslne dane
	wheel_distance = 0.105;
	wheel_conversion_left = 0.13194 / 19456.0; // perimeter over pulses per rev
	wheel_conversion_right = 0.13194 / 19456.0;
    acceleration_step = 2.;
    speed_min = 10.;
    speed_max = 150.;
    gotox = 0.;
    gotoy = 0.;
    speed_left_internal = 0;
    speed_right_internal = 0;
    r_speed_left = 0;
    r_speed_right = 0;
    result_x = 0.1;
    result_y = 0.1;
    result_theta = 0;
    //timestamp = 0;

    pos_left_prev = pos_left;
    pos_right_prev = pos_right;
    atgoal=0;
    veryclosetogoal=0;
    closetogoal=0;
    r_theta=0;
    del_theta=0;
    atheading=0;
}
/**********************************************************************************************
 * *******************************************************************************************/
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
    while (result_theta > PI) {
        result_theta -= 2 * PI;
    }
    while (result_theta < -PI) {
        result_theta += 2 * PI;
    }
		//timestamp = khepera4_current_time();
		pos_left_prev = pos_left;
		pos_right_prev = pos_right;
}
/**********************************************************************************************
 *********************************************************************************************/
void check_curr_point()
{
    float res_x;
    float res_y;
    res_x=result_x*10.0;
    res_y=result_y*10.0;
    res_x=(float)round(res_x);
    res_y=(float)round(res_y);
    int ress_x=res_x;
    int ress_y=res_y;
    cur_x=ress_x;
    cur_y=ress_y;
    mapx[0]=cur_x-1;
    mapy[0]=cur_y-1;
    mapx[1]=cur_x;
    mapy[1]=cur_y-1;
    mapx[2]=cur_x+1;
    mapy[2]=cur_y-1;
    mapx[3]=cur_x+1;
    mapy[3]=cur_y;
    mapx[4]=cur_x+1;
    mapy[4]=cur_y+1;
    mapx[5]=cur_x;
    mapy[5]=cur_y+1;
    mapx[6]=cur_x-1;
    mapy[6]=cur_y+1;
    mapx[7]=cur_x-1;
    mapy[7]=cur_y;

}
/**********************************************************************************************
 *********************************************************************************************/
void set_current_point()
{
    map[cur_x][cur_y]=1;
    
}
/**********************************************************************************************
 *********************************************************************************************/
void update_map()
{
    int i,j;
    kh4_measure_us(Buffer,dsPic);
        for (i=0;i<5;i++){
        	usvalues[i] = (short)(Buffer[i*2] | Buffer[i*2+1]<<8);
          if(usvalues[i]>MAX_US_DIST2){
            usvalues[i]=0;
          }
          else if(usvalues[i]<MIN_US_DIST){
            usvalues[i]=0;
          }
    }
    usleep(100000);
    double us_value[5];
    int us_values[5];
    for (i=0;i<5;i++){
        us_value[i] = usvalues[i]/10.0;
        us_value[i] = floor(us_value[i]);
        us_values[i] = us_value[i];
    }
    int map0x=cur_x-1;
    int map0y=cur_y-1;
    int usx,usy;
    usx=
    usy=
    map[cur_x+us_values[2]][cur_y]=us_values[2];   //front
    map[cur_x+us_values[1]-1][cur_y-us_values[1]-1]=us_values[1];   //front left
    map[cur_x+us_values[3]-1][cur_y+us_values[3]-1]=us_values[3];   //front right
    map[cur_x][cur_y-us_values[0]]=us_values[0];   //left
    map[cur_x][cur_y+us_values[4]]=us_values[4];   //right/


    if (current_angle==0){
        kier_f=map[cur_x-1][cur_y-1];
        kier_l=map[cur_x-1][cur_y];
        kier_r=map[cur_x][cur_y-1];
        
        if(kier_f==0){
            avoid_past_theta=-2.356194;
        }
        else if (kier_l==0){
            avoid_past_theta=3.141593;
        }
        else if (kier_r==0){
            avoid_past_theta=-1.570796;
        }
        else {avoid_past_theta=0.785398;}
    }
    //kierunek -90
    else if (current_angle==1){
        kier_f=map[cur_x][cur_y-1];
        kier_l=map[cur_x-1][cur_y-1];
        kier_r=map[cur_x+1][cur_y-1];
        
        if(kier_f==0){
            avoid_past_theta=-1.570796;
        }
        else if (kier_l==0){
            avoid_past_theta=-2.356194;
        }
        else if (kier_r==0){
            avoid_past_theta=-0.785398;
        }
        else {avoid_past_theta=1.570796;}
    }
    //kierunek -45
    else if (current_angle==2){
        kier_f=map[cur_x+1][cur_y-1];
        kier_l=map[cur_x][cur_y-1];
        kier_r=map[cur_x+1][cur_y];
        
        if(kier_f==0){
            avoid_past_theta=-0.785398;
        }
        else if (kier_l==0){
            avoid_past_theta=-1.570796;
        }
        else if (kier_r==0){
            avoid_past_theta=0;
        }
        else {avoid_past_theta=2.356194;}
    }
    //kierunek 0
    else if (current_angle==3){
        kier_f=map[cur_x+1][cur_y];
        kier_l=map[cur_x+1][cur_y-1];
        kier_r=map[cur_x+1][cur_y+1];
        
        if(kier_f==0){
            avoid_past_theta=0;
        }
        else if (kier_l==0){
            avoid_past_theta=-0.785398;
        }
        else if (kier_r==0){
            avoid_past_theta=0.785398;
        }
        else {avoid_past_theta=PI;}
    }
    //kierunek 45
    else if (current_angle==4){
        kier_f=map[cur_x+1][cur_y+1];
        kier_l=map[cur_x+1][cur_y];
        kier_r=map[cur_x][cur_y+1];
        
        if(kier_f==0){
            avoid_past_theta=0.785398;
        }
        else if (kier_l==0){
            avoid_past_theta=0;
        }
        else if (kier_r==0){
            avoid_past_theta=1.570796;
        }
        else {avoid_past_theta=PI;}
    }
    //kierunek 90
    else if (current_angle==5){
        kier_f=map[cur_x][cur_y+1];
        kier_l=map[cur_x+1][cur_y+1];
        kier_r=map[cur_x-1][cur_y+1];
        
        if(kier_f==0){
            avoid_past_theta=1.570796;
        }
        else if (kier_l==0){
            avoid_past_theta=0.785398;
        }
        else if (kier_r==0){
            avoid_past_theta=2.356194;
        }
        else {avoid_past_theta=-1.570796;}
    }
    //kierunek 135
    else if (current_angle==6){
        kier_f=map[cur_x-1][cur_y+1];
        kier_l=map[cur_x][cur_y+1];
        kier_r=map[cur_x-1][cur_y];
        
        if(kier_f==0){
            avoid_past_theta=2.356194;
        }
        else if (kier_l==0){
            avoid_past_theta=1.570796;
        }
        else if (kier_r==0){
            avoid_past_theta=PI;
        }
        else {avoid_past_theta=0.785398;}
    }
    //kierunek 180
    else if (current_angle==7){
        kier_f=map[cur_x-1][cur_y];
        kier_l=map[cur_x-1][cur_y+1];
        kier_r=map[cur_x-1][cur_y-1];
        
        if(kier_f==0){
            avoid_past_theta=PI;
        }
        else if (kier_l==0){
            avoid_past_theta=2.356194;
        }
        else if (kier_r==0){
            avoid_past_theta=-2.356194;
        }
        else {avoid_past_theta=0.785398;}
    }
    else {avoid_past_theta=666666;}
}
/**********************************************************************************************
 *********************************************************************************************/
void check_heading()
{   
    odometria();
    //Front
    if (result_theta>-0.3926 && result_theta<0.3926){
        current_angle=3;
        correct_angle=0;
    }
    //Front right
    else if (result_theta>0.3926 && result_theta<1.177998){
        current_angle=4;
        correct_angle=0.785398;
        }
    //Right
    else if (result_theta>1.177998 && result_theta<1.963396){
        current_angle=5;
        correct_angle=1.570796;
        }
    //Bottom right
    else if (result_theta>1.963396 && result_theta<2.748794){
        current_angle=6;
        correct_angle=2.356194;
        }
    //Bottom
    
    else if (result_theta>2.748794 && result_theta<=3.141593){
        current_angle=7;
        correct_angle=3.141593;
        }
    else if (result_theta<-2.748794 && result_theta>=-3.141593){
        current_angle=7;
        correct_angle=3.141593;
        }
    
    //Front left
    else if (result_theta<-0.3926 && result_theta>-1.177998){
        current_angle=2;
        correct_angle=-0.785398;
    }
    //Left
    else if (result_theta<-1.177998 && result_theta>-1.963396){
        current_angle=1;
        correct_angle=-1.570796;
        }
    //Bottom left
    else if (result_theta<-1.963396 && result_theta>-2.748794){
        current_angle=0;
        correct_angle=-2.356194;
        }
    else {current_angle=666;}
 //CORRECT HEADING ?
}
/**********************************************************************************************
 *********************************************************************************************/
void check_past_points()
{   int kier_f,kier_l,kier_r;
    check_heading();
    //kierunek -135
    if (current_angle==0){
        kier_f=map[cur_x-1][cur_y-1];
        kier_l=map[cur_x-1][cur_y];
        kier_r=map[cur_x][cur_y-1];
        
        if(kier_f==0){
            avoid_past_theta=-2.356194;
        }
        else if (kier_l==0){
            avoid_past_theta=3.141593;
        }
        else if (kier_r==0){
            avoid_past_theta=-1.570796;
        }
        else {avoid_past_theta=0.785398;}
    }
    //kierunek -90
    else if (current_angle==1){
        kier_f=map[cur_x][cur_y-1];
        kier_l=map[cur_x-1][cur_y-1];
        kier_r=map[cur_x+1][cur_y-1];
        
        if(kier_f==0){
            avoid_past_theta=-1.570796;
        }
        else if (kier_l==0){
            avoid_past_theta=-2.356194;
        }
        else if (kier_r==0){
            avoid_past_theta=-0.785398;
        }
        else {avoid_past_theta=1.570796;}
    }
    //kierunek -45
    else if (current_angle==2){
        kier_f=map[cur_x+1][cur_y-1];
        kier_l=map[cur_x][cur_y-1];
        kier_r=map[cur_x+1][cur_y];
        
        if(kier_f==0){
            avoid_past_theta=-0.785398;
        }
        else if (kier_l==0){
            avoid_past_theta=-1.570796;
        }
        else if (kier_r==0){
            avoid_past_theta=0;
        }
        else {avoid_past_theta=2.356194;}
    }
    //kierunek 0
    else if (current_angle==3){
        kier_f=map[cur_x+1][cur_y];
        kier_l=map[cur_x+1][cur_y-1];
        kier_r=map[cur_x+1][cur_y+1];
        
        if(kier_f==0){
            avoid_past_theta=0;
        }
        else if (kier_l==0){
            avoid_past_theta=-0.785398;
        }
        else if (kier_r==0){
            avoid_past_theta=0.785398;
        }
        else {avoid_past_theta=PI;}
    }
    //kierunek 45
    else if (current_angle==4){
        kier_f=map[cur_x+1][cur_y+1];
        kier_l=map[cur_x+1][cur_y];
        kier_r=map[cur_x][cur_y+1];
        
        if(kier_f==0){
            avoid_past_theta=0.785398;
        }
        else if (kier_l==0){
            avoid_past_theta=0;
        }
        else if (kier_r==0){
            avoid_past_theta=1.570796;
        }
        else {avoid_past_theta=PI;}
    }
    //kierunek 90
    else if (current_angle==5){
        kier_f=map[cur_x][cur_y+1];
        kier_l=map[cur_x+1][cur_y+1];
        kier_r=map[cur_x-1][cur_y+1];
        
        if(kier_f==0){
            avoid_past_theta=1.570796;
        }
        else if (kier_l==0){
            avoid_past_theta=0.785398;
        }
        else if (kier_r==0){
            avoid_past_theta=2.356194;
        }
        else {avoid_past_theta=-1.570796;}
    }
    //kierunek 135
    else if (current_angle==6){
        kier_f=map[cur_x-1][cur_y+1];
        kier_l=map[cur_x][cur_y+1];
        kier_r=map[cur_x-1][cur_y];
        
        if(kier_f==0){
            avoid_past_theta=2.356194;
        }
        else if (kier_l==0){
            avoid_past_theta=1.570796;
        }
        else if (kier_r==0){
            avoid_past_theta=PI;
        }
        else {avoid_past_theta=0.785398;}
    }
    //kierunek 180
    else if (current_angle==7){
        kier_f=map[cur_x-1][cur_y];
        kier_l=map[cur_x-1][cur_y+1];
        kier_r=map[cur_x-1][cur_y-1];
        
        if(kier_f==0){
            avoid_past_theta=PI;
        }
        else if (kier_l==0){
            avoid_past_theta=2.356194;
        }
        else if (kier_r==0){
            avoid_past_theta=-2.356194;
        }
        else {avoid_past_theta=0.785398;}
    }
    else {avoid_past_theta=666666;}
}
/**********************************************************************************************
 *********************************************************************************************/
void check_space()
{
    int i;
    kh4_measure_us(Buffer,dsPic);
        for (i=0;i<5;i++){
        	usvalues[i] = (short)(Buffer[i*2] | Buffer[i*2+1]<<8);
          if(usvalues[i]>MAX_US_DIST){
            usvalues[i]=0;
          }
          else if(usvalues[i]<MIN_US_DIST){
            usvalues[i]=0;
          }
        }
    //check_heading();
    if (usvalues[2] > 30){check_space_angle=correct_angle;}
    else if (usvalues[1] > 30){check_space_angle=correct_angle-R_45;}
    else if (usvalues[3] > 30){check_space_angle=correct_angle+R_45;}
    else if (usvalues[0] > 30){check_space_angle=correct_angle-R_90;}
    else if (usvalues[4] > 30){check_space_angle=correct_angle+R_90;}
    else {check_space_angle=correct_angle+PI;}
}
void go_front(int pozycja)
{
    kh4_set_speed(0,0,dsPic);
    kh4_get_position(&pos_left,&pos_right,dsPic);
    int poz_docel;
    poz_docel=pos_left+pozycja;
    while(!kb_kbhit())
    {
        kh4_get_position(&pos_left,&pos_right,dsPic);
        if(pos_left>=poz_docel)
        {
           kh4_set_speed (0,0,dsPic);
           break;
        }
        kh4_set_speed (50,50,dsPic);
        usleep(200);
    }
}

/**********************************************************************************************
 *********************************************************************************************/

/**********************************************************************************************
 * *******************************************************************************************/
void odometry_goto(float goal_x,float goal_y)
{
	       //STEP
         atgoal=0;
         veryclosetogoal=0;
         closetogoal=0;
	       float dx, dy;
	       float distance, goalangle, alpha;
	       float speedfactor;
	       long speed_left_wish, speed_right_wish;
	       int atmaxspeed = 0;


        while(!kb_kbhit()){
          kb_clrscr();
          int atmaxspeed = 0;

	       // Do nothing if we are at goal
         kh4_SetRGBLeds(5,5,0,5,5,0,5,5,0,dsPic);
         odometria();
         printf("Aktualna pozycja robota: x: %.3f  y: %.3f  kat: %.4f\n",result_x, result_y, result_theta);
         fprintf(xytxt,"%.4f %.4f %.4f\n", result_x, result_y, result_theta);
	       if (atgoal != 0)
	       {
	    	   kh4_SetRGBLeds(0,5,0,0,5,0,0,5,0,dsPic);
           kh4_set_speed(0,0,dsPic);
	    	   break;
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
              //if (result_x )
	        }
          kh4_set_speed(r_speed_left,r_speed_right,dsPic);

        }
        usleep(1000);
}
/**********************************************************************************************
 * *******************************************************************************************/

void show_goal_param(float goalx,float goaly)
{
    float dx, dy;
	float distance, goalangle, alpha;
    dx = goalx - result_x;
	dy = goaly - result_y;
	distance = sqrt(dx * dx + dy * dy);
	goalangle = atan2(dy, dx);
    dist=distance;
    goaltheta=goalangle;
}
void go_to_dist()
{
    odometria();
    show_goal_param(golx,goly);
    

}
void run_goto_heading(float goal_theta) {
    float diff_theta;
    float res_theta=0;
    atheading=0;

    // Move until we have reached the target position
      while(!kb_kbhit()) {
        //kb_clrscr();
        // Update position and calculate new speeds
        odometria();
        //printf("Aktualna pozycja robota(skret): x: %.3f  y: %.3f  kat: %.6f\n",result_x, result_y, result_theta);
        // Calculate the current heading error
        diff_theta = goal_theta - result_theta;
        while (diff_theta > PI) {
            diff_theta -= 2 * PI;
        }
        while (diff_theta < -PI) {
            diff_theta += 2 * PI;
        }
        del_theta=diff_theta;
        // Termination condition
        /*if (fabs(diff_theta) < 0.01) {
          atheading=1;
        }*/
        if (atheading!=0){
          kh4_set_speed(0,0,dsPic);
          break;
        }
        if (diff_theta>0 && diff_theta>0.01)
        {
            kh4_set_speed(50,-50,dsPic);
            kh4_SetRGBLeds(0,0,0,0,5,0,0,0,0,dsPic);
        }
        else if (diff_theta<0 && diff_theta<-0.01)
        {
            kh4_set_speed(-50,50,dsPic);
            kh4_SetRGBLeds(0,5,0,0,0,0,0,0,0,dsPic);
        }
        else{
          atheading=1;
        }
        //khepera4_drive_set_speed_differential_bounded(og.configuration.speed_max, 0, 0, diff_theta * 8., 1);

    }
    usleep(2000);
    kh4_SetRGBLeds(0,5,0,0,5,0,0,5,0,dsPic);
    kh4_set_speed(0,0,dsPic);

}
/**********************************************************************************************
 * *******************************************************************************************/

/**********************************************************************************************
 * *******************************************************************************************/
 

/**********************************************************************************************
 * *******************************************************************************************/

/**********************************************************************************************
 * *******************************************************************************************/

/**********************************************************************************************/
 void go_str(int poz_f)
{
  int pozycjaf,i;
  kh4_get_position(&pos_left,&pos_right,dsPic);
  //kh4_proximity_ir(Buffer, dsPic);
        //for (i=0;i<12;i++){
			  //     sensors[i]=(Buffer[i*2] | Buffer[i*2+1]<<8);
        //}
  pozycjaf=pos_left+poz_f;
  while(!kb_kbhit()){
        //kb_clrscr();
        if (pozycjaf>pos_left)
        {
          kh4_set_speed(0, 0, dsPic);
          break;
        }
    kh4_get_position(&pos_left,&pos_right,dsPic);
    kh4_SetRGBLeds(0,0,0,0,0,0,8,0,0,dsPic);
    kh4_set_speed(sl, sr, dsPic);
    //kh4_SetMode(kh4RegPosition,dsPic);
   //kh4_set_position(pos_left+14745,pos_right+14745,dsPic);
    //usleep(2000000);
    //kh4_SetMode(kh4RegSpeedProfile,dsPic);
    //kh4_set_speed(0, 0, dsPic);
    //break;
    usleep(1000);
  }
  kh4_set_speed(0, 0, dsPic);
}
/**********************************************************************************************
 * *******************************************************************************************/


/**********************************************************************************************
 * *******************************************************************************************/


/**********************************************************************************************
 * *******************************************************************************************/

/**********************************************************************************************
 * *******************************************************************************************/
int test()
{
    //xytxt = fopen(PLIK_XY,"w");
    datatxt = fopen(PLIK_US,"w");
    kh4_activate_us(31,dsPic); //wl. ultradzwiekowe
    int i,j;
    float f[5];
    int ii,jj;
    int mapka[2][3];
    int poz_l,poz_r;
    int us_heading;
    float kierunek;

	kh4_SetSpeedProfile(accinc,accdiv,minspacc, minspdec,maxsp,dsPic ); // Acceleration increment ,  Acceleration divider, Minimum speed acc, Minimum speed dec, maximum speed
	kh4_SetMode(kh4RegSpeedProfile,dsPic);
	//inicjalizacja odometrii
    printf("Predkosci kol\n");
    printf("Lewe:");
    scanf("%d", &sl);
    printf("Prawe:");
    scanf("%d", &sr);
    kh4_set_speed(0, 0, dsPic);
    kh4_get_position(&pos_left,&pos_right,dsPic);
// inicjalizacja funkcji
    create_map();
    odometria_init();
    odometria();
    check_heading();
    check_curr_point();
    set_current_point();
    map[2][1]=1;

    printf("Orientacja %f %f %f\ncurr p %d %d\n",result_x, result_y, result_theta,cur_x,cur_y);
    

    /*kh4_measure_us(Buffer,dsPic);
        for (i=0;i<5;i++){
        	usvalues[i] = (short)(Buffer[i*2] | Buffer[i*2+1]<<8);
          if(usvalues[i]>MAX_US_DIST){
            usvalues[i]=0;
          }
          else if(usvalues[i]<MIN_US_DIST){
            usvalues[i]=0;
          }
        }*/
	while(!kb_kbhit()){
    kb_clrscr();
    printf("\nNacisnij klawisz aby zatrzymac\n");
    printf("**********************************\n");
//POMIARY
   /* kh4_proximity_ir(Buffer, dsPic);
        for (i=0;i<12;i++){
			       sensors[i]=(Buffer[i*2] | Buffer[i*2+1]<<8);
        }
    kh4_measure_us(Buffer,dsPic);
        for (i=0;i<5;i++){
        	usvalues[i] = (short)(Buffer[i*2] | Buffer[i*2+1]<<8);
          if(usvalues[i]>MAX_US_DIST){
            usvalues[i]=0;
          }
          else if(usvalues[i]<MIN_US_DIST){
            usvalues[i]=0;
          }
        }*/
    kh4_get_position(&pos_left,&pos_right,dsPic);    
//FUNCKJE
    odometria();
    check_heading();
    check_curr_point();
    check_past_points();
    check_space();
    show_goal_param(2.0,0.0);
    set_current_point();
    update_map();
    printf("Aktualna pozycja robota: x: %.3f  y: %.3f  kat: %.6f\n",result_x, result_y, result_theta);
    printf("Aktualna komorka %d %d\n",cur_x,cur_y);
    printf("Current angle %d Correct angle %f\n",current_angle,correct_angle);
    printf("past theta %f space angle %f\n", avoid_past_theta, check_space_angle);
    //printf("dist %f goaltheta %f\n",dist,goaltheta);
    //printf ("us %f %f %f %f %f\n",usvalues[0],usvalues[1],usvalues[2],usvalues[3],usvalues[4]);
    for (j=0;j<22;j++)
    {
        for(i=0;i<22;i++)
        {
            printf("%d ",map[22-j-1][i]);
        }
        printf("\n");
    }
    //printf("Czujniki us\nL 90 (0): %d\nFL 45 (1): %d\nF 0(2): %d\nFR 45 (3): %d\nR 90 (4): %d\n", usvalues[0], usvalues[1], usvalues[2], usvalues[3], usvalues[4]);
    //fprintf(datatxt,"%.4f %.4f %.4f %d %d %d %d %d\n",result_x, result_y, result_theta, usvalues[0],usvalues[1],usvalues[2],usvalues[3],usvalues[4]);
    
    //if (current_angle!=4){
        //run_goto_heading(R_45);
    //}
    //go_front(14745);
    /*
    if (map[cur_x+1][cur_y]!=0)
    {
        kh4_set_speed(0,0,dsPic);
        printf ("STOP\n");
    }
    else{
        go_front(14745);
    }
    */
    usleep(100000);
    }
    //tcflush(0, TCIFLUSH); // flush input
    kh4_set_speed(0,0,dsPic ); // stop robot
    //kh4_SetMode( kh4RegIdle,dsPic ); // set motors to idle


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
    dummyItem = (struct DataItem*) malloc(sizeof(struct DataItem));
    dummyItem->data = -1;  
    dummyItem->key = -1;
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
//fclose(xytxt);
fclose(datatxt);
  kh4_set_speed(0 ,0 ,dsPic); // stop robot
  kh4_SetMode( kh4RegIdle,dsPic ); // set motors to idle
  kh4_SetRGBLeds(0,0,0,0,0,0,0,0,0,dsPic); // clear rgb leds because consumes energy

  return 0;
}
