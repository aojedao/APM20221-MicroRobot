/******************************************************************************
* rc_motor_control.c
*
* This is a simple dc motor control program. It takes in a character
* and then controls the motors to move forward, reverse, left or right
******************************************************************************/

#include <rc_usefulincludes.h>
#include <roboticscape.h>
#include <rc/motor.h>
#include <rc/time.h> 

/******************************************************************************
* int main()
*
* This main function contains these critical components
* - call to initialize_cape
* - main while loop that checks for EXITING condition
* - switch statement to send proper controls to the motors
* - cleanup_roboticscape() at the end
******************************************************************************/
int main(){
  char input;
  int freq_hz = 25000;
  // always initialize cape library first
  rc_initialize();

  printf("\nHello BeagleBone\n");

  // done initializing so set state to RUNNING
  rc_set_state(RUNNING);

  // bring H-bridges of of standby
  rc_enable_motors();
  //rc_motor_init_freq(freq_hz);
  //rc_motor_init();
  if(rc_motor_init()==-1){
     fprintf(stderr,"ERROR: failed to initialize motors\n");
     return -1;
     }
  //rc_motor_standby(1); // start with motors in standby
  rc_led_set(RC_LED_GREEN,1);
  rc_led_set(RC_LED_RED,1);

  rc_set_motor_free_spin(1);
  rc_set_motor_free_spin(2);
  printf("Motors are now ready.\n");
  // Turn on a raw terminal to get a single character
  system("stty raw");
  do
    {
      printf("> ");
      input = getchar();
      switch(input){
      case 'f':
        rc_motor_set(1, 0.5);
        rc_motor_set(2, 0.5);
        break;
      case 'r':
        rc_set_motor(1, 0.5);
        rc_set_motor(2, -0.5);
        break;
      case 'l':
        rc_set_motor(1, -0.5);
        rc_set_motor(2, 0.5);
        break;
      case 'b':
        rc_set_motor(1, -0.5);
        rc_set_motor(2, -0.5);
        break;
      case 's':
        rc_set_motor_brake_all();
        break;
      case 'q':
        rc_disable_motors();
        break;
      default:
        printf("Invalid Character.\n");
      }
    }
  while(input != 'q');
  printf("Done\n");
  rc_cleanup();
  system("stty cooked");

  return 0;
}
