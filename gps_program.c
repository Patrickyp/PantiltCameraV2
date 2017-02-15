/* This program reads gps data on serial port using gpsd.  Every minute it will save the average 
*  Long/Lat/Alt into a text file.  Every 10 minutes the program will update the computer time to 
*  match gps time (When gps time comes in, start interrupt and wait for 3 interrupts.  Update system 
*  time to gps time + 2 seconds).
*	
*  Note that both gpsd and pps must be set up.  See PPS_Notes and Read_GPS_Serial_Notes.
*  see Odroid_GPS_Connections.odt for wiring.
*
*  To run
*  $ gcc -o gps_program gps_program.c -lgps
*  $ sudo ./gps_program
*/


#include <stdint.h>
#include <gps.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sched.h>
#include <sys/time.h>

#define SYSFS_GPIO_DIR  "/sys/class/gpio"
#define POLL_TIMEOUT    (3 * 1000) /* 3 seconds */
#define MAX_BUF         64


int data_count = 0;
float latitude = 0;
float longitude = 0;
float altitude = 0;

int gpio = 31; //<---- set pps pin here
uint32_t gps_data_interval = 20;  //<---- set how often to save average long/lat/alt here in sec
uint32_t update_time_interval = 40; //<--- set how often to update system time using pps here in sec

//function headers
int save_average();
int set_system_time(double unix_time);
int convert_time(double unix_time);
int pps_sync(double gps_time);
int gpio_export(unsigned int gpio);
int gpio_unexport(unsigned int gpio);
int gpio_set_dir(unsigned int gpio, unsigned int out_flag);
int gpio_set_value(unsigned int gpio, unsigned int value);
int gpio_get_value(unsigned int gpio, unsigned int *value);
int gpio_set_edge(unsigned int gpio, char *edge);
int gpio_fd_open(unsigned int gpio);
int gpio_fd_close(int fd);


int main() {

   // set a high priority schedulling for the running program
   {
      struct sched_param sched ;
        
      memset (&sched, 0, sizeof(sched)) ;
        
      sched.sched_priority = 55;
        
      sched_setscheduler (0, SCHED_RR, &sched) ;
   }
   //********set up gpsd code***********
   // 32 bit int to store starting time for gps_data_interval
   uint32_t serial_interval_start = time(NULL); 
   
	// Used to check for duplicate data, stores the gps time of previous data
	double prev_time_gps = 0;
   
   
	//rc holds return value of gps_open
	int rc;
	struct gps_data_t gps_data;
   
   // Check for gpsd error
	if ((rc = gps_open("localhost", "2947", &gps_data)) == -1) {
	    printf("code: %d, reason: %s\n", rc, gps_errstr(rc));
	    return EXIT_FAILURE;
	}
   
	gps_stream(&gps_data, WATCH_ENABLE | WATCH_JSON, NULL);
   
   
   
   // 32 bit int to store starting time for update_time_interval
   uint32_t pps_interval_start = time(NULL);
   
   //printf("\n%d %d \n", serial_interval_start, pps_interval_start); //check variable are correct
   
   int count = 0;  
	// start loop to read gps data
	while (1) {
	    /* time to wait to receive data */
	   if (gps_waiting (&gps_data, 100000)) {
		// read data
		if ((rc = gps_read(&gps_data)) == -1) {
		    printf("error occured reading gps data. code: %d, reason: %s\n", rc, gps_errstr(rc));
		} else {
		    // Check that valid gps data were obtained.
		    if ((gps_data.status == STATUS_FIX) && 
		        (gps_data.fix.mode == MODE_2D || gps_data.fix.mode == MODE_3D) && !isnan(gps_data.fix.latitude) && !isnan(gps_data.fix.longitude) && !isnan(gps_data.fix.time)) {
				
            // Get the gps time.
            double current_time_gps = gps_data.fix.time; 
            
            // Get the system time.
            uint32_t current = time(NULL); 
				
            // This if condition is to remove duplicate data because sometimes libgps outputs the same line of data twice and I couldn't figure out why
            // Check that current gps time != previous gps time before printing data.
				if (current_time_gps != prev_time_gps){
               //duplicate check update
					prev_time_gps = current_time_gps;
               
               //convert unix to standard time
				   convert_time(current_time_gps);
               
               // Store how long since the last time update using pps.
               uint32_t elapsed = current - pps_interval_start;
                              
               //check for 10 minutes to update time
					if (elapsed >= update_time_interval){
						printf("-------%d seconds passed[UPDATING TIME]--------\n", elapsed);
                  //start pps and set system time
						pps_sync(current_time_gps);
                  //set_system_time(current_time_gps);
                  printf("System time set!\n");
                  //restart loop, reset all variable to starting values
                  serial_interval_start = time(NULL);
                  prev_time_gps = 0;
                  pps_interval_start = time(NULL);
                  // Reset gpsd
               	if ((rc = gps_open("localhost", "2947", &gps_data)) == -1) {
               	    printf("code: %d, reason: %s\n", rc, gps_errstr(rc));
               	    return EXIT_FAILURE;
               	}
                  gps_stream(&gps_data, WATCH_ENABLE | WATCH_JSON, NULL);
                  continue;
               }
               // Check for 60 seconds to save average.
               current = time(NULL);
               elapsed = current - serial_interval_start;
					if (elapsed >= gps_data_interval){
						printf("--------%d seconds passed[SAVING AVERAGE]---------\n", elapsed);
						serial_interval_start = current;
						save_average();
                  continue;
					} else {
						data_count++;
						longitude += gps_data.fix.longitude;
						latitude += gps_data.fix.latitude;
						altitude += gps_data.fix.altitude;
					}
               count ++;
					printf("[#%d]height: %f, latitude: %f, longitude: %f, speed: %f, timestamp: %f\n", count, 
                        gps_data.fix.altitude, gps_data.fix.latitude, gps_data.fix.longitude, gps_data.fix.speed, current_time_gps);		//record stop time
					

				}
											
		    } else {
		        printf("no GPS data available\n");
		    }
		}
	    }
		//usleep(100000); //sleep for 500k microseconds
		//sleep(1);
	}

	/* When you are done... */
	gps_stream(&gps_data, WATCH_DISABLE, NULL);
	gps_close (&gps_data);

	return EXIT_SUCCESS;
}

// This function syncs time using pps.  First start reading pps interrupt, after the 3rd pulse, update system time with 
// the given gps_time + 2.  This is due to the fact that when you start the interrupt there is a always a pulse in the beginning 
// that is not from the pps so the 2nd pulse is actually the "next" pps. I waited for the 3rd pulse just to be sure it is in sync.
int pps_sync(double gps_time){
   //pollfd is a struct with 3 members: int fd(descriptor), short events(specified events), short revents(events found returned).
	//fdset is a list of pollfd structs
	struct pollfd fdset[2]; // this creates an array of pollfd structs called fdset
	int nfds = 2; //the number of items in fdset
	int gpio_fd, timeout, rc; // return value of gpio_fd_open, timeout duration to wait for interrupt, 
	char *buf[MAX_BUF];	
	unsigned int gpio = 31; //the gpio pin number to read in
	int len;
	
   
	gpio_export(gpio);
	gpio_set_dir(gpio, 0);
	gpio_set_edge(gpio, "rising");    // rising/falling/both Edge Trigger
	gpio_fd = gpio_fd_open(gpio);

	timeout = POLL_TIMEOUT;

	//while (1) {
	for(int z = 0; z <= 5; z++){
		memset((void*)fdset, 0, sizeof(fdset)); //set initial values of fdset to 0?
		//pollfd is a struct with 3 members: int fd(descriptor), short events(specified events), short revents(events found returned).
		//fdset is a list of pollfd structs
		fdset[0].fd = STDIN_FILENO;
		fdset[0].events = POLLIN; //POLLIN is normal data.
      
		fdset[1].fd = gpio_fd;
		fdset[1].events = POLLPRI; //POLLPRI is urgent data.
		
		//poll will take a list of pollfd, number of items in pollfd list, and timeout duration
		rc = poll(fdset, nfds, timeout);      

		if (rc < 0) {
			printf("\npoll() failed!\n");
			return -1;
		}
      
		if (rc == 0) {
			printf("POLL Timeout!\n");
		}
            
		if (fdset[1].revents & POLLPRI) {
         if (z == 2){
            double final_time = gps_time + 2;
            set_system_time(final_time);
            printf("system time set to +2 seconds\n");
         }
			len = read(fdset[1].fd, buf, MAX_BUF);
			printf("\npoll() GPIO %d interrupt occurred # %d\n", gpio, z);
		}

		if (fdset[0].revents & POLLIN) {
			(void)read(fdset[0].fd, buf, 1);
			printf("\npoll() stdin read 0x%2.2X\n", (unsigned int) buf[0]);
		}

		fflush(stdout);
	}
   
   //unexport the pin when done
	gpio_unexport(gpio);
	gpio_fd_close(gpio_fd);
	return 0;

}

// Save average long/lat/alt into a file and reset those values back to 0.
int save_average(){
	float avg_lat = latitude / data_count;
	float avg_long = longitude / data_count;
	float avg_alt = altitude / data_count;

	FILE *fp;
	fp = fopen("data_history.txt", "a");
	fprintf(fp, "Long: %f, Latitude: %f, Altitude: %f, Data Count: %d.\n", avg_lat, avg_long, avg_alt, data_count);
	//fputs("This is testing for fputs...\n", fp);
	fclose(fp);

	fp = fopen("current_data.txt", "w");
	fprintf(fp, "%f\n%f\n%f", avg_lat, avg_long, avg_alt);
	//fputs("This is testing for fputs...\n", fp);
	fclose(fp);

	latitude = 0;
	longitude = 0;
	altitude = 0;
	data_count = 0;
}

// First convert the given unix time into tm struct and set system time.
int set_system_time(double unix_time){
   //cast from double to time_t
   time_t gps_unix_time = unix_time;
   //turn time_t as a local time into a struct with sec,min,hr etc
   struct tm* tm_ptr = localtime(&gps_unix_time);
   
   const struct timeval tv = {mktime(tm_ptr), 0};

   //set the system time using the given unix time from gps
   settimeofday(&tv, 0);

}

// Take as input a unix time and returns the current minutes and print out the time as 
// local time in standard format.  
int convert_time(double unix_time){
   // tm has 9 int members e.g. sec, min, hour etc, lt will hold converted time as localtime
   struct tm lt;
   
	time_t t = unix_time;
   
   localtime_r(&t, &lt);
   //print minutes
   //printf("minutes: %d\n",lt.tm_min);
   
   // Optional print out full time date for debugging
   char res[32];
   // See documentation of strftime for the format
   static const char default_format[] = "%a %b %d %Y, %I:%M:%S";
   // Store the time as defined by default_format in res
   if (strftime(res, sizeof(res), default_format, &lt) == 0) {
                (void) fprintf(stderr,  "strftime(3): cannot format supplied "
                                        "date/time into buffer of size %u "
                                        "using: '%s'\n",
                                        sizeof(res), default_format);
                return 1;
   }
   // print unix time and converted time
   printf("%u -> '%s'\n", (unsigned) t, res);
   
   return lt.tm_min;
}

//-----------------------------------------------------------------------------
int gpio_export(unsigned int gpio)
{
	int fd, len;
	char buf[MAX_BUF];
 
	fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
	if (fd < 0) {
		perror("gpio/export");
		return fd;
	}
 
	len = snprintf(buf, sizeof(buf), "%d", gpio);
	write(fd, buf, len);
	close(fd);
 
	return 0;
}

//-----------------------------------------------------------------------------
int gpio_unexport(unsigned int gpio)
{
	int fd, len;
	char buf[MAX_BUF];
 
	fd = open(SYSFS_GPIO_DIR "/unexport", O_WRONLY);
	if (fd < 0) {
		perror("gpio/export");
		return fd;
	}
 
	len = snprintf(buf, sizeof(buf), "%d", gpio);
	write(fd, buf, len);
	close(fd);
	return 0;
}

//-----------------------------------------------------------------------------
int gpio_set_dir(unsigned int gpio, unsigned int out_flag)
{
	int fd, len;
	char buf[MAX_BUF];
 
	len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR  "/gpio%d/direction", gpio);
 
	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("gpio/direction");
		return fd;
	}
 
	if (out_flag)
		write(fd, "out", 4);
	else
		write(fd, "in", 3);
	close(fd);
	return 0;
}

//-----------------------------------------------------------------------------
int gpio_set_value(unsigned int gpio, unsigned int value)
{
	int fd, len;
	char buf[MAX_BUF];
 
	len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);
 
	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("gpio/set-value");
		return fd;
	}
 
	if (value)
		write(fd, "1", 2);
	else
		write(fd, "0", 2);
 
	close(fd);
	return 0;
}

//-----------------------------------------------------------------------------
int gpio_get_value(unsigned int gpio, unsigned int *value)
{
	int fd, len;
	char buf[MAX_BUF];
	char ch;

	len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);
 
	fd = open(buf, O_RDONLY);
	if (fd < 0) {
		perror("gpio/get-value");
		return fd;
	}
 
	read(fd, &ch, 1);

	if (ch != '0') {
		*value = 1;
	} else {
		*value = 0;
	}
 
	close(fd);
	return 0;
}

//-----------------------------------------------------------------------------
int gpio_set_edge(unsigned int gpio, char *edge)
{
	int fd, len;
	char buf[MAX_BUF];

	len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/edge", gpio);
 
	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("gpio/set-edge");
		return fd;
	}
 
	write(fd, edge, strlen(edge) + 1); 
	close(fd);
	return 0;
}

// ---------------------------------------------------------------------------
int gpio_fd_open(unsigned int gpio)
{
	int fd, len;
	char buf[MAX_BUF];

	len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);
 
	fd = open(buf, O_RDONLY | O_NONBLOCK );
	if (fd < 0) {
		perror("gpio/fd_open");
	}
	return fd;
}

//-----------------------------------------------------------------------------
int gpio_fd_close(int fd)
{
	return close(fd);
}
