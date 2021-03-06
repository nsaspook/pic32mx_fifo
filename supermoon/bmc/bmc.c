/*
 * MoonLight logger for ADS1220
 * 
 * Demo code for remote Daq using Comedi and sockets
 *
 * This file may be freely modified, distributed, and combined with
 * other software, as long as proper attribution is given in the
 * source code.
 */

#include <stdlib.h>
#include <stdio.h>	/* for printf() */
#include <unistd.h>
#include <string.h>
#include <comedilib.h>
#include "bmc/bmc.h"
#include "bmc/daq.h"
#include "bmc_x86/bmcnet.h"
#include <time.h>
#include <sys/time.h>
#include <X11/StringDefs.h>
#include <X11/Intrinsic.h>
#include <X11/Xaw/Box.h> 
#include <X11/Xaw/Label.h>        /* Include the Label widget's header file. */
#include <X11/Xaw/Cardinals.h>  /* Definition of ZERO. */
#include <X11/Xaw/Command.h> 

#define MDB	TRUE
#define MDB1	FALSE

#define ADOFFSET	-0.0000025
#define ADRES		4998.0 // OHM

struct bmcdata bmc;
unsigned char HAVE_DIO = TRUE, HAVE_AI = TRUE;

String fallback_resources[] = {"*Label.Label:    BMC", NULL};

// bmcnet server information
char hostip[32] = "10.1.1.41";
int hostport = 9760;
double gain_adj = ADGAIN1;
struct timeval start, end;

void quit(w, client, call)
Widget w;
XtPointer client;
XtPointer call;
{

	exit(0);

}

int main(int argc, char *argv[])
{

	Widget toplevel;
	Widget box;
	Widget command;
	Widget label;
	void quit();
	Arg wargs[10];
	int n, update = 0, update_num = 0, raw_data = 0, update_rate = 59;
	char net_message[256], solar_data[256];
	time_t rawtime, firsttime;
	struct tm * timeinfo;
	FILE *fp;
	double PVcal, PVi, PVp, sigtime = 0.0;

	/*
	 * start a new log file
	 */
	fp = fopen("moonlight.txt", "w+");
	fclose(fp);

	// Xwindows stuff for later
	toplevel = XtInitialize(argv[0], "simple", NULL, 0,
				&argc, argv);
	box = XtCreateManagedWidget("box", boxWidgetClass,
				toplevel, NULL, 0);
	n = 0;
	XtSetArg(wargs[n], XtNorientation, XtorientVertical);
	n++;
	XtSetArg(wargs[n], XtNvSpace, 10);
	n++;
	XtSetValues(box, wargs, n);
	label = XtCreateManagedWidget("label",
				labelWidgetClass, box, NULL, 0);
	n = 0;
	XtSetArg(wargs[n], XtNlabel, "Hello World");
	n++;
	XtSetValues(label, wargs, n);
	command = XtCreateManagedWidget("command",
					commandWidgetClass, box, NULL, 0);
	n = 0;
	XtSetArg(wargs[n], XtNlabel, "press and die");
	n++;
	XtSetValues(command, wargs, n);
	XtAddCallback(command, XtNcallback, quit, NULL);
	XtRealizeWidget(toplevel);

	if (init_daq() < 0) HAVE_AI = FALSE;
	if (init_dio() < 0) HAVE_DIO = FALSE;
	printf("\r\n Remote DAQ Client running        \r\n");

	get_data_sample(); /* clear the sample buffers */
	time(&firsttime);
	fp = fopen("moonlight.txt", "a");
	sprintf(solar_data, "         \r\n %2.6f, %1.9f, %2.6f, %ld",
		PVcal, PVp, bmc.pv_voltage_null, firsttime);
	fprintf(fp, "%s", solar_data);
	if (!RAW_DATA) {
		fclose(fp);
	}
	gettimeofday(&start, NULL);
	while (HAVE_AI && HAVE_DIO) {
		get_data_sample();
		gettimeofday(&end, NULL);
		if (++update >= update_rate || RAW_DATA) {
			if (MDB) {
				// sample time in fractions of a second
				sigtime = ((double) ((end.tv_sec * 1000000 + end.tv_usec)-(start.tv_sec * 1000000 + start.tv_usec))) / 1000000.0;
				time(&rawtime);
				/*
				 * update the console
				 */
				PVcal = (bmc.pv_voltage - bmc.pv_voltage_null + ADOFFSET) * gain_adj;
				PVi = PVcal / ADRES;
				PVp = PVcal*PVi;
				printf(" \r\n PV Voltage %2.6fV, PV Power %0.9fW, Raw data %x, PV Null %2.6fV, Raw Null %x, Raw time %ld, %3.6f",
				PVcal, PVp, bmc.raw[PVV_C], bmc.pv_voltage_null, bmc.raw[PVV_NULL], rawtime, sigtime);
				/*
				 * update the log file
				 */
				if (!RAW_DATA) fp = fopen("moonlight.txt", "a");
				if (SIGVIEW) {
					sprintf(solar_data, "%3.6f	%ld\r\n",
						sigtime, (int) (PVcal * 1000000.0));
				} else {
					sprintf(solar_data, "         \r\n %2.6f, %1.9f, %2.6f, %ld",
						PVcal, PVp, bmc.pv_voltage_null, rawtime - firsttime);
				}
				fprintf(fp, "%s", solar_data);
				if (!RAW_DATA) fclose(fp);
			}
		}
		usleep(50500); // ~5hz or 20 SPS
		if (++update >= update_rate + 1 || RAW_DATA) {
			update = 0;
			if (MDB1) {
				printf("\r\nUpdate number %d ", update_num++);
				time(&rawtime);
				printf("%ld ", rawtime);
			}
			sprintf(net_message, "%i,%i,%i,%i,X", (int) (bmc.pv_voltage * V_SCALE), (int) (bmc.cm_amps * C_SCALE), update_num, STATION);
			if (!RAW_DATA) bmc_client(net_message);
		}
		//        XtMainLoop(); // X-windows stuff for later...
		if (RAW_DATA && raw_data++ > 30000) {
			fclose(fp);
			break;
		}
	}

	printf("\r\n Remote DAQ Client exiting        \r\n");
	return 0;
}


