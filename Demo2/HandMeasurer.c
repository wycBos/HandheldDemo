/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+
#+     Glade / Gtk Programming
#+
#+     Copyright (C) 2019 by Kevin C. O'Kane
#+
#+     Kevin C. O'Kane
#+     kc.okane@gmail.com
#+     https://www.cs.uni.edu/~okane
#+     http://threadsafebooks.com/
#+
#+ This program is free software; you can redistribute it and/or modify
#+ it under the terms of the GNU General Public License as published by
#+ the Free Software Foundation; either version 2 of the License, or
#+ (at your option) any later version.
#+
#+ This program is distributed in the hope that it will be useful,
#+ but WITHOUT ANY WARRANTY; without even the implied warranty of
#+ MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#+ GNU General Public License for more details.
#+
#+ You should have received a copy of the GNU General Public License
#+ along with this program; if not, write to the Free Software
#+ Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
#+
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

// #include "gpioTest.h"
// #include "UART_test.h"
#include "waveForm.h"
// #include "ADS1x15.h"
#include <stdlib.h>
#include <sys/types.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <gtk/gtk.h>
#include <gtk/gtkx.h>
#include <gst/gst.h>
#include <math.h>
#include <time.h>
#include <ctype.h>
#include <sys/mman.h>
#include <stdio.h>
#include <linux/limits.h>
// #include <wiringPi.h>
#include <pigpio.h>
#include <pthread.h>
#include <signal.h>
// #include <raspicam/raspicam.h>
#include <gst/video/videooverlay.h>
#include <gst/video/video.h>
// #include <gst/interfaces/xoverlay.h>
#include <Python.h>
#if defined(GDK_WINDOWING_X11)
#include <gdk/gdkx.h>
#elif defined(GDK_WINDOWING_WIN32)
#include <gdk/gdkwin32.h>
#elif defined(GDK_WINDOWING_QUARTZ)
#include <gdk/gdkquartz.h>
#endif

#include "piSerial.h"
#include "AD_DAC.h"
#include "UART_test.h"
/* GPIOs for Buttons */
#define LEFT_BUTTON 22	 // Input
#define MIDDLE_BUTTON 27 // Input
#define RIGHT_BUTTON 17	 // Input

/* GPIOs for UART port */
#define UART_SELEC 1 // 0 - Dist measu; 1 - Temp ctrl
// distance measurement

// TEMP controller
#define TEMP_ENAB 23 // 0 - Enable; 1 - Disable
#define TEMP_STAT 24 // Input

/* Laser Detector enable */
#define LASER_DETECT_EN 25 // 1 - Enable; 0 - Disable

/* GPIOs for ADC/DAC */
#define ADC_SELEC 8 // 0 - Enable; 1 - Disable
#define DAC_SELEC 7 // 0 - Enable; 1 - Disable

// ADC Control
#define ADC_CLK_EN 21 // 0 - Disable; 1 - Enable
#define ADC_DRDY 16	  // Input, 0 - Data Ready; 1 - Disable
#define ADC_REST 20	  // 0 - Reset; 1 - No Effect

// DAC Control
#define DAC_LDAC 19 // 0 - Disable; 1 - Enable

GtkWidget *window;
GtkWidget *fixed1;
GtkWidget *left_label;
GtkWidget *middle_label;
GtkWidget *right_label;
GtkWidget *status_label;
GtkWidget *bat_label;
GtkWidget *eventbox_label_l;
GtkWidget *eventbox_label_m;
GtkWidget *eventbox_label_r;
GtkWidget *eventbox_status;
GtkWidget *eventbox_ppm;
GtkWidget *date_label;
GtkWidget *time_label;
GtkWidget *ppm_display_label;
GtkWidget *video_screen;
GtkWidget *splash_screen;
GtkWidget *laser_on, *laser_off, *gps_on, *gps_off;
GtkWidget *setup_fields_labels;
GtkWidget *setup_fields_values;
GtkWidget *info_laser, *info_gps, *info_bat;
GtkWidget *setup_label_1, *setup_label_2, *setup_label_3, *setup_label_4, *setup_label_5, *setup_label_6, *setup_label_7;
GtkTextView *setup_value_1, *setup_value_2, *setup_value_3, *setup_value_4, *setup_value_5, *setup_value_6, *setup_value_7;

GtkBuilder *builder;

GstElement *pipeline, *source, *sink, *convert, *filter, *flip;
GstCaps *videosrc_caps;
GstBus *bus;
GstMessage *msg;
GstStateChangeReturn ret;
gboolean status = 0;

GdkWindow *video_window_xwindow;
gulong embed_xid;
GstStateChangeReturn sret;

GstSample *from_sample, *to_sample;
GstCaps *image_caps;
GstBuffer *buf;
GstMapInfo map_info;
GError *err = NULL;

void on_destroy();

/* sharing data */
enum Mode
{
	Splash = 0,
	Setup,
	Idle,
	BarGraph,
	PPM,
	LiveCam,
	IRCam,
	Shutdown
} OpMode;

/* gas thread data */
enum mState
{
	measEmpty = -1,
	measEnter = 0,
	measSet,
	measIdle,
	measReady,
	measAdjst
} measState;

const gchar *labelstring, *setup_buf;
time_t current_time;
struct tm *time_info;
// gpointer *ptime_info = &time_info;
int current_min = -1;
int current_sec = -1;
int ppm = 0;
int cam = 0;
long int timedate;
int counter = 0;
float dist = 0;

/* gas measure thread data TODO - no used now */
typedef struct measThread_t
{
	int curMeasSt;
	void *measData;
} measThr;

measData mData;
ads1x15_p adcMain;

GMutex mutex_1, mutex_2, mutex_3, mutex_data;
pthread_mutex_t pmtx_mData, pmtx_mode;
/**
	 * C++ version 0.4 char* style "itoa":
	 * Written by Lukás Chmela
	 * Released under GPLv3.

	 */
char *itoa(int value, char *result, int base)
{
	// check that the base if valid
	if (base < 2 || base > 36)
	{
		*result = '\0';
		return result;
	}

	char *ptr = result, *ptr1 = result, tmp_char;
	int tmp_value;

	do
	{
		tmp_value = value;
		value /= base;
		*ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz"[35 + (tmp_value - value * base)];
	} while (value);

	// Apply negative sign
	if (tmp_value < 0)
		*ptr++ = '-';
	*ptr-- = '\0';
	while (ptr1 < ptr)
	{
		tmp_char = *ptr;
		*ptr-- = *ptr1;
		*ptr1++ = tmp_char;
	}
	return result;
}

void on_destroy()
{
	gtk_main_quit();
}

void cleanup(int signo)
{
	// pullUpDnControl(LEFT_BUTTON, PUD_DOWN);
	// pullUpDnControl(MIDDLE_BUTTON, PUD_DOWN);
	// pullUpDnControl(RIGHT_BUTTON, PUD_DOWN);

	gpioSetPullUpDown(LEFT_BUTTON, PI_PUD_DOWN /*PI_PUD_UP*/);
	gpioSetPullUpDown(MIDDLE_BUTTON, PI_PUD_DOWN);
	gpioSetPullUpDown(RIGHT_BUTTON, PI_PUD_DOWN);

	/* add tunr laser/TempCtrl off */

	exit(0);
}
/* TODO - move to python routints file */
char *call_Python_QR(int argc, char *argv1, char *argv2)
{
	PyObject *pName, *pModule, *pDict, *pFunc, *pValue, *pmyresult;
	char *l_UserID;
	// Set PYTHONPATH TO working directory
	setenv("PYTHONPATH", ".", 1);

	wchar_t *program = Py_DecodeLocale(argv1, NULL);
	if (program == NULL)
	{
		fprintf(stderr, "Fatal error: cannot decode argv[0]\n");
		exit(1);
	}
	Py_SetProgramName(program); /* optional but recommended */
	// Initialize the Python Interpreter
	Py_Initialize();

	// Build the name object
	pName = PyUnicode_DecodeFSDefault((char *)argv1);

	// Load the module object
	pModule = PyImport_Import(pName);

	// pDict is a borrowed reference
	pDict = PyModule_GetDict(pModule);

	// pFunc is also a borrowed reference
	pFunc = PyDict_GetItemString(pDict, argv2);

	if (PyCallable_Check(pFunc))
	{
		pmyresult = PyObject_CallObject(pFunc, NULL);
	}
	else
	{
		PyErr_Print();
	}
	if (PyUnicode_Check(pmyresult))
	{
		PyObject *temp_bytes = PyUnicode_AsEncodedString(pmyresult, "UTF-8", "strict"); // Owned reference
		if (temp_bytes != NULL)
		{
			l_UserID = PyBytes_AS_STRING(temp_bytes); // Borrowed pointer
			l_UserID = strdup(l_UserID);
			Py_DECREF(temp_bytes);
		}
		else
		{
			// TODO: Handle encoding error.
		}
	}

	// Clean up
	Py_DECREF(pModule);
	Py_DECREF(pName);

	// Finish the Python Interpreter
	Py_Finalize();

	return l_UserID;
}
/* TODO - move to python routints file */
int call_Python_Stitch(int argc, char *argv1, char *argv2, char *argv3, char *argv4, char *argv5, char *argv6)
{
	PyObject *pName, *pModule, *pDict, *pFunc, *pValue, *pmyresult, *args, *kwargs;
	int i;
	// Set PYTHONPATH TO working directory
	setenv("PYTHONPATH", ".", 1);

	wchar_t *program = Py_DecodeLocale(argv1, NULL);
	if (program == NULL)
	{
		fprintf(stderr, "Fatal error: cannot decode argv[0]\n");
		exit(1);
	}
	Py_SetProgramName(program); /* optional but recommended */
	// Initialize the Python Interpreter
	Py_Initialize();

	// Build the name object
	pName = PyUnicode_DecodeFSDefault(argv1);

	// Load the module object
	pModule = PyImport_Import(pName);

	// pDict is a borrowed reference
	pDict = PyModule_GetDict(pModule);

	// pFunc is also a borrowed reference
	pFunc = PyDict_GetItemString(pDict, argv2);

	// args = PyTuple_Pack(2,PyUnicode_DecodeFSDefault(argv3), PyUnicode_DecodeFSDefault(argv4));
	// kwargs = PyTuple_Pack(2,PyUnicode_DecodeFSDefault(argv5), PyUnicode_DecodeFSDefault(argv6));
	args = Py_BuildValue("ssss", argv5, argv3, argv6, argv4);
	// kwargs = Py_BuildValue("ss", argv5, argv6);
	if (PyCallable_Check(pFunc))
	{
		pmyresult = PyObject_CallObject(pFunc, NULL);
		i = 0;
	}
	else
	{
		PyErr_Print();
		i = 1;
	}

	// Clean up
	Py_DECREF(pModule);
	Py_DECREF(pName);

	// Finish the Python Interpreter
	Py_Finalize();

	return i;
}

void setup_filestructure()
{
	GtkTextIter start, end;
	GtkTextBuffer *userID;
	gchar *text;
	userID = gtk_text_view_get_buffer(setup_value_7);
	gtk_text_buffer_get_bounds(userID, &start, &end);
	text = gtk_text_buffer_get_text(userID, &start, &end, FALSE);
	if (strcmp(text, "") == 0)
	{
		text = call_Python_QR(2, "Decode_QR", "decode");
	}
	gtk_text_buffer_set_text(userID, text, strlen(text));
	gtk_text_view_set_buffer(setup_value_7, userID);
}
/* TODO - it might be moved to Gtk_proc.c file */
void left_button_pressed(int gpio, int level, uint32_t tick)
{
	// printf("left-isr %d gpio, %d level, %u\n", gpio, level, tick);
	gpioDelay(600000); // delay(500);
	if (level != 0)
		return;

	labelstring = gtk_label_get_text(GTK_LABEL(left_label));
	if (strcmp(labelstring, "Setup") == 0)
	{
		gtk_label_set_text(GTK_LABEL(left_label), (const gchar *)"Return");
		gtk_label_set_text(GTK_LABEL(middle_label), (const gchar *)"Save");
		gtk_label_set_text(GTK_LABEL(right_label), (const gchar *)"");
		gtk_label_set_text(GTK_LABEL(status_label), (const gchar *)"Setup");
		gtk_widget_hide(eventbox_ppm);
		gtk_widget_hide(ppm_display_label);
		gtk_widget_hide(video_screen);
		gtk_widget_show(setup_fields_labels);
		gtk_widget_show(setup_fields_values);
		OpMode = Setup;
	}
	else if (strcmp(labelstring, "Return") == 0)
	{
		gtk_label_set_text(GTK_LABEL(left_label), (const gchar *)"Setup");
		gtk_label_set_text(GTK_LABEL(middle_label), (const gchar *)"Start");
		gtk_label_set_text(GTK_LABEL(right_label), (const gchar *)"Exit");
		gtk_label_set_text(GTK_LABEL(status_label), (const gchar *)"Idle");
		gtk_widget_hide(setup_fields_labels);
		gtk_widget_hide(setup_fields_values);
		OpMode = Idle;
	}
	else if (strcmp(labelstring, "Yes") == 0)
	{
		labelstring = gtk_label_get_text(GTK_LABEL(status_label));
		if (strcmp(labelstring, "Confirm Exit") == 0)
		{
			// if(mData.wid >= 0) // stop the waveforms.
			//{
			//	wavePistop(mData.wid);
			// }

			gtk_label_set_text(GTK_LABEL(status_label), (const gchar *)"Bye");
			OpMode = Shutdown;
		}
	}
	else if (strcmp(labelstring, "PPM/DIST") == 0)
	{
		// if(mData.wid < 0)
		//{
		// mData.wid = wavePiset();
		//}
		gtk_widget_show(ppm_display_label);
		gtk_widget_show(eventbox_ppm);
		//gtk_label_set_text(GTK_LABEL(left_label), (const gchar *)"");
		//gtk_label_set_text(GTK_LABEL(middle_label), (const gchar *)"");
		gtk_label_set_text(GTK_LABEL(left_label), (const gchar *)"IR Cam");
		gtk_label_set_text(GTK_LABEL(middle_label), (const gchar *)"Snapshot");

		gtk_label_set_text(GTK_LABEL(right_label), (const gchar *)"Quit");
		gtk_label_set_text(GTK_LABEL(status_label), (const gchar *)"Survey PPM");

		/* reset SER_SEL for distance measurement */

		OpMode = PPM;
	}
	else if (strcmp(labelstring, "IR Cam") == 0)
	{
		gtk_label_set_text(GTK_LABEL(left_label), (const gchar *)"");
		gtk_label_set_text(GTK_LABEL(middle_label), (const gchar *)"");
		gtk_label_set_text(GTK_LABEL(right_label), (const gchar *)"Quit");
		gtk_label_set_text(GTK_LABEL(status_label), (const gchar *)"IR Camera");
		OpMode = IRCam;
	}
}
/* TODO - it might be moved to Gtk_proc.c file */
void middle_button_pressed(int gpio, int level, uint32_t tick)
{
	// printf("middle-isr %d gpio, %d level, %u\n", gpio, level, tick);
	gpioDelay(600000); // delay(500);
	if (level != 0)
		return;

	labelstring = gtk_label_get_text(GTK_LABEL(middle_label));
	if (strcmp(labelstring, "Start") == 0)
	{
		gtk_label_set_text(GTK_LABEL(status_label), (const gchar *)"Startup");
		// call startup sequence here
		gtk_label_set_text(GTK_LABEL(status_label), (const gchar *)"Choose Mode");
		gtk_label_set_text(GTK_LABEL(left_label), (const gchar *)"PPM/DIST");
		gtk_label_set_text(GTK_LABEL(middle_label), (const gchar *)"");
		//gtk_label_set_text(GTK_LABEL(middle_label), (const gchar *)"Live Cam");
		gtk_label_set_text(GTK_LABEL(right_label), (const gchar *)"Bar Graph");
		// printf("middle-start\n");
	}
	else if (strcmp(labelstring, "Live Cam") == 0) //change to PPM mode
	{
		gtk_widget_hide(ppm_display_label);
		gtk_widget_hide(eventbox_ppm);
		// gtk_label_set_text(GTK_LABEL(ppm_display_label), (const gchar* ) "");
		gtk_label_set_text(GTK_LABEL(left_label), (const gchar *)"IR Cam");
		gtk_label_set_text(GTK_LABEL(middle_label), (const gchar *)"Snapshot");
		gtk_label_set_text(GTK_LABEL(right_label), (const gchar *)"Quit");
		gtk_label_set_text(GTK_LABEL(status_label), (const gchar *)"Survey Live");
		OpMode = LiveCam;
		// printf("middle-Live Cam\n");
	}
	else if (strcmp(labelstring, "Snapshot") == 0)
	{
		g_object_get(sink, "last-sample", &from_sample, NULL);
		if (from_sample == NULL)
		{
			GST_ERROR("Error getting last sample form sink");
			return;
		}
		image_caps = gst_caps_from_string("image/png");
		to_sample = gst_video_convert_sample(from_sample, image_caps, GST_CLOCK_TIME_NONE, &err);
		gst_caps_unref(image_caps);
		gst_sample_unref(from_sample);

		if (to_sample == NULL && err)
		{
			GST_ERROR("Error converting frame: %s", err->message);
			g_printerr("Error converting frame. \n");
			g_error_free(err);
			return;
		}
		buf = gst_sample_get_buffer(to_sample);
		if (gst_buffer_map(buf, &map_info, GST_MAP_READ))
		{
			if (!g_file_set_contents("Snap.png", (const char *)map_info.data,
									 map_info.size, &err))
			{
				GST_CAT_LEVEL_LOG(GST_CAT_DEFAULT, GST_LEVEL_WARNING, NULL, "Could not save thumbnail: %s", err->message);
				g_error_free(err);
			}
		}
	}
	else if (strcmp(labelstring, "Edit") == 0)
	{
		gtk_label_set_text(GTK_LABEL(left_label), (const gchar *)"Return");
		gtk_label_set_text(GTK_LABEL(right_label), (const gchar *)"Save");
		gtk_text_view_set_editable(setup_value_1, True);
		gtk_text_view_set_cursor_visible(setup_value_1, True);
	}
}
/* TODO - it might be moved to Gtk_proc.c file */
void right_button_pressed(int gpio, int level, uint32_t tick)
{
	// printf("right-isr %d gpio, %d level, %u\n", gpio, level, tick);
	gpioDelay(600000); // delay(500);
	if (level != 0)
		return;
	labelstring = gtk_label_get_text(GTK_LABEL(right_label));
	if (strcmp(labelstring, "Exit") == 0)
	{
		gtk_label_set_text(GTK_LABEL(left_label), (const gchar *)"Yes");
		gtk_label_set_text(GTK_LABEL(middle_label), (const gchar *)"");
		gtk_label_set_text(GTK_LABEL(right_label), (const gchar *)"No");
		gtk_label_set_text(GTK_LABEL(status_label), (const gchar *)"Confirm Exit");
	}
	else if (strcmp(labelstring, "No") == 0)
	{
		labelstring = gtk_label_get_text(GTK_LABEL(status_label));
		if (strcmp(labelstring, "Confirm Exit") == 0)
		{
			gtk_label_set_text(GTK_LABEL(left_label), (const gchar *)"Setup");
			gtk_label_set_text(GTK_LABEL(middle_label), (const gchar *)"Start");
			gtk_label_set_text(GTK_LABEL(right_label), (const gchar *)"Exit");
			gtk_label_set_text(GTK_LABEL(status_label), (const gchar *)"Idle");
			OpMode = Idle;
		}
		// printf("right-No\n");
	}
	else if (strcmp(labelstring, "IR Cam") == 0)
	{
		gtk_label_set_text(GTK_LABEL(left_label), (const gchar *)"");
		gtk_label_set_text(GTK_LABEL(middle_label), (const gchar *)"");
		gtk_label_set_text(GTK_LABEL(right_label), (const gchar *)"Quit");
		gtk_label_set_text(GTK_LABEL(status_label), (const gchar *)"Survey IR");
		OpMode = IRCam;
	}
	else if (strcmp(labelstring, "Quit") == 0)
	{
		gtk_label_set_text(GTK_LABEL(left_label), (const gchar *)"Setup");
		gtk_label_set_text(GTK_LABEL(middle_label), (const gchar *)"Start");
		gtk_label_set_text(GTK_LABEL(right_label), (const gchar *)"Exit");
		gtk_label_set_text(GTK_LABEL(status_label), (const gchar *)"Idle");
		OpMode = Idle;
		printf("right-Quit\n");
		// if(mData.wid >= 0)
		//{
		//	wavePistop(mData.wid);
		//	mData.wid = -1;
		// }
	}
	else if (strcmp(labelstring, "Save") == 0)
	{
	}
}
/* TODO - it might be moved to measurer_utility.c file */
gboolean update_ppm(gpointer ppm) // TODO - remove it later
{
	char buffer[10];

	// g_mutex_lock(&mutex_1);
	pthread_mutex_lock(&pmtx_mData); // TODO -check if it's need?

	// update the GUI here:
	// gtk_button_set_label(button,"label");
	itoa((int)ppm, buffer, 10);
	// itoa((float)ppm, buffer, 10);
	strcat(buffer, " PPM");
	strcat(buffer, "\n\r 8.0 M");
	gtk_label_set_text(GTK_LABEL(ppm_display_label), buffer);

	// And read the GUI also here, before the mutex to be unlocked:
	// gchar * text = gtk_entry_get_text(GTK_ENTRY(entry));
	// g_mutex_unlock(&mutex_1);
	pthread_mutex_unlock(&pmtx_mData);
	return FALSE;
}
/* TODO - it might be moved to measurer_utility.c file */
gboolean update_meas(gpointer mData)
{
	char buffer[30], buffer1[10], buffer2[10];
	measData lmData = *(measData *)mData;

	// g_mutex_lock(&mutex_1);
	pthread_mutex_lock(&pmtx_mData);
	/* update the GUI here: */
	// gtk_button_set_label(button,"label");
	// itoa((int)ppm, buffer, 10); //DONE ppm=>ydays
	// itoa(lmData.ppm, buffer, 10);
	// sprintf(buffer, "%d", lmData.ppm);
	sprintf(buffer, "\n\n\r%.2f", lmData.gas_ppm);
	strcat(buffer, " PPM\n\r");
	sprintf(buffer1, "%2.2f", lmData.dist);
	strcat(buffer1, " M\n\r");
	sprintf(buffer2, "%2.2f", lmData.ADVoltag);
	strcat(buffer2, " V\n\r");
	strcat(strcat(buffer, buffer1), buffer2);
	gtk_label_set_text(GTK_LABEL(ppm_display_label), buffer);

	// And read the GUI also here, before the mutex to be unlocked:
	// gchar * text = gtk_entry_get_text(GTK_ENTRY(entry));
	// g_mutex_unlock(&mutex_1);
	pthread_mutex_unlock(&pmtx_mData);

	return FALSE;
}

gboolean update_time(gpointer time_info)
{
	char dateString[9];
	char timeString[9];
	g_mutex_lock(&mutex_2);
	// update the GUI here:
	// gtk_button_set_label(button,"label");

	// char *cp = calloc(1, sizeof(dateString));//test

	strftime(dateString, sizeof(dateString), "%D", time_info);
	gtk_label_set_text(GTK_LABEL(date_label), dateString);
	strftime(timeString, sizeof(timeString), "%I:%M %p", time_info);
	gtk_label_set_text(GTK_LABEL(time_label), timeString);

	// And read the GUI also here, before the mutex to be unlocked:
	// gchar * text = gtk_entry_get_text(GTK_ENTRY(entry));
	g_mutex_unlock(&mutex_2);

	return FALSE;
}
/* TODO - it might be moved to Gtk_proc.c file */
gboolean live_stream(gpointer pipeline)
{
	char dateString[9];
	char timeString[9];
	GstStateChangeReturn sret;
	g_mutex_lock(&mutex_3);
	// update the GUI here:
	// gtk_button_set_label(button,"label");

	strftime(dateString, sizeof(dateString), "%D", time_info);
	gtk_label_set_text(GTK_LABEL(date_label), dateString);
	strftime(timeString, sizeof(timeString), "%I:%M %p", time_info);
	gtk_label_set_text(GTK_LABEL(time_label), timeString);

	if (status == 0)
	{
		/* run the pipeline */
		gtk_widget_show(video_screen);
		sret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
		status = 1;
		g_printerr("Called set pipeline to PLAYING. \n");
		if (sret == GST_STATE_CHANGE_FAILURE)
		{
			gst_element_set_state(pipeline, GST_STATE_NULL);
			g_printerr("Could not set pipeline to PLAYING. \n");
			status = 0;
			return -1;
		}
	}

	// And read the GUI also here, before the mutex to be unlocked:
	// gchar * text = gtk_entry_get_text(GTK_ENTRY(entry));
	g_mutex_unlock(&mutex_3);

	return FALSE;
}

/* TODO - It is modified to be the dev_mainloop() */
void *start_loop_thread(void *arg)
{
	int lMode; // TODO - used to duplicate a local op-mode. it's used for generating random value for test right now.
	uint16_t runTemp = 0;

	char lineData[64], parm0[16], parm1[16], parm2[16], parm3[16], parm4[16];
	char dateString[9], timeString[9];
	int lydays, ret;
	float ltempPoint, lDAch0, lConMod;

	// FILE *fh;
	int readin, fret; //, retI, outN = 0;

	measData mloopData;

	time_t mlcurrent_time;
	struct tm *mltime_info;

	char img_filename[32], buffer[5]; // for image name
	int lcurrent_sec = -1, LSDisTiming = 10;
	float dis = 0;

	/* initialize mloopData */
	// 1- get configuration data from a file
#if 1 // for debugM1 - from inside of the while()
	// TODO - call getSets(const char *filename, measData *pDataSet);
	fret = getSets("./settingsDataNew.txt", &mloopData);
	// printf("temp-point: %.2f, DACH0: %.2f, Modulus: %.2f\n", mloopData.tecSets.tempPoint, mloopData.dacSets.voltCh0, mloopData.adcSets.conMod);
	printf("temp-point: %.2f, DACHMD: %.2f, DACHDC: %.2f, Modulus: %.2f\n", mloopData.tecSets.tempPoint, mloopData.dacSets.voltCh0, mloopData.dacSets.voltCh1, mloopData.adcSets.conMod);

	/* get current date-time */

	time(&mlcurrent_time);
	mltime_info = localtime(&mlcurrent_time);
	strftime(dateString, sizeof(dateString), "%D", mltime_info);

	mloopData.ydays = mltime_info->tm_yday;

	printf("Date: %s.\n", dateString);
	printf("Year:%d - Month:%d - Date:%d. \n\n", mltime_info->tm_year - 100, mltime_info->tm_mon + 1, mltime_info->tm_yday);

	// 2- set tec/dac/adc/distance
	/* turn Laser off & set DAC CH0/1 lowest */
	gpioWrite(TEMP_ENAB, PI_HIGH);		// disable Tec with high
	gpioWrite(LASER_DETECT_EN, PI_LOW); // disable Gas Laser with low
	tspi_mcp4822(1, 2, 0.001, -1.0);	// first set to DC (ch1), -1.0 indecates no pre set
	gpioDelay(1000);					// gpioSleep(0, 1, 0);
	tspi_mcp4822(0, 2, 0.4, -1.0);	    // second set to mod (ch0), 0.4 default setting
	gpioDelay(2000);					// gpioSleep(0, 1, 0);

	/* set Tec/DAC_CH0, TODO do it later, as well as set DAC_CH1 */
	gpioWrite(TEMP_ENAB, PI_LOW);

	// get temp-point and waiting temperature closer to it, less than 0.05
#if 1 // comment for debug1 TODO -remove later
	char *presult;
	float tecRet, tempPoint, voltCh0, voltCh1, conModulus;
	tempPoint = mData.tecSets.tempPoint = mloopData.tecSets.tempPoint; // TODO - use ltempPoint
	// voltCh0 = mData.dacSets.voltCh0 = mloopData.dacSets.voltCh0; // TODO - use lDAch0
	voltCh1 = mData.dacSets.voltCh1 = mloopData.dacSets.voltCh1; // TODO - use lDAch0
	conModulus = mData.adcSets.conMod = mloopData.adcSets.conMod;
	printf("   settings - %.2f, %.2f, %.2f\n", tempPoint, voltCh1, conModulus);

	/* check tec mask-bits */
	{
		char argu3[32] = "null";
		presult = tempCtrll_py(3, mtd415, mtd415SafeChk, &argu3);
		//tecRet = atof(presult);
		printf("     SafetyMask-bits is %s.\n\n", presult);
	}
	if(strcmp("255", presult)) //if mask-bits is not 255, exit app.
	{
		printf(" ---- no Tec available in Test code. \n");
		//exit(0);
	}
	/* get temp point */
	{
		char argu3[32] = "get temp point";
		char *psetVal = "-1";
		presult = tempCtrll_py(3, mtd415, mtd415setTempPoint, psetVal);
		//tecRet = atof(presult);
		//gpioDelay(2000);
	}
	//printf(" tec-return TP: %.2f\n", tecRet);
	// if(tecRet != tempPoint) = > re-set temp-point. -DOTO later
	{
		// char argu3[32] = {0}; // = "set temp point";
		////char *psetVal = "20";
		// sprintf(argu3, "%.4f", tempPoint);
		// printf(" ----- TempP %s \n", argu3); //for debug
		// presult = tempCtrll_py(3, mtd415, mtd415setTempPoint, &argu3[0]);
		// tecRet = atof(presult);
		// gpioDelay(2000);
	}

	printf("     Temperature point is %s.\n\n", presult); /*presvelop-system */
	// gpioWrite(TEMP_ENAB, PI_LOW);  // enable Tec with low

	/* waiting Tec-temperature is closer to temp-point, less than 0.05 */
	int count = 0;
	float tempRet = 0;
	// while(fabs(tecRet - tempRet) > 0.05 && count < 60)
	//{
	//	char argu3[32] = "null";
	//	//presult = "no temperature getting!";//tempCtrll_py(3, mtd415, mtd415getFunc, &argu3); //for no Tec debug
	//	presult = tempCtrll_py(3, mtd415, mtd415getTemperture, argu3);
	//	tempRet = atof(presult);
	//	//printf("     Temperature point is %s.\n\n", presult);
	//	printf("     delta Temp is (%d) %.4f, %.4f, %.4f.\n\n", count, tecRet, tempRet, fabs(tecRet - tempRet));
	//	gpioDelay(500);
	//	count++;
	// }

	int tecS, tecEb;
	tecEb = gpioRead(TEMP_ENAB);
	tecS = gpioRead(TEMP_STAT);

	/* check tec-status */
	while (tecS != 1 && count < 15)
	{
		gpioDelay(500000); // 500ms
		tecS = gpioRead(TEMP_STAT);
		count++;
	}
	printf("     Tec state is (%.4f) %d %d.\n\n", tecRet, tecS, tecEb);

	if (1 && count >= 15) // for debug
	{
		int errs = 0;
		char argu3[32] = "null";
		// presult = "no temperature getting!";//tempCtrll_py(3, mtd415, mtd415getFunc, &argu3); //for no Tec debug
		presult = tempCtrll_py(3, mtd415, mtd415getErrors, argu3);
		// errs = atoi(presult);
		// printf("     Temperature point is %s.\n\n", presult);
		printf("     Tec Error is (%.4f, %d) %s %d.\n\n", tecRet, count, presult, tecS);
	}

#endif // end of debug1

	// 2- set daily calibration flag
	if (mloopData.ydays > mloopData.dailyJust)
	{
		/* set to daily  justment */
		printf(" ydays - %d, %d", mloopData.ydays, mloopData.dailyJust);
		mloopData.dailyJust = -1;
	}

	/* update the mData */
	pthread_mutex_lock(&pmtx_mData);
	mData.dailyJust = mloopData.dailyJust; // TODO - should set correctly!
	mData.ydays = mloopData.ydays;
	mData.tecSets.tempPoint = mloopData.tecSets.tempPoint;
	// mData.dacSets.voltCh0 = mloopData.dacSets.voltCh0;
	mData.dacSets.voltCh1 = mloopData.dacSets.voltCh1;
	mData.adcSets.conMod = mloopData.adcSets.conMod;
	pthread_mutex_unlock(&pmtx_mData);

#endif // end of debugM1

	while (1)
	{
		// update time once a minute
		time(&current_time);
		time_info = localtime(&current_time);

		if (time_info->tm_min != current_min)
		{
			gdk_threads_add_idle(update_time, time_info);
			gpioDelay(10000); // delay(10);
			current_min = time_info->tm_min;
		}
		/* update LS distance. TODO - the distance has to be openned when in state of daily calibration */
		if (((time_info->tm_sec - lcurrent_sec + 60) % 60) >= LSDisTiming && (OpMode != Splash)) // update time TODO - change to update gas Concentration.
		{
			lcurrent_sec = time_info->tm_sec;

			// pthread_mutex_lock(&pmtx_mData); // nolonger used here
			dis = UART_distMain(LASERDST); // TODO - set uport selection. (muxer to distance laser)
			//dis = 5.0;
			gpioDelay(3500); // gpioDelay(2000);//delay(5);
			// pthread_mutex_unlock(&pmtx_mData);
			// printf("no distance laser used for debug.\n");

			/* check current tec-temperature */
			float tecRet;
			if (0 && (runTemp++) % 3) // remove get temperture every second
			{
				char *presult;
				char *psetVal = "null"; //"25.01"; //

				presult = tempCtrll_py(3, mtd415, mtd415getTemperture, psetVal);
				tecRet = atof(presult);
				gpioDelay(30000);
			}

			// update mData. the ppm - TODO -not used. it's probably used somehow.
			g_mutex_lock(&pmtx_mData);
			if (dis >= 0.0)
				mData.dist = dis;
			g_mutex_unlock(&pmtx_mData);
			// g_mutex_lock(&mutex_1);
			// g_mutex_unlock(&mutex_1);

			// printf("  main dis&temp - %f, %f, %d, %d\n", dis, tecRet, lcurrent_sec, LSDisTiming);
		}

		gpioDelay(5000);
		g_mutex_lock(&pmtx_mode);
		lMode = OpMode;
		g_mutex_unlock(&pmtx_mode);

		switch (lMode /*OpMode*/)
		{
			//{Splash = 0, Setup, Idle, BarGraph, PPM, LiveCam, IRCam, Shutdown}
		case Splash:
			gtk_widget_hide(ppm_display_label);
			gtk_widget_hide(eventbox_ppm);
			gtk_widget_hide(video_screen);
			gtk_widget_show(splash_screen);

			/* set Tec & DAC */
			/************************************************
			 * The RMLD is started up.
			 *  1. Turn off Tec/Laser, and set DEC_CH0 to 0.00; (TODO - DEC_CH1 would be set.)
			 *  2. Turn on the Tec, set Tec-Temp and check current Tec-Temp.
			 *  3. If the Tec-Temp reaches a certain value, set DEC_CH1;
			 *  4. Check if the daily calibration need;
			 *  4.1 if it's , do calibration process call a routine and return the new constant;
			 *  4.2 if it's not. set Tec;
			 *  DONE - all above is moved to the beginning of the thread
			 *  5. set to Idle Mode;
			 */

			mloopData.startFlag = true;
			pthread_mutex_lock(&pmtx_mData);
			mData.startFlag = mloopData.startFlag;
			pthread_mutex_unlock(&pmtx_mData);

			/* set DAC_CH0 output */
			// tspi_mcp4822(0, 2, voltCh0, 0.001); //set it in the gas-measuring thread

			if (mloopData.dailyJust < 0) // TODO - pause here for calibration???
			{
				gpioDelay(200000); // delay(1000);
				pthread_mutex_lock(&pmtx_mData);
				mloopData.dailyJust = mData.dailyJust;
				pthread_mutex_unlock(&pmtx_mData);
				printf(" ----- waiting for daily-adjust %d.\n", mloopData.dailyJust);
				break;
			} // end of the daily calibration

			/* debugging end of app */
			// OpMode = Shutdown;
			// delay(500);
			// exit(0);
			/* end of the debugging */
			/* re-set DAC ch0 & conMod value */
			pthread_mutex_lock(&pmtx_mData);
			// voltCh0 = mloopData.dacSets.voltCh0 = mData.dacSets.voltCh0;
			voltCh1 = mloopData.dacSets.voltCh1 = mData.dacSets.voltCh1;
			conModulus = mloopData.adcSets.conMod = mData.adcSets.conMod;
			pthread_mutex_unlock(&pmtx_mData);
			// printf(" ----- start system running %.4f %.4f.\n", voltCh0, conModulus);
			printf(" ----- start system running %.4f %.4f %.4f.\n", voltCh0, voltCh1, conModulus);

			gpioDelay(100000); // gpioDelay(5000);//delay(500);

			OpMode = Idle;
			// OpMode = Shutdown; //TODO - debug, remove later
			gtk_widget_hide(splash_screen);
			break;
		case Setup:
			if (LSDisTiming != 100)
			{
				LSDisTiming = 100;
				// printf("set the setup timing to 100.\n");
			}

			gpioDelay(10000); // delay(1);
			break;

		case Idle:
			if (LSDisTiming != 10)
				LSDisTiming = 10;

			gpioDelay(10000);
			gtk_widget_hide(gps_off);
			gtk_widget_hide(laser_off);
			gtk_widget_show(gps_on);
			gtk_widget_show(laser_on);

			if (status == 1)
			{
				/* Free the pipeline */
				sret = gst_element_set_state(pipeline, GST_STATE_PAUSED);

				if (sret == GST_STATE_CHANGE_FAILURE)
				{
					gst_element_set_state(pipeline, GST_STATE_NULL);
					g_printerr("Could not set pipeline to NULL. \n");
					status = 1;
				}
				else
				{
					status = 0;
					g_printerr("Called set pipeline to PAUSED. \n");
					// hide screen widget & show bkg
					gtk_widget_hide(video_screen);
					gtk_widget_show(eventbox_ppm);
					gtk_label_set_text(GTK_LABEL(left_label), (const gchar *)"Setup");
					gtk_label_set_text(GTK_LABEL(middle_label), (const gchar *)"Start");
					gtk_label_set_text(GTK_LABEL(right_label), (const gchar *)"Exit");
					gtk_label_set_text(GTK_LABEL(status_label), (const gchar *)"Idle");
				}
			}
			break;

		case PPM: // TODO - it would do someting here.
			if (LSDisTiming != 1)
				LSDisTiming = 1;
			gpioDelay(10000); // delay(10);
			// gdk_threads_add_idle(update_ppm, (measData *)ppm);
			gdk_threads_add_idle(update_meas, (gpointer)&mData); // TODO
			/* display the spot video while measuring gas */
			/* TODO: if there is any gas warning, take spots pictures (10) and then stich them */
			gpioDelay(1000);
			gdk_threads_add_idle(live_stream, (int *)pipeline); // TODO
			break;
		case LiveCam:
			gpioDelay(1000);									// delay(1);
			gdk_threads_add_idle(live_stream, (int *)pipeline); // TODO
			break;
		case IRCam:
			gpioDelay(10000); // delay(100);
			strcpy(img_filename, "./Images/Img_");
			{
				// char imgPath[64];
				// realpath(img_filename, imgPath);
				// printf("%s\n", imgPath);
			}
			if (counter < 10)
			{
				itoa((int)counter, buffer, 10);
				strcat(img_filename, buffer);
				strcat(img_filename, ".jpeg");

				g_object_get(sink, "last-sample", &from_sample, NULL);
				if (from_sample == NULL)
				{
					GST_ERROR("Error getting last sample form sink");
				}
				image_caps = gst_caps_from_string("image/jpeg");
				to_sample = gst_video_convert_sample(from_sample, image_caps, GST_CLOCK_TIME_NONE, &err);
				gst_caps_unref(image_caps);
				gst_sample_unref(from_sample);

				if (to_sample == NULL && err)
				{
					GST_ERROR("Error converting frame: %s", err->message);
					g_printerr("Error converting frame. \n");
					g_error_free(err);
				}
				buf = gst_sample_get_buffer(to_sample);
				if (gst_buffer_map(buf, &map_info, GST_MAP_READ))
				{
					if (!g_file_set_contents(img_filename, (const char *)map_info.data,
											 map_info.size, &err))
					{
						GST_CAT_LEVEL_LOG(GST_CAT_DEFAULT, GST_LEVEL_WARNING, NULL, "Could not save thumbnail: %s", err->message);
						g_error_free(err);
					}
				}
				counter++;
				gpioDelay(20000);
			}
			else
			{
				int cresult;
				counter = 0;
				// cresult = call_Python_Stitch(6, "Image_Stitching", "main", "Images", "output.jpeg","--images","--output");
				printf("The stitching is removed for test!\n");
				OpMode = Idle;
			}

			break;

		case Shutdown:
			gpioDelay(500000); // delay(500);
			exit(0);
		}
	}
}

/***************************************************
 * the dev_gasMeasure() routine is the start routine of the measer thread.
 * - arguments
 *    4 channel ADC input value (for debug)
 *    measuring value.
 *    ADC state
 *
 * **************************************************/
typedef struct ThreadDbg_t
{
	int inumData[32];
	int count[32];
	float fRatio[32];
	uint32_t u32tickDbg[32];
	float f1[32];
	float f2[32];
} thrDbg;

void *dev_gasMeasure_thread(void *arg)
{
	int lMode; // TODO - used to duplicate a local op-mode. it's used for generating random value for test right now.

	/* generate a random seed for test TODO - removed it later */
	time_t ltm;
	float randVal;
	srand((unsigned)time(&ltm));

	measData *plmData = (measData *)arg;
	userData *pcapFuncData = &adcCapFuncData; // TODO - it is first definition here?
	bool runFlag = false, isOK;
	int ldailyJust = 0;
	float rsltRatio;

	const uint32_t LSRatioTiming = 100000; // 100ms
	int timeCnt = 1, lcurrent_sec = -1, LSDisTiming = 10, max162ain = 0;
	uint32_t curTick, preTick;

	senCali dailyCal, predailyCal;
	dataCap adcCap;

	thrDbg gasDbg;
	int dbgIdx = 0, cntIn = 0, cntInV = 0, lIdx, numdataPre, numdataPos;
	int lstNumData = 100;
	float samples = 0, avgSmpls = 100, avgRtoSec = 0;

	pcapFuncData->handle = -1; // no handle enable.
	measState = measEmpty;	   // no started.
	preTick = curTick = gpioTick();

	mData.wid = wavePiset(); // not need TODO start when measState is measIdle.
	printf("waveforms are generated,\n");

	int runCont = 0, caliCout = 0, aviNum = 0;

	while (1)
	{
		/**************************************************
		 * In the while loop:
		 *  1. if measState is Idle, LS distance update once per 10Sec.
		 *  2. if measState is ppm, LS distance update once persce, and
		 *     LS Ratio value calculated almost once per100ms.
		 * ......
		 **************************************************
		 *
		 * */
		/* update time once a minute */
		// time(&current_time);
		// time_info = localtime(&current_time);

		uint32_t deltTick;
		curTick = gpioTick();
		// printf(" 0 ---- %d, %d, delt: %d\n", curTick, preTick, deltTick);//TODO - check it

		if ((deltTick = abs(curTick - preTick)) >= LSRatioTiming) // update time TODO - change to update gas Concentration.
		{
			preTick = gpioTick();
			// printf(" ---- %d, %d, delt: %d\n", curTick, preTick, deltTick);//TODO - check it

			if (measState == measReady || measState == measAdjst)
			{
				// calculate ADC intput ration, capturing adc is opened.
				int avgAdc0, avgAdc1, avgAdc2, avgAdc3; // numdataPre, numdataPos,
				pthread_mutex_lock(&pmtx_funcData);
				numdataPre = pcapFuncData->datIdx;
				pcapFuncData->isRun = 0;
				avgAdc0 = pcapFuncData->avgData[0];
				avgAdc1 = pcapFuncData->avgData[1];
				avgAdc2 = pcapFuncData->avgData[2];
				avgAdc3 = pcapFuncData->avgData[3];
				rsltRatio = getLSRatio(pcapFuncData);
				pcapFuncData->datIdx = 0;
				pcapFuncData->isRun = 1;
				pthread_mutex_unlock(&pmtx_funcData);

				/* update the ratio each 100ms if the value is satisfied */
				cntIn++; samples += (float)numdataPre;
				//avgSmpls = avgSmpls + (float)numdataPre; avgSmpls /= 2;
				if (numdataPre > 95 /*avgSmpls numdataPre*/)//for test
				{
					avgRtoSec = (avgRtoSec < rsltRatio)? rsltRatio:avgRtoSec; //get maximum value
					cntInV++;
				}//else{
				//	printf("miss a sample group - (%d, %.1f, %d, %d)\n", cntInV, avgSmpls, lstNumData, numdataPre);
					//lstNumData = (lstNumData>numdataPre)?numdataPre:lstNumData;
				//}
				lstNumData = (lstNumData>numdataPre)?numdataPre:lstNumData;
				//avgSmpls = (float)numdataPre;				
				
				/* random value for test - TODO remove it in final code */
				//randVal = (float)rand();
				//randVal = randVal / RAND_MAX;
				//randVal -= 0.5;
				//randVal *= 0.01;
				// printf("  the randome out %.4f.\n\r", randVal);
			}

			timeCnt = (timeCnt + 1) % 11; // TODO for mornitoring system state, such as battery level...
		}

		/* 1 second section */
		if(!timeCnt && cntInV)
		{
			float voltIn; int ret;
			
			/* capture monitoring system states, such as power level, etc */
			ret = getMAX11612vals(1, max162ain, &voltIn); // 0 - system power; 1-gas-laser's current (I*10); 2-thermal-voltage.
			// voltIn = (float)(max162ain);
			/* TODO - warning the gas-laser current out of range */

			/* convert volt to temperature in deg-c */
			if (max162ain == 2) // convert to temperature
			{
				if (voltIn > 0)
				{

					double Tc, Tk, par1, par2;
					par1 = 3950.0;
					par2 = 298.15;
					par2 = exp(-par1 / par2) * 10000; // const

					Tk = par1 / log((7000 * 1.8 / voltIn - 7000) / par2);
					Tc = Tk - 273.15;
					voltIn = Tc;
					// printf("\n --- Temperature: %.3f %.3f", Tk, Tc);
				}
				else
				{
					printf(" -- temp-volt is wrong: %.3f\n", voltIn);
				}
			}

			max162ain = (max162ain + 1) % 3;
			
			/* gas measuering or daily adjust */
			/* obtain the average ratio & calulat PPM */
			//avgRtoSec /= cntInV; //using the maximum one.

			if (cntIn != cntInV)
			{
				printf("count value - (%d, %d, %.1f, %d, %.3f)\n", cntIn, cntInV, samples/cntIn, lstNumData, avgRtoSec);
			}
			cntIn = cntInV = 0; // TODO remove later. they are for debugging.
			samples = 0; lstNumData = 100; timeCnt = 1;

			if(measState == measReady)
			{
				// update sys-states and gas_ppm
				pthread_mutex_lock(&pmtx_mData);
				plmData->ADVoltag = voltIn; // TODO - it's not used now.
				plmData->gas_ppm = avgRtoSec; // TODO calculat gas-ppm value late. now it updates the screen
				//plmData->gas_ppm = avgRtoSec*plmData->adcSets.conMod;
				pthread_mutex_unlock(&pmtx_mData);
				avgRtoSec = 0;

			}
			else if(measState == measAdjst)
			{
				dailyCal.avgRatio = avgRtoSec; avgRtoSec = 0;
				printf("    (%d)daily cali in: cur- %.3f %.3f, pre- %.3f %.3f.\n\r", caliCout, dailyCal.avgRatio, dailyCal.dacChDC, predailyCal.avgRatio, predailyCal.dacChDC);
				isOK = gasMeasJust(&dailyCal, &predailyCal);
				printf("    daily cali out(%d): cur- %.3f %.3f, pre- %.3f %.3f.\n\r", isOK, dailyCal.avgRatio, dailyCal.dacChDC, predailyCal.avgRatio, predailyCal.dacChDC);
				
				if (isOK)
				{
					/* set new data & end the daily calibration */
					pthread_mutex_lock(&pmtx_mData);
					plmData->dailyJust = ldailyJust = plmData->ydays;
					// plmData->dacSets.voltCh0 = dailyCal.dacCh0;
					plmData->dacSets.voltCh1 = dailyCal.dacChDC;
					plmData->adcSets.conMod = SAMP_PPM / dailyCal.avgRatio;
					// ldailyJust = plmData->dailyJust = plmData->ydays; // DONE - it's set at the startup!
					// plmData.ydays = mloopData.ydays;
					pthread_mutex_unlock(&pmtx_mData);
					
					printf(" ------ get out from calib - %d", ldailyJust);
					
					/* save the new settings, DAC_CH0 and ConMuduls */
					saveSets("./settingsDataNew.txt", plmData); // TODO - measState would be changed, measEnter!?
				}
				caliCout++;
			}
		}

		//gpioDelay(5000);//TODO - test OpMode racing
		g_mutex_lock(&pmtx_mode);
		lMode = OpMode;
		g_mutex_unlock(&pmtx_mode);

		switch (lMode /*OpMode*/)
		{
			//{Splash = 0, Setup, Idle, BarGraph, PPM, LiveCam, IRCam, Shutdown}
		case Splash: // map to idle_start.
			/**********************************************************
			 *  The measurement state was set to measEnter in very beginning.
			 *  1. Turn off Tec/Laser, and set DEC_CH0 to 0.00; (TODO - DEC_CH1 would be set.)
			 *  2. Turn on the Tec, set Tec-Temp and check current Tec-Temp.
			 *  3. If the Tec-Temp reaches a certain value, set DEC_CH1;
			 *  4. Check if the daily calibration need;
			 *  4.1 if it's , do calibration process call a routine and return the new constant;
			 *  4.2 if it's not. set Tec;
			 *  5. break;
			 *
			 * *******************************************************
			 *
			 */

			/* daily calibration if need, and read the parameters from record file to preparing the gas measurement */
			/* 1-update the startFlag and dailyJust */

			pthread_mutex_lock(&pmtx_mData);
			runFlag = plmData->startFlag;
			ldailyJust = plmData->dailyJust;
			pthread_mutex_unlock(&pmtx_mData);
			// printf("%d, %d \n", runFlag, ldailyJust);

			if (!runFlag)
			{
				gpioDelay(50000); // delay(50);
				pthread_mutex_lock(&pmtx_mData);
				dailyCal.dacChDC = predailyCal.dacChDC = plmData->dacSets.voltCh1;
				dailyCal.avgRatio = predailyCal.avgRatio = 8/plmData->adcSets.conMod; // assume constans value under sample ppm = 8
				pthread_mutex_unlock(&pmtx_mData);
				measState = measEnter;
				break;
			}

			{								  /* start ADC if need */
				if (pcapFuncData->handle < 0) // TODO - should do it in main loop
				{
					/* lowest gas-laser current */
					tspi_mcp4822(0, 2, 0.4, -1.0);//set DAC_Ch0 to default stting
					gpioDelay(1000);

					gasMeasStart(); // only set the ADC, no capture.
				}

				if (ldailyJust < 0) // daily justment - TODO move to gas measuremetn thread
				{

					if (measState != measAdjst)
					{

						/* turn gas laser on */
						gpioWrite(LASER_DETECT_EN, PI_HIGH); // enable Gas Laser with high
						gpioDelay(1000);

						/* set gas laser current */
						// tspi_mcp4822(0, 2, dailyCal.dacCh0, 0.001);
						tspi_mcp4822(1, 2, dailyCal.dacChDC, 0.001);
						gpioDelay(1000);

						/* start adc capture */
						pthread_mutex_lock(&pmtx_funcData);
						pcapFuncData->datIdx = 0;
						pcapFuncData->isRun = 1;
						gpioSetISRFuncEx(ADC_DRDY, 1, 10, adcCaptureFun, pcapFuncData);
						pthread_mutex_unlock(&pmtx_funcData);

						/* start daily adjustment, initialize the daily calibration parameters */
						// dailyCal.dacChDC = predailyCal.dacChDC = plmData->dacSets.voltCh1;
						// dailyCal.avgRatio = predailyCal.avgRatio = 15/plmData->adcSets.conMod; //assume constans value
						measState = measAdjst;
						LSDisTiming = 1; // TODO - transfer to main loop to inform it has to measure distance every second. it can be set based on the dalyJust flag setting in main thread loop.
					}
					// gpioDelay(50000);//delay(50);
					// printf("  run adjustment.\n");
					// break;
				}
				else // in mormal running
				{
					/* start gas-laser if in measEnter state */
					if (measState == measEnter)
					{
						gpioWrite(LASER_DETECT_EN, PI_HIGH); // enable Gas Laser with high
						gpioDelay(1000);

						/* set gas laser current */
						// tspi_mcp4822(0, 2, dailyCal.dacCh0, 0.001);
						tspi_mcp4822(1, 2, dailyCal.dacChDC, 0.001);
						gpioDelay(1000);

						/* TODO remove it. stop/start adc capture */
						// pthread_mutex_lock(&pmtx_funcData);
						// pcapFuncData->datIdx = 0;
						// pcapFuncData->isRun = 0;//pcapFuncData->isRun = 1;
						// gpioSetISRFuncEx(ADC_DRDY, 1, 10, NULL, pcapFuncData);//gpioSetISRFuncEx(ADC_DRDY, 1, 10, adcCaptureFun, pcapFuncData);
						// pthread_mutex_unlock(&pmtx_funcData);
						printf("\n StateID (Splash-%d), system in normal start. turn on Tec only.\n", OpMode);
						// printf(" --- DAC_CH0 is set to %.4f %.4f\n", dailyCal.dacCh0, predailyCal.dacCh0);
						printf(" --- DAC_CH1 is set to %.4f %.4f\n", dailyCal.dacChDC, predailyCal.dacChDC);
					}
					/* start gas-laser after daily calibration */
					else if (measState == measAdjst)
					{
						/* close adc capture */
						pthread_mutex_lock(&pmtx_funcData);
						pcapFuncData->datIdx = 0;
						pcapFuncData->isRun = 0;							   // pcapFuncData->isRun = 1;
						gpioSetISRFuncEx(ADC_DRDY, 1, 10, NULL, pcapFuncData); // gpioSetISRFuncEx(ADC_DRDY, 1, 10, adcCaptureFun, pcapFuncData);
						pthread_mutex_unlock(&pmtx_funcData);
						measState = measEnter;
						printf("\n StateID (Splash-%d), daily calibration done.\n", OpMode);
						// printf(" --- Stop the ADCapture. DAC_CH0 is set to %.4f %.4f\n", dailyCal.dacCh0, predailyCal.dacCh0);
						printf(" --- Stop the ADCapture. DAC_CH1 is set to %.4f %.4f\n", dailyCal.dacChDC, predailyCal.dacChDC);
						gpioDelay(50000);
					}
					else
					{
						printf(" --- start run with error measState %d\n", measState);
					}
				}
			}
			// printf("\n (Splash-%d), daily calibration done - %d. Meas-state - %d.\n", OpMode, ldailyJust, measState);

			gpioDelay(200000); // delay(1000);
			break;
		case Setup: // map to set_device. TODO - it is a new routine.
			/**********************************************************
			 *  The measurement state was set to measIdle or measSet in previous running.
			 *  1. turn off Tec/Laser, and set DEC_CH0 to 0.00;
			 *  2  set the measurement state to measReset;
			 *  3. set Tec/Laser/DEC_CH0/DEC_CH1 by user;
			 *  4. if need, save the new settings into record file;
			 *  5. set the measurement state to measIdle
			 *
			 * *******************************************************
			 *
			 */
			/* turn off Laser*/
			if (measState != measSet)
			{
				/* close gas LS ADC capture */
				pthread_mutex_lock(&pmtx_funcData);
				pcapFuncData->datIdx = 0;
				pcapFuncData->isRun = 0;
				gpioSetISRFuncEx(ADC_DRDY, 1, 10, NULL, pcapFuncData);
				pthread_mutex_unlock(&pmtx_funcData);
				gpioDelay(50000);

				/* set DAC_CH0 */
				float ldacCh0, ldacCh1;
				pthread_mutex_lock(&pmtx_mData);
				// ldacCh0 = plmData->dacSets.voltCh0;
				ldacCh1 = plmData->dacSets.voltCh1;
				pthread_mutex_unlock(&pmtx_mData);

				// tspi_mcp4822(0, 2, 0.001, ldacCh0); // set DAC_CH0 output to 0
				tspi_mcp4822(1, 2, 0.001, ldacCh1); // set DAC_CH1 output to 0
				gpioDelay(1000);

				/* turn laser off */
				gpioWrite(LASER_DETECT_EN, PI_LOW);

				/* stop waveforms */

				/* stop distance measure, now in the main loop - move to the main loop */

				measState = measSet;
				runCont = 0;
				printf("\n StateID (Setup-%d), set to measSet.\n", OpMode);
				printf(" --- disable the gas-laser & ADC capture. Tec is on only. \n");
			}

			/* set isSetGas flag ture */
			runCont++;
			if (!runCont % 50)
			{
				printf("\n StateID (Setup-%d), in setup.\n", OpMode);
				printf(" --- Tec is on only.\n");
				runCont = 0;
			}
			gpioDelay(200000); // delay(1000);
			break;

		case Idle: // map to run_idle.
			/**********************************************************
			 *  1. check if the gas measurement state is measIdle;
			 *  2.1. if not, report the measurement state error;
			 *  2.2. if it's measSet, start the waveforms;
			 *  2.3. if it's measIdle, goto step t5;
			 *  3. start laser distance measurement per 10sec;
			 *  4. turn on gas laser, set the measurement state to measIdle;
			 *  5. break;
			 *
			 * *******************************************************
			 *
			 */

			gpioDelay(100000);
			if (measState != measIdle)
			{
				if (measState == measSet)
				{
					/* start waveforms TOD later. now it starts at beginning of thread */
					//;

					/* turn gas laser on */
					gpioWrite(LASER_DETECT_EN, PI_HIGH); // enaable gas laser

					/* set DAC_CH0 or DAC_CH1 output */
					float ldacCh0, ldacCh1;
					pthread_mutex_lock(&pmtx_mData);
					// ldacCh0 = plmData->dacSets.voltCh0;
					ldacCh1 = plmData->dacSets.voltCh1;
					pthread_mutex_unlock(&pmtx_mData);

					// tspi_mcp4822(0, 2, ldacCh0, 0.001); // set DAC_CH0 output
					tspi_mcp4822(1, 2, ldacCh1, 0.001); // set DAC_CH1 output a working value
					gpioDelay(1000);
					// printf(" --- enable gas laser & set DAC Ch0 %.f.\n", ldacCh0); //TODO the ldacCh0 is updated in measSet state.
					printf(" --- enable gas laser & set DAC Ch1 %.f.\n", ldacCh1); // TODO the ldacCh1 is updated in measSet state.
				}
				else if (measState == measReady)
				{
					/* stop gas LD ADC capture */
					pthread_mutex_lock(&pmtx_funcData);
					pcapFuncData->datIdx = 0;
					pcapFuncData->isRun = 0;
					gpioSetISRFuncEx(ADC_DRDY, 1, 10, NULL, pcapFuncData);
					pthread_mutex_unlock(&pmtx_funcData);
					printf(" --- disable ADC capture.\n"); // pre state is measReady.
				}

				printf("\n StateID (Idle-%d), start waveforms, set to measIdle. \n", OpMode);
				printf(" --- run distance measurement per 10sec. pre-State %d\n", measState);

				measState = measIdle;

				/* start distance measure once per 10sec, now in the main loop */
				LSDisTiming = 10;
			}
			// else
			//	printf("\n StateID (Idle-%d), start waveforms, set to measIdle. \n", OpMode);

			//gpioDelay(100000); // delay(100);

			break;

		case PPM: // map to run_normal, TODO - communicate to gasMeasure thread.
			/**********************************************************
			 *  1. check if the gas measurement state is measSet or measReady;
			 *  2.1. if not, report the measurement state error;
			 *  2.2. if it's measSet, turn laser on & start to running ADC capture ;
			 *  3. start laser distance measurement per 10sec
			 *  4. set the measurement state to measReady
			 *
			 * *******************************************************
			 *
			 */

			gpioDelay(1000);
			if (measState != measReady)
			{
				measState = measReady;
				/* start adc capture */
				pthread_mutex_lock(&pmtx_funcData);
				pcapFuncData->datIdx = 0;
				pcapFuncData->isRun = 1;
				gpioSetISRFuncEx(ADC_DRDY, 1, 10, adcCaptureFun, pcapFuncData);
				pthread_mutex_unlock(&pmtx_funcData);
				/* start distance measure once persec, now in the main loop */
				LSDisTiming = 1;
				printf("\n StateID (PPM-%d), Turn on the Laser & run ADC input per 100ms, set to measReady. \n", OpMode);
				printf("  run distance measurement perSec.\n");
			}

			// printf("\n StateID (PPM-%d), Turn on the Laser & run ADC input per 100ms, set to measReady. \n", OpMode);
			// printf("  run distance measurement perSec.\n");
			
			/* PPM should no delay for caputeing data TODO debug */
			//gpioDelay(100000); // delay(100);
			break;
		case LiveCam: // map to img_capture, video display done in main loop
			if (measState != measIdle)
			{
				printf("\n LiveCam with Err-measState %d, should be measIdle. \n", measState);
			}
			// printf("\n StateID (LiveCam-%d), display video, in measIdle. \n", OpMode);
			gpioDelay(2000); // gpioDelay(200000);//delay(100);

			// gdk_threads_add_idle(live_stream, (int *)pipeline);
			break;
		case IRCam: // map to img_capture. image capture done in main loop
			//if (measState != measIdle)
			if (measState != measReady)
			{
				printf("\n IRCam with Err-measState %d, should be measIdle. \n", measState);
				measState = measReady;
			}
			// printf("\n StateID (IRCam-%d), capture picture. in measIdle\n", OpMode);
			gpioDelay(2000); // gpioDelay(200000);//delay(100);
			break;

		case Shutdown: // map to idle_end
			/* set DAC to 0 & close the function -Don't need, it's closed after setting. */

			/* close gas laser */
			gpioWrite(LASER_DETECT_EN, PI_LOW);
			/* close ADC */
			gasMeasClose();
			/* close Tec */
			// gpioWrite(TEMP_ENAB, PI_HIGH); //TODO - check later
			/* stop the waveforms */
			if (plmData->wid >= 0) // stop the waveforms.
			{
				wavePistop(plmData->wid);
				printf("waveforms are closed,\n");
			}
			/* close pigpio */
			gpioTerminate();

			printf("\n StateID (Shutdown-%d), close ADC, set DAC to 0, pigpio. \n", OpMode);
			gpioDelay(500000); // delay(500);
			exit(0);
		}
	}
}

/* The application entry */
int main(int argc, char *argv[])
{

	// signal(SIGINT, cleanup);
	// signal(SIGTERM, cleanup);
	// signal(SIGHUP, cleanup);

	/* Intialize the app's modules TODO later */
	// gpio_init(); // pigpio initialization, TODO it is in gas measure thread.
	// btn_init();  // buttons set & set device's state. done in this funciton.
	// set_DAC();   // set DAC output value
	// set_TempCtrl();  // temperature controller configuration
	// init_ADC();  // initialization of ADC

	/* register buttons mornitor */
	// btn_event(); // check if any buttons event happens

	if (gpioInitialise() < 0)
		return 1;

	// signal(SIGINT, cleanup);
	// signal(SIGTERM, cleanup);
	// signal(SIGHUP, cleanup);

	/**** config GPIOs replaced by gpio_init() move to gas measure thread and btn_init() ****/
	// GPIOs for button, routines are wrapped in gpio_proc.c.
	// TODO - buttons set are moved into guiGtk_proc.c
	gpioSetMode(LEFT_BUTTON, PI_INPUT);
	gpioSetMode(MIDDLE_BUTTON, PI_INPUT);
	gpioSetMode(RIGHT_BUTTON, PI_INPUT);

	gpioSetPullUpDown(LEFT_BUTTON, PI_PUD_UP);
	gpioSetPullUpDown(MIDDLE_BUTTON, PI_PUD_UP);
	gpioSetPullUpDown(RIGHT_BUTTON, PI_PUD_UP);

	int leftset = gpioSetISRFunc(LEFT_BUTTON, FALLING_EDGE, 0 /*60000*/, left_button_pressed);
	int middleset = gpioSetISRFunc(MIDDLE_BUTTON, FALLING_EDGE, 0 /*60000*/, middle_button_pressed);
	int rightset = gpioSetISRFunc(RIGHT_BUTTON, FALLING_EDGE, 0 /*60000*/, right_button_pressed);
	printf(" set btn Func: %d, %d, %d\n", leftset, middleset, rightset); // TODO err handler

	// GPIOs for UARTs, replace by gpio_init() & set_TempCtrl()
	// TODO - gpio pins set are moved into measurer_utility.c files and wrapped in gpio_proc.c
	gpioSetMode(UART_SELEC, PI_OUTPUT); // for distance & Tec.
	gpioSetMode(TEMP_ENAB, PI_OUTPUT);	// Tec enable pin.
	gpioSetMode(TEMP_STAT, PI_INPUT);	// Tec status pin

	gpioSetMode(LASER_DETECT_EN, PI_OUTPUT); // gas laser enable pin.
	gpioWrite(LASER_DETECT_EN, PI_LOW);		 // disable gas laser

	gpioWrite(UART_SELEC, PI_HIGH); // set for Tec
	gpioWrite(TEMP_ENAB, PI_HIGH);	// disable Tec

	// GPIOs for ADC, replace by gpio_init() & init_ADC()
	// TODO - gpio pins set are moved into AD_DAC.c files and wrapped in gpio_proc.c
	gpioSetMode(ADC_SELEC, PI_OUTPUT);
	gpioSetMode(ADC_CLK_EN, PI_OUTPUT);
	gpioSetMode(ADC_REST, PI_OUTPUT);
	gpioSetMode(ADC_DRDY, PI_INPUT);

	gpioWrite(ADC_SELEC, PI_HIGH);
	gpioWrite(ADC_CLK_EN, PI_HIGH);
	gpioWrite(ADC_REST, PI_HIGH);

	// GPIOs for DAC, replace by gpio_init() & set_DAC()
	// TODO - gpio pins set are moved into AD_DAC.c files and wrapped in gpio_proc.c
	gpioSetMode(DAC_SELEC, PI_OUTPUT);
	gpioSetMode(DAC_LDAC, PI_OUTPUT);

	gpioWrite(DAC_SELEC, PI_HIGH);
	gpioWrite(DAC_LDAC, PI_HIGH);

	/* start Gtk GUI */
	gtk_init(&argc, &argv); // init Gtk
	gst_init(&argc, &argv); // init Gstreamer

	/* Create the elements, replaced by init_gts() */
	// TODO - gstream pipeline operations are moved into guiGtk_proc.c
	source = gst_element_factory_make("libcamerasrc", "Source");
	filter = gst_element_factory_make("capsfilter", "filter");
	convert = gst_element_factory_make("videoconvert", "convert");
	sink = gst_element_factory_make("xvimagesink", "sink");
	videosrc_caps = gst_caps_new_simple("video/x-raw",
										"width", G_TYPE_INT, 480, "height", G_TYPE_INT, 480, NULL);
	/* Create the empty pipeline */
	flip = gst_element_factory_make("videoflip", "flip");
	g_object_set(G_OBJECT(flip), "method", 0, NULL);

	pipeline = gst_pipeline_new("test-pipeline");
	// pipeline = gst_parse_launch("appsrc libcamerasrc ! 'video/x-raw,width=480,height=480' ! videoconvert ! autovideosink", NULL);
	if (!pipeline || !source || !filter || !flip || !convert || !sink)
	{
		g_printerr("Not all elements could be created.\n");
		return -1;
	}

	/* Build the pipeline */
	gst_bin_add_many(GST_BIN(pipeline), source, filter, flip, convert, sink, NULL);
	if (gst_element_link_many(source, filter, flip, convert, sink, NULL) != TRUE)
	{
		g_printerr("Elements could not be linked.\n");
		gst_object_unref(pipeline);
		return -1;
	}
	/* filter the pipeline*/
	g_object_set(G_OBJECT(filter), "caps", videosrc_caps, NULL);
	gst_caps_unref(videosrc_caps);

	/* create GUI, replaced by init_gtk() */
	// keep in here, some elements might to be modified. TODO.
	builder = gtk_builder_new_from_file("gpioTestImg.glade");

	window = GTK_WIDGET(gtk_builder_get_object(builder, "window"));

	g_signal_connect(window, "destroy", G_CALLBACK(on_destroy), NULL);

	gtk_builder_connect_signals(builder, NULL);

	fixed1 = GTK_WIDGET(gtk_builder_get_object(builder, "fixed1"));
	left_label = GTK_WIDGET(gtk_builder_get_object(builder, "left_label"));
	middle_label = GTK_WIDGET(gtk_builder_get_object(builder, "middle_label"));
	right_label = GTK_WIDGET(gtk_builder_get_object(builder, "right_label"));
	status_label = GTK_WIDGET(gtk_builder_get_object(builder, "status_label"));
	eventbox_label_l = GTK_WIDGET(gtk_builder_get_object(builder, "eventbox_label_l"));
	eventbox_label_m = GTK_WIDGET(gtk_builder_get_object(builder, "eventbox_label_m"));
	eventbox_label_r = GTK_WIDGET(gtk_builder_get_object(builder, "eventbox_label_r"));
	eventbox_status = GTK_WIDGET(gtk_builder_get_object(builder, "eventbox_status"));
	eventbox_ppm = GTK_WIDGET(gtk_builder_get_object(builder, "eventbox_ppm"));
	date_label = GTK_WIDGET(gtk_builder_get_object(builder, "date_label"));
	time_label = GTK_WIDGET(gtk_builder_get_object(builder, "time_label"));
	ppm_display_label = GTK_WIDGET(gtk_builder_get_object(builder, "ppm_display_label"));
	video_screen = GTK_WIDGET(gtk_builder_get_object(builder, "video_screen"));
	splash_screen = GTK_WIDGET(gtk_builder_get_object(builder, "splash_screen"));
	bat_label = GTK_WIDGET(gtk_builder_get_object(builder, "bat_label"));
	laser_on = GTK_WIDGET(gtk_builder_get_object(builder, "laser_on"));
	laser_off = GTK_WIDGET(gtk_builder_get_object(builder, "laser_off"));
	gps_on = GTK_WIDGET(gtk_builder_get_object(builder, "gps_on"));
	gps_off = GTK_WIDGET(gtk_builder_get_object(builder, "gps_off"));
	info_laser = GTK_WIDGET(gtk_builder_get_object(builder, "info_laser"));
	info_bat = GTK_WIDGET(gtk_builder_get_object(builder, "info_bat"));
	info_gps = GTK_WIDGET(gtk_builder_get_object(builder, "info_gps"));
	setup_fields_labels = GTK_WIDGET(gtk_builder_get_object(builder, "setup_fields_labels"));
	setup_fields_values = GTK_WIDGET(gtk_builder_get_object(builder, "setup_fields_values"));
	setup_label_1 = GTK_WIDGET(gtk_builder_get_object(builder, "setup_label_1"));
	setup_label_2 = GTK_WIDGET(gtk_builder_get_object(builder, "setup_label_2"));
	setup_label_3 = GTK_WIDGET(gtk_builder_get_object(builder, "setup_label_3"));
	setup_label_4 = GTK_WIDGET(gtk_builder_get_object(builder, "setup_label_4"));
	setup_label_5 = GTK_WIDGET(gtk_builder_get_object(builder, "setup_label_5"));
	setup_label_6 = GTK_WIDGET(gtk_builder_get_object(builder, "setup_label_6"));
	setup_label_7 = GTK_WIDGET(gtk_builder_get_object(builder, "setup_label_7"));
	setup_value_1 = GTK_TEXT_VIEW(gtk_builder_get_object(builder, "setup_value_1"));
	setup_value_2 = GTK_TEXT_VIEW(gtk_builder_get_object(builder, "setup_value_2"));
	setup_value_3 = GTK_TEXT_VIEW(gtk_builder_get_object(builder, "setup_value_3"));
	setup_value_4 = GTK_TEXT_VIEW(gtk_builder_get_object(builder, "setup_value_4"));
	setup_value_5 = GTK_TEXT_VIEW(gtk_builder_get_object(builder, "setup_value_5"));
	setup_value_6 = GTK_TEXT_VIEW(gtk_builder_get_object(builder, "setup_value_6"));
	setup_value_7 = GTK_TEXT_VIEW(gtk_builder_get_object(builder, "setup_value_7"));

	GdkColor navy;
	navy.red = 0x1919;
	navy.green = 0x7474;
	navy.blue = 0xd2d2;

	GdkColor black;
	black.red = 0x0;
	black.green = 0x0;
	black.blue = 0x0;

	GdkColor white;
	white.red = 0xffff;
	white.green = 0xffff;
	white.blue = 0xffff;

	// res = gdk_rgba_parse (&color, "rgba(25, 116, 210,1)");
	gtk_widget_modify_bg(eventbox_label_l, GTK_STATE_NORMAL, &navy);
	gtk_widget_modify_bg(eventbox_label_m, GTK_STATE_NORMAL, &navy);
	gtk_widget_modify_bg(eventbox_label_r, GTK_STATE_NORMAL, &navy);
	gtk_widget_modify_bg(eventbox_status, GTK_STATE_NORMAL, &black);
	gtk_widget_modify_bg(eventbox_ppm, GTK_STATE_NORMAL, &white);
	gtk_widget_modify_bg(video_screen, GTK_STATE_NORMAL, &white);
	gtk_widget_modify_bg(window, GTK_STATE_NORMAL, &white);
	gtk_widget_hide(gps_on);
	gtk_widget_hide(laser_on);
	gtk_widget_show(gps_off);
	gtk_widget_show(laser_off);

	/* set application default/start up state. TODO - check/modification */
	OpMode = Splash;
	mData.startFlag = false;
	/* set the window position */
	gint x, y;
	x = 0;
	y = 1130;
	gtk_window_set_position(GTK_WINDOW(window), GDK_GRAVITY_NORTH_WEST);
	gtk_window_move(GTK_WINDOW(window), x, y);
	// gtk_window_get_position(GTK_WINDOW(window), &x, &y);
	// gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_CENTER_ALWAYS);

	/* end the position set */
	gtk_widget_show(window);

	video_window_xwindow = gtk_widget_get_window(video_screen);
	embed_xid = GDK_WINDOW_XID(video_window_xwindow);
	gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(sink), embed_xid);

	/* threads id, new code has dev_position(), dev_gasMeasure()*/
	pthread_t tidMainLoop, tidMeasure; // -TODO: =>tidMainLoop, tidMeasure, tidPosition for new code

	// initialize thread mutex, pmtx_mode

	if (pthread_mutex_init(&pmtx_mode, NULL) != 0)
	{
		printf("\n pmtx_mode init has failed\n");
		return EXIT_FAILURE;
	}

	if (pthread_mutex_init(&pmtx_mData, NULL) != 0)
	{
		printf("\n pmtx_mData init has failed\n");
		return EXIT_FAILURE;
	}

	if (pthread_mutex_init(&pmtx_funcData, NULL) != 0)
	{
		printf("\n pmtx_funcData init has failed\n");
		return EXIT_FAILURE;
	}

	/* create threads */
	pthread_create(&tidMainLoop, NULL, start_loop_thread, pipeline); // TODO - it will be dev_mainloop()
	/* TODO - add the other thread and some handler, dev_gasMeasure() dev_position() */

	pthread_create(&tidMeasure, NULL, dev_gasMeasure_thread, &mData); // TODO - it will be dev_mainloop()

	/* wait for threads (warning, they should never terminate) */
	// pthread_join(tidMainLoop, NULL);
	// pthread_join(tidMeasure, NULL);
	//   setup_filestructure();

	gtk_main();
	pthread_mutex_destroy(&pmtx_mData);

	/* wait for threads (warning, they should never terminate) */
	pthread_join(tidMainLoop, NULL);
	pthread_join(tidMeasure, NULL);

	return EXIT_SUCCESS;
}
