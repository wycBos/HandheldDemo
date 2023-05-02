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
#include "UART_test.h"
#include "waveForm.h"
#include "ADS1x15.h"
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
#include <wiringPi.h>
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
#include "ADS1x15.h"
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

enum mState
{
	measEmpty = -1,
	measEnter = 0,
	measSet,
	measIdle,
	measReady
} measState;

const gchar *labelstring, *setup_buf;
time_t current_time;
struct tm *time_info;
int current_min = -1;
int current_sec = -1;
int ppm = 0;
int cam = 0;
long int timedate;
int counter = 0;
float dist = 0;

/* gas measure thread data */
typedef struct measThread_t
{
	int curMeasSt;
	void *measData;
} measThr;

typedef struct tecParms_t
{
	int curState;
	float tempPoint;
	float currenLimit;
	float p_gain;
	float i_gain;
	float g_gain;

} tecSettings;

typedef struct DACParms_t
{
	float voltCh0;
	float voltCh1;
} dacSettings;

typedef struct ADCParms_t
{
	int curStatr;
	void *pSettings;
} adcSettings;

typedef struct measData_t
{
	int ppm;	   // count
	float gas_ppm; // TODO - present ratio right now. it'll be changed later.
	float cur_temp;
	float ADVoltag;
	float dist;
	int wid;
	tecSettings tecSets;
	dacSettings dacSets;
	adcSettings adcSets;
} measData;

measData mData;
ads1x15_p adcMain;

GMutex mutex_1, mutex_2, mutex_3, mutex_data;
pthread_mutex_t pmtx_mData;
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
	pullUpDnControl(LEFT_BUTTON, PUD_DOWN);
	pullUpDnControl(MIDDLE_BUTTON, PUD_DOWN);
	pullUpDnControl(RIGHT_BUTTON, PUD_DOWN);
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
	delay(500);
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
			// if(mData.wid >= 0)
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
		gtk_label_set_text(GTK_LABEL(left_label), (const gchar *)"");
		gtk_label_set_text(GTK_LABEL(middle_label), (const gchar *)"");
		gtk_label_set_text(GTK_LABEL(right_label), (const gchar *)"Quit");
		gtk_label_set_text(GTK_LABEL(status_label), (const gchar *)"Survey PPM");

		/* reset SER_SEL for distance measurement */

		OpMode = PPM;

		// delay(100);
		// OpMode = PPM;
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
	delay(500);
	if (level != 0)
		return;

	labelstring = gtk_label_get_text(GTK_LABEL(middle_label));
	if (strcmp(labelstring, "Start") == 0)
	{
		gtk_label_set_text(GTK_LABEL(status_label), (const gchar *)"Startup");
		// call startup sequence here
		gtk_label_set_text(GTK_LABEL(status_label), (const gchar *)"Choose Mode");
		gtk_label_set_text(GTK_LABEL(left_label), (const gchar *)"PPM/DIST");
		gtk_label_set_text(GTK_LABEL(middle_label), (const gchar *)"Live Cam");
		gtk_label_set_text(GTK_LABEL(right_label), (const gchar *)"Bar Graph");
		// printf("middle-start\n");
	}
	else if (strcmp(labelstring, "Live Cam") == 0)
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
	delay(500);
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
		// printf("right-Quit\n");
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
	pthread_mutex_lock(&pmtx_mData);

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

	pthread_mutex_lock(&pmtx_mData);
	measData lmData = *(measData *)mData;
	pthread_mutex_unlock(&pmtx_mData);

	// g_mutex_lock(&mutex_1);

	/* update the GUI here: */
	// gtk_button_set_label(button,"label");
	// itoa((int)ppm, buffer, 10);
	// itoa(lmData.ppm, buffer, 10);
	//sprintf(buffer, "%d", lmData.ppm);
	sprintf(buffer, "%.4f", lmData.gas_ppm);
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
	char buffer[5], img_filename[32];
	int LSDisTiming = 10;
	int lcurrent_sec = -1;
	float dis = 0;
	int readin, fret;

	FILE *fh;

	while (1)
	{
		// update time once a minute
		time(&current_time);
		time_info = localtime(&current_time);
		if (time_info->tm_min != current_min)
		{
			gdk_threads_add_idle(update_time, time_info);
			delay(10);
			current_min = time_info->tm_min;
		}
		// update LS distance
		if (((time_info->tm_sec - lcurrent_sec + 60) % 60) >= LSDisTiming && (OpMode != Splash)) // update time TODO - change to update gas Concentration.
		{
			lcurrent_sec = time_info->tm_sec;

			//pthread_mutex_lock(&pmtx_mData); // nolonger used here
			dis = UART_distMain(LASERDST); // TODO - set uport selection. (muxer to distance laser)
			//pthread_mutex_unlock(&pmtx_mData);
			//printf("no distance laser used for debug.\n");

			/* check current tec-temperature */
			float tecRet;
			{
				char *presult;
				char *psetVal = "null";

				presult = tempCtrll_py(3, mtd415, mtd415getTemperture, psetVal);
				tecRet = atof(presult);
			}

			// update mData. the ppm - TODO -not used. it's probably used somehow.
			g_mutex_lock(&mutex_1);
			mData.ppm += 1;			
			if (dis >= 0.0)
				mData.dist = dis;

			g_mutex_unlock(&mutex_1);
			

			printf("  main Mea dist - %f, %f, %d, %d\n", dis, tecRet, lcurrent_sec, LSDisTiming);
		}
		switch (OpMode)
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
			 *  5. set to Idle Mode;
			 */
			/* read parameter file */
			{

				fh = fopen("./settingsData.txt", "r");
				if(fh == NULL)
				{
					printf("open file failure!\n");
					exit(EXIT_FAILURE);
				}

				{ /* get current date */
					char dateString[9];
					char timeString[9];
					time_t current_time;
					struct tm *time_info;

					time(&current_time);
					time_info = localtime(&current_time);
					strftime(dateString, sizeof(dateString), "%D", time_info);
					//strftime(timeString, sizeof(timeString), "%I:%M", time_info);

					printf("Date: %s.\n", dateString);
					printf("Year:%d - Month:%d - Date:%d. \n\n", time_info->tm_year, time_info->tm_mon, time_info->tm_yday);
				}

				char data[64];
				int retC, outN = 0;

				char parm1[16], parm2[16], parm3[16];
				printf("    reading a file.\n");

				retC = fgets(data, 64, fh);
				printf("    %s, (%d), %d.\n", data, outN, retC);
				outN++;

				while(outN < 32)
				{
					retC = fscanf(fh, "%s %s %s", parm1, parm2, parm3);
					printf("   %s, (%d), %d. \n", parm1, outN, retC);
					outN++;
					if(retC <= 0)
						break;
				}
				fret = fclose(fh);

				float ltempPoint, lDAch0;

				ltempPoint = atof(parm1);
				mData.tecSets.tempPoint = ltempPoint;

				lDAch0 = atof(parm3);
				mData.dacSets.voltCh0 = lDAch0;

 				printf("    temp-point: %.2f, DACH0: %.2f.\n", mData.tecSets.tempPoint, mData.dacSets.voltCh0);			
			}

			/* turn Laser off & set DAC CH0 lowest */
			gpioWrite(TEMP_ENAB, PI_HIGH);  // disable Tec with high
			gpioWrite(LASER_DETECT_EN, PI_LOW);   // disable Gas Laser with low
			tspi_mcp4822(0, 2, 0.001); // set DAC_CH0 output to 0
			gpioDelay(400);

			/* enable Tec, TODO - set DAC_CH1 */
			gpioWrite(TEMP_ENAB, PI_LOW);
			gpioWrite(SER_SEL, SLE_LDIS);

			// get temp-point
			#if 1 //comment for debug1
			char *presult; float tecRet, tempPoint, voltCh0;
			tempPoint = mData.tecSets.tempPoint;
			voltCh0 = mData.dacSets.voltCh0;

			{
				char *psetVal = "-1";
				presult = tempCtrll_py(3, mtd415, mtd415setTempPoint/*mtd415getTemperture*/, psetVal);
				tecRet = atof(presult);
			}
			printf(" tec-return: %.2f\n", tecRet);
			//if(tecRet != tempPointSet) = > re-set temp-point. -TODO later
			{
				char argu3[32] = {0}; //"set temp point";
				//char *psetVal = "20";
				sprintf(argu3, "%.4f", tempPoint);
				presult = tempCtrll_py(3, mtd415, mtd415setTempPoint, argu3);
				//tecRet = atof(presult);
			}
			printf("     Temperature point is %s.\n\n", presult);/*presvelop-system */

			{
				char *psetVal = "-1";
				presult = tempCtrll_py(3, mtd415, mtd415setTempPoint/*mtd415getTemperture*/, psetVal);
				tecRet = atof(presult);
			}
			printf(" tec-return2: %.2f\n", tecRet);

			gpioWrite(TEMP_ENAB, PI_LOW);  // enable Tec with low
			#endif

			/* do daily calibration if need, TODO later */

			delay(3000);

			/* turn on the gas laser and set the DAC_CH0 valtage */
			gpioWrite(LASER_DETECT_EN, PI_HIGH); // enaable gas laser

			/* set DAC_CH0 output */
			tspi_mcp4822(0, 2, voltCh0); // set DAC_CH0 output to 0

			OpMode = Idle;
			//OpMode = Shutdown; //TODO - debug, remove later
			gtk_widget_hide(splash_screen);
			break;
		case Setup:
			if (LSDisTiming != 100)
			{
				LSDisTiming = 100;
				// printf("set the setup timing to 100.\n");
			}

			delay(1);
			break;

		case Idle:
			if (LSDisTiming != 10)
				LSDisTiming = 10;

			delay(1);
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

		case PPM://TODO - it would do someting here.
			if (LSDisTiming != 1)
				LSDisTiming = 1;
			delay(10);
			// gdk_threads_add_idle(update_ppm, (measData *)ppm);
			gdk_threads_add_idle(update_meas, (gpointer)&mData);
			break;
		case LiveCam:
			delay(1);
			gdk_threads_add_idle(live_stream, (int *)pipeline);
			break;
		case IRCam:
			delay(100);
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
			delay(500);
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
thrDbg gasDbg;

void *dev_gasMeasure_thread(void *arg)
{
	const uint32_t LSRatioTiming = 100000; // 100ms

	// char buffer[5], img_filename[32];
	measData *plmData = (measData *)arg;
	float dis, rsltRatio, ratioArry[32];
	uint32_t curTick, preTick, tickDbg[32];
	int dbgIdx = 0, cntIn = 0, cntInV = 0, LSDisTiming = 10, lIdx; // 10s
	preTick = curTick = gpioTick();
	userData *pcapFuncData = &adcCapFuncData; // TODO - it is first definition here?

	pcapFuncData->handle = -1; // no handle enable.

	mData.wid = wavePiset(); // TODO start when measState is measIdle.
	printf("waveforms are generated,\n");

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
		// update time once a minute
		//time(&current_time);
		//time_info = localtime(&current_time);

		curTick = gpioTick();

		if ((curTick = abs(curTick - preTick)) >= LSRatioTiming) // update time TODO - change to update gas Concentration.
		{
			gpioDelay(1000);
			preTick = gpioTick();
			if (measState == measReady)
			{
				// calculate ADC intput ration, capturing adc is opened.
				pthread_mutex_lock(&pmtx_funcData);
				int numdataPre = pcapFuncData->datIdx;
				pcapFuncData->isRun = 0;
				rsltRatio = getLSRatio(pcapFuncData); // TODO creat routine in utility file.
				int numdataPos = pcapFuncData->datIdx;
				pcapFuncData->isRun = 1;
				pthread_mutex_unlock(&pmtx_funcData);

				/* check pcapFuncData -debugging */
				// printf(" funcData: %d, %.4f\n", pcapFuncData->datIdx, (pcapFuncData->pRslts + 4)->results1);
				cntIn++;
				if (numdataPre > 10)
				{
					gasDbg.u32tickDbg[dbgIdx] = curTick;
					plmData->gas_ppm = rsltRatio; //update the screen
					gasDbg.fRatio[dbgIdx] = rsltRatio;
					gasDbg.inumData[dbgIdx] = numdataPre;
					gasDbg.count[dbgIdx] = dbgIdx;
					lIdx = dbgIdx;
					dbgIdx = (dbgIdx + 1) % 16;
					if (0)
					{
						printf(" Ratio(%d): %d, %.4f, %d\n", cntIn, gasDbg.u32tickDbg[lIdx],
							   gasDbg.fRatio[lIdx], gasDbg.inumData[lIdx]);
					}
					cntInV++;
					//	printf("\n");
					// printf("    tick_delta: %d, %.4f\n", curTick, rslt);
				}
			}
		}
		// update LS distance, now it's only for ADS1115 ADC update!
		if (((time_info->tm_sec - current_sec + 60) % 60) >= LSDisTiming) // update time TODO - change to update gas Concentration.
		{
			current_sec = time_info->tm_sec;

			// update the measure data
			// g_mutex_lock(&mutex_1);
			pthread_mutex_lock(&pmtx_mData);//TODO - remove it. it's done in main loop.

			//plmData->ppm += 1;

			plmData->ADVoltag = ADS1115_main();
			pthread_mutex_unlock(&pmtx_mData);
			// g_mutex_unlock(&mutex_1);

			// printf("gasMea dist - %f, %d, %d\n", dis, current_sec, LSDisTiming);//never need!!!
			int idx = 0;
			//for (idx = 0; idx < 16; idx++)
			if(0)
			{
				printf(" gasData: %d, %.4f, %d\n", gasDbg.u32tickDbg[idx],
					   gasDbg.fRatio[idx], gasDbg.inumData[idx]);
			}
			printf("count value - %d, %d)\n", cntIn, cntInV);
			cntIn = cntInV = 0;
		}
		switch (OpMode)
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
			if(measState != measEnter){
				printf("\n StateID (Splash-%d), do daily calibration if need. turn on Tec only.\n", OpMode);
				printf(" read gas measurement parameters. set measState to measEnter\n");
			}
			measState = measEnter;
			/* read parameter file - Done in main loop */

			/* turn Laser off - Done in main loop */

			/* set Tec/DAC_CH0, TODO do it later, as well as set DAC_CH1 - Done in main loop */

			/* turn Tec on. TODO later, it cannot work in develop-system - Done in main loop */

			/* do daily calibration if need, TODO later - Done in main loop */

			/* start gas measuring ADC */
			if (pcapFuncData->handle < 0) // TODO - should do it in main loop
				gasMeasStart();			  // only set the ADC, no capture.

			//printf("\n StateID (Splash-%d), do daily calibration if need. turn on Tec only.\n", OpMode);
			//printf(" read gas measurement parameters. set measState to measEnter\n");
			delay(1000);
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
				measState = measSet;
				/* turn laser off and set DAC CH0 to 0 */
				gpioWrite(LASER_DETECT_EN, PI_LOW);
				tspi_mcp4822(0, 2, 0.001); // set DAC_CH0 output to 0

				/* close gas LS ADC capture */
				pthread_mutex_lock(&pmtx_funcData);
				pcapFuncData->datIdx = 0;
				pcapFuncData->isRun = 0;
				gpioSetAlertFuncEx(ADC_DRDY, NULL, pcapFuncData);
				pthread_mutex_unlock(&pmtx_funcData);

				/* stop waveforms -TODO later */

				/* stop distance measure, now in the main loop - move to the main loop */

				printf("\n StateID (Setup-%d), set to measSet.\n", OpMode);
				printf(" setup flag to inform setting measurement parameters. Tec is on only. \n", OpMode);
			}

			/* set isSetGas flag ture */
			printf("\n StateID (Setup-%d), in setup.\n", OpMode);
			printf(" setup flag to inform setting measurement parameters. Tec is on only.\n", OpMode);
			delay(1000);
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

			if (measState != measIdle)
			{
				if (measState == measSet)
				{
					/* start waveforms TOD later. now it starts at beginning of thread */
					//;

					/* turn gas laser on */
					gpioWrite(LASER_DETECT_EN, PI_HIGH); // enaable gas laser

					/* set DAC_CH0 output */
					float ldacCh0;
					pthread_mutex_lock(&pmtx_mData);
					ldacCh0 = mData.dacSets.voltCh0;
					pthread_mutex_unlock(&pmtx_mData);

					tspi_mcp4822(0, 2, ldacCh0); // set DAC_CH0 output

				}

				measState = measIdle;

				/* stop gas LD ADC capture */
				// TODO - close capture adc data
				pthread_mutex_lock(&pmtx_funcData);
				pcapFuncData->datIdx = 0;
				pcapFuncData->isRun = 0;
				gpioSetAlertFuncEx(ADC_DRDY, NULL, pcapFuncData);
				pthread_mutex_unlock(&pmtx_funcData);

				/* start distance measure once per 10sec, now in the main loop */
				LSDisTiming = 10;

				printf("\n StateID (Idle-%d), start waveforms, set to measIdle. \n", OpMode);
				printf("run distance measurement per 10sec. \n");
			}
			// printf("\n StateID (Idle-%d), start waveforms, set to measIdle. \n", OpMode);

			delay(100);

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

			if (measState != measReady)
			{
				measState = measReady;
				/* start adc capture */
				pthread_mutex_lock(&pmtx_funcData);
				pcapFuncData->datIdx = 0;
				pcapFuncData->isRun = 1;
				gpioSetAlertFuncEx(ADC_DRDY, adcCaptureFun, pcapFuncData);
				pthread_mutex_unlock(&pmtx_funcData);
				/* start distance measure once persec, now in the main loop */
				LSDisTiming = 1;
				printf("\n StateID (PPM-%d), Turn on the Laser & run ADC input per 100ms, set to measReady. \n", OpMode);
				printf("  run distance measurement perSec.\n");
			}

			// printf("\n StateID (PPM-%d), Turn on the Laser & run ADC input per 100ms, set to measReady. \n", OpMode);
			// printf("  run distance measurement perSec.\n");
			delay(100);
			break;
		case LiveCam: // map to img_capture, video display done in main loop
			if (measState != measIdle)
			{
				printf("\n LiveCam with Err-measState %d, should be measIdle. \n", measState);
			}
			//printf("\n StateID (LiveCam-%d), display video, in measIdle. \n", OpMode);
			delay(100);

			// gdk_threads_add_idle(live_stream, (int *)pipeline);
			break;
		case IRCam: // map to img_capture. image capture done in main loop
			if (measState != measIdle)
			{
				printf("\n IRCam with Err-measState %d, should be measIdle. \n", measState);
			}
			// printf("\n StateID (IRCam-%d), capture picture. in measIdle\n", OpMode);
			delay(100);

			break;

		case Shutdown: // map to idle_end
			/* set DAC to 0 & close the function -Don't need, it's closed after setting. */
			
			/* close gas laser */
			gpioWrite(LASER_DETECT_EN, PI_LOW);
			/* close ADC */
			gasMeasClose();
			/* close Tec */
			gpioWrite(TEMP_ENAB, PI_HIGH); //TODO - check later
			/* close pigpio */
			gpioTerminate();

			printf("\n StateID (Shutdown-%d), close ADC, set DAC to 0, pigpio. \n", OpMode);
			delay(100);
			exit(0);
		}
	}
}

/* The application entry */
int main(int argc, char *argv[])
{

	signal(SIGINT, cleanup);
	signal(SIGTERM, cleanup);
	signal(SIGHUP, cleanup);

	/* Intialize the app's modules TODO later */
	// gpio_init(); // pigpio initialization, TODO it is in gas measure thread.
	// btn_init();  // buttons set & set device's state. done in this funciton.
	// set_DAC();   // set DAC output value
	// set_TempCtrl();  // temperature controller configuration
	// init_ADC();  // initialization of ADC

	/* register buttons mornitor */
	// btn_event(); // check if any buttons event happens

	// mData.wid = -1;

	// mData.wid = wavePiset(); //DONE move to dev_gasMeasure_thread, pigpio is initilzed in the routine.
	if (gpioInitialise() < 0)
		return 1;
	// DONE - execute in the dev_gas_measure_thread. following the main loop

	/**** config GPIOs replaced by gpio_init() move to gas measure thread and btn_init() ****/
	// GPIOs for button, routines are wrapped in gpio_proc.c.
	// TODO - buttons set are moved into guiGtk_proc.c
	gpioSetMode(LEFT_BUTTON, PI_INPUT);
	gpioSetMode(MIDDLE_BUTTON, PI_INPUT);
	gpioSetMode(RIGHT_BUTTON, PI_INPUT);

	gpioSetPullUpDown(LEFT_BUTTON, PI_PUD_UP);
	gpioSetPullUpDown(MIDDLE_BUTTON, PI_PUD_UP);
	gpioSetPullUpDown(RIGHT_BUTTON, PI_PUD_UP);

	int leftset = gpioSetISRFunc(LEFT_BUTTON, FALLING_EDGE, 60000, left_button_pressed);
	int middleset = gpioSetISRFunc(MIDDLE_BUTTON, FALLING_EDGE, 60000, middle_button_pressed);
	int rightset = gpioSetISRFunc(RIGHT_BUTTON, FALLING_EDGE, 60000, right_button_pressed);
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
	builder = gtk_builder_new_from_file("gpioTest.glade");

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

	// initialize thread mutex,
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
	//  setup_filestructure();

	gtk_main();
	pthread_mutex_destroy(&pmtx_mData);

	/* wait for threads (warning, they should never terminate) */
	pthread_join(tidMainLoop, NULL);
	pthread_join(tidMeasure, NULL);

	return EXIT_SUCCESS;
}
