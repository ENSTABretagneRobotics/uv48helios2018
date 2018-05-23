#include <stdlib.h>
#include <gtk/gtk.h>
#include <cairo.h>
#include <unistd.h>
#include <math.h>


#define AUTOPILOT 0
#define MANUAL 1
#define UFREQUENCY 5

#define M_PI 3.14159265358979323846264338327
#define xPixelMin 0
#define xLambertMin 154000.00
#define xPixelMax 5000
#define xLambertMax 159000.00
#define yPixelMin 0
#define yLambertMin 6831000.00
#define yPixelMax 4000
#define yLambertMax 6827000.00

// Constantes pour Lambert 93
#define GRS80E 0.081819191042816
#define LONG_0 3
#define XS 700000
#define YS 12655612.0499
#define n 0.7256077650532670
#define C 11754255.4261

typedef struct coord_t coord_t;
typedef struct list_t list_t;
typedef struct widget2_t widget2_t;
typedef struct widget3_t widget3_t;
typedef struct lambert93_t lambert93_t;
typedef struct WGS_t WGS_t;
typedef struct WpData WpData;

struct coord_t{
    float x;  // map coord
    float y; //map coord
    float xt; // true coord
    float yt; //true coord
    int wp;
};
struct list_t{
    coord_t* array;
    int size;
};

struct widget2_t{
    list_t* w1;
    GtkWidget* w2;
};

struct widget3_t{
    list_t* w1;
    GtkWidget* w2;
    GtkWidget* w3;
};

struct WpData{
    GtkWidget* w1;
    list_t* l;
};

struct lambert93_t{
    float x;
    float y;
};

struct WGS_t{
    float lat;
    float lon;
};


void init_lst(list_t* l);
int add_lst(list_t* l, coord_t c);
int empty_lst(list_t *t);
void out_list(list_t* l);
void OnToggle(GtkWidget *pToggle, gpointer data);
void setSpeed(GtkWidget* speedLabel);
void setCourse(GtkWidget* courseLabel);
void setDepth(GtkWidget* depthLabel);
void setBattery(GtkWidget* batterycharge);
void setHorizon(GtkWidget* horizonLabel);
void setGPSCoordinate(char* coordinate, GtkWidget* targetLabel);
void setWPTarget(char* coordinate, GtkWidget* targetLabel);
void setCompletion(int step, int nn, GtkWidget* stepcnt);
void loadWP(GtkButton *button,gpointer data);
void clearWP(GtkButton *button,gpointer data);
void switchtocockpit(GtkButton *button,gpointer data);
void on_window_destroy(GtkWidget *widget, gpointer window);
//static gboolean mouse_moved(GtkWidget *widget,GdkEvent *event, gpointer user_data);
static gboolean mouse_moved(GtkWidget *widget, GdkEventMotion *event);
static gboolean button_press_event_cb (GtkWidget *widget, GdkEventButton *event, gpointer data);
static gboolean on_expose (GtkWidget *widget, GdkEventExpose *event);
static gboolean update_text (GtkWidget *label);
lambert93_t Pixels_TO_Lambert93(int x, int y);
WGS_t Lambert93_To_WGPS(lambert93_t measure);


