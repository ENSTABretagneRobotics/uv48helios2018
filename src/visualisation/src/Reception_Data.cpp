#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Header.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/tf.h"
#include "tf2/LinearMath/Quaternion.h"
#include "sensor_msgs/CompressedImage.h"
#include <ros/master.h>
#include <pthread.h>

#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/core/core.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <vector>

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
void setGPSCoordinate(GtkWidget* targetLabel);
void setWPTarget(GtkWidget* targetLabel);
void setCompletion(GtkWidget* stepcnt);
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
gboolean draw_speed(GtkWidget *widget, cairo_t *cr, gpointer data);
gboolean draw_course(GtkWidget *widget, cairo_t *cr, gpointer data);
gboolean draw_M1(GtkWidget *widget, cairo_t *cr, gpointer data);
gboolean draw_M2(GtkWidget *widget, cairo_t *cr, gpointer data);
void update(GtkWidget* widget);
gboolean init_window (GtkWidget *widget, cairo_t *cr, gpointer data);
void setDir1(GtkWidget* dir1);
void setDir2(GtkWidget* dir1);
void setAlarm(GtkWidget* speedLabel);
void* lasthope(void * arg);
void update_wp(list_t* wplist);





namespace enc = sensor_msgs::image_encodings;
//using namespace cv;
using namespace std;
int ControlMode = AUTOPILOT;


struct X{
    double x;
    double y;
    double depth;
    double theta;
    double xp;
    double yp;
};

float k;
int s;

typedef struct X X_state;

double bat_pourcentage;
double moteurG;
double moteurD;
std_msgs::String etat_fsm;
X_state X = {0,0,0,0,0};
long int numero;
nav_msgs::Path waypoint;
gboolean bEtat;
bool connected;

//sensor_msgs::CompressedImage webcam;
//cv_bridge::CvImageConstPtr cv_ptr;


void batterieCallback(const std_msgs::Float64::ConstPtr& msg){
    bat_pourcentage = msg->data;
}

void fCallback(const std_msgs::Float64::ConstPtr& msg){
    X.x+= msg->data;
}
void moteurGCallback(const std_msgs::Float64::ConstPtr& msg){
    moteurG = msg->data;
}
void moteurDCallback(const std_msgs::Float64::ConstPtr& msg){
    moteurD = msg->data;
}
void etatCallback(const std_msgs::String::ConstPtr& msg){
    etat_fsm.data = msg->data;
}
void vectorCallback(const geometry_msgs::Pose::ConstPtr& msg){
    X.x = msg->position.x;
    X.y = msg->position.y;
    X.depth = msg->position.z;
    X.theta = tf::getYaw(msg->orientation);
}
void twistCallback(const geometry_msgs::Twist::ConstPtr& msg){
    X.xp = msg->linear.x;
    X.yp = msg->linear.y;
}
void numeroCallback(const std_msgs::Int64::ConstPtr& msg){
    numero = msg->data;
}
/*void webcamCallback(const sensor_msgs::ImageConstPtr& msg){
    try{
        cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
    }
}*/


int main(int argc, char **argv){
////////////////////////////////////////////////////////
    GtkWidget* pWindow;   // main window
    GtkWidget* pPopup;    // cockpit mode
    ////////////////////////////////////////////////////////
    GtkWidget* pVBox_L;   // box verticale gauche
    GtkWidget* pVBox_L1;  // box verticale gauche 1
    GtkWidget* pVBox_L2;  // box verticale gauche 2
    GtkWidget* pVBox_L31; // box verticale gauche 3.1
    GtkWidget* pVBox_M;   // box verticale milieu
    GtkWidget* pVBox_M1;   // box verticale milieu 1
    GtkWidget* pVBox_R;   // box verticale droite
    GtkWidget* pVBox_R1;   // box verticale droite 1
    GtkWidget* pVBox_R2;   // box verticale droite 2
    ////////////////////////////////////////////////////////
    GtkWidget* pFrame_L1; // Frame gauche 1 
    GtkWidget* pFrame_L2; // Frame gauche 2
    GtkWidget* pFrame_L3; // Frame gauche 3  
    GtkWidget* pFrame_M;  // Frame milieu
    GtkWidget* pFrame_R1; // Frame droit 1 
    GtkWidget* pFrame_R2; // Frame droit 2 
    ////////////////////////////////////////////////////////
    GtkWidget* pPose;          // Label for pose estimation
    GtkWidget* pSpeed;         // Label for Speed
    GtkWidget* pCap;           // Label for Course 
    GtkWidget* pDepth;         // Label for Depth
    GtkWidget* pCellspower;    // Label for CellsPower
    GtkWidget* pTarget;        // Label for Target
    GtkWidget* pCompletion;    // Label for Completion
    GtkWidget* pMode;          // Label for Mode selection
    GtkWidget* pReverse1;      // Label for motor direction 1
    GtkWidget* pReverse2;      // Label for motor direction 2
    GtkWidget* pAlarm;         // Alarm Label
    ////////////////////////////////////////////////////////
    GtkWidget *pScrollbar;      // ScrollViewPort
    ////////////////////////////////////////////////////////
    GtkWidget* pClearWP;       // Clear Waypoint
    GtkWidget* pRefreshWP;     // Update Waypoints
    GtkWidget* pToogleMode;    // Toogle control mode
    GtkWidget* instruments[4]; // instruments
    ////////////////////////////////////////////////////////
    GtkWidget* pGStreamer;     // GstreamerView
    GtkWidget* pDraw;          // Drawing area
    list_t wplst; 
    ///////////////////////////////////////////////////////
    GtkWidget* pTable;         // Layout table pointer
    //////////////////////////////////////////////////////
    GtkWidget* text_view;
    FILE* fp;
    fp = fopen("Waypoints.txt","w");
    fclose(fp);
    //////////////////////////////////////////////////////
    gtk_init(&argc, &argv);
    
    //*________________________________MAIN WINDOW SETTINGS__________________________________________
    pWindow = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_default_size(GTK_WINDOW(pWindow), 800, 500);
    gtk_window_set_title(GTK_WINDOW(pWindow), "Centre de commandement Helios");
    gtk_window_set_resizable(GTK_WINDOW(pWindow), FALSE);
    gtk_window_set_decorated(GTK_WINDOW(pWindow), TRUE);
    gtk_window_activate_focus(GTK_WINDOW(pWindow));
    gtk_widget_set_events(pWindow, GDK_POINTER_MOTION_MASK);
    //*_______________________________________________________________________________________________
    //*____________________________________LABEL INIT_________________________________________________
    
    pPose       = gtk_label_new("<span face=\"Verdana\" foreground=\"#68838B\" size=\"x-large\"><u><b>Position</b></u>   : %s  </span>");
    gtk_label_set_use_markup(GTK_LABEL(pPose), TRUE);

    pReverse1       = gtk_label_new("<span face=\"Verdana\" foreground=\"#68838B\" size=\"x-large\"><u><b>Moteur G :</b></u>   : Marche AR </span>");
    gtk_label_set_use_markup(GTK_LABEL(pReverse1), TRUE);

    pReverse2       = gtk_label_new("<span face=\"Verdana\" foreground=\"#68838B\" size=\"x-large\"><u><b>Moteur D :</b></u>   : Marche AR </span>");
    gtk_label_set_use_markup(GTK_LABEL(pReverse2), TRUE);

    pSpeed      = gtk_label_new("<span face=\"Verdana\" foreground=\"#68838B\" size=\"x-large\"><u><b>Speed</b></u>   : v m/s </span>");
    gtk_label_set_use_markup(GTK_LABEL(pSpeed), TRUE);

    pCap        = gtk_label_new("<span face=\"Verdana\" foreground=\"#68838B\" size=\"x-large\"><u><b>Course</b></u>  : %.2f ° </span>");
    gtk_label_set_use_markup(GTK_LABEL(pCap), TRUE);

    pDepth      = gtk_label_new("<span face=\"Verdana\" foreground=\"#68838B\" size=\"x-large\"><u><b>Depth</b></u>   : %.2f m </span>");
    gtk_label_set_use_markup(GTK_LABEL(pDepth), TRUE);

    pCellspower = gtk_label_new("<span face=\"Verdana\" foreground=\"#68838B\" size=\"x-large\"><u><b>Battery</b></u>   : %.2f % </span>");
    gtk_label_set_use_markup(GTK_LABEL(pCellspower), TRUE);

    pTarget     = gtk_label_new("<span face=\"Verdana\" foreground=\"#68838B\" size=\"x-large\"><u><b>Next Waypoint :</b></u> \n %s  </span>");
    gtk_label_set_use_markup(GTK_LABEL(pTarget), TRUE);

    pCompletion = gtk_label_new("<span face=\"Verdana\" foreground=\"#68838B\" size=\"x-large\"><u><b>Mission Completion:</b></u> \n%d/%d  </span>");
    gtk_label_set_use_markup(GTK_LABEL(pCompletion), TRUE);

    pMode       = gtk_label_new("<span face=\"Verdana\" foreground=\"#ff0000\" size=\"xx-large\"><b>AUTOPILOT OFF</b></span>");
    gtk_label_set_use_markup(GTK_LABEL(pMode), TRUE);


    pAlarm       = gtk_label_new("<span face=\"Verdana\" foreground=\"#00ff00\" size=\"xx-large\"><b>LINK DETECTED</b></span>");
    gtk_label_set_use_markup(GTK_LABEL(pAlarm), TRUE);
    //*_______________________________________________________________________________________________
    //*____________________________________BUTTON_INIT________________________________________________
    pClearWP    = gtk_button_new_with_label("Clear Waypoints");
    pRefreshWP  = gtk_button_new_with_label("Update Waypoints");
    pToogleMode = gtk_toggle_button_new_with_label("Toggle Autopilot");
    //*_______________________________________________________________________________________________
    //*____________________________________IMAGES_INIT________________________________________________
    pGStreamer =  gtk_image_new_from_file("gstreamer.png");
    //*_______________________________________________________________________________________________
    //*____________________________________DRAWING AREA_______________________________________________
    pDraw      =  gtk_drawing_area_new();
    gtk_widget_set_events (pDraw, gtk_widget_get_events (pDraw)| GDK_BUTTON_PRESS_MASK| GDK_POINTER_MOTION_MASK);
    init_lst(&wplst);
    //*_____________________________________TEXT_VIEW_________________________________________________
    text_view  =  gtk_text_view_new();
    //*_______________________________________________________________________________________________

    //*______________________________________LAYOUT___________________________________________________
    pTable=gtk_table_new(1,4,FALSE);
    pVBox_L = gtk_vbox_new(FALSE, 10);
    pVBox_L1 = gtk_vbox_new(TRUE, 0);
    pVBox_L2 = gtk_vbox_new(TRUE, 0);
    pVBox_M = gtk_vbox_new(TRUE, 0);
    pVBox_M1 = gtk_vbox_new(TRUE, 0);
    pVBox_R = gtk_vbox_new(FALSE, 0);
    pVBox_R1 = gtk_vbox_new(TRUE, 0);
    pVBox_R2 = gtk_vbox_new(TRUE, 0);
    pVBox_L31 = gtk_vbox_new(FALSE,5);
    gtk_container_add(GTK_CONTAINER(pWindow), GTK_WIDGET(pTable));

    //gtk_table_attach(GTK_TABLE(pTable), pVBox_L,0, 1, 0, 1,GTK_EXPAND | GTK_FILL, GTK_EXPAND, 0, 0);
    //gtk_table_attach(GTK_TABLE(pTable), pVBox_M,1, 4, 0, 1,GTK_EXPAND | GTK_FILL, GTK_EXPAND, 0, 0);
    //gtk_table_attach(GTK_TABLE(pTable), pVBox_R,4, 5, 0, 1,GTK_EXPAND | GTK_FILL, GTK_EXPAND, 0, 0);

    gtk_table_attach_defaults(GTK_TABLE(pTable), pVBox_L,0, 1, 0, 1);
    gtk_table_attach_defaults(GTK_TABLE(pTable), pVBox_M,1, 3, 0, 1);
    gtk_table_attach_defaults(GTK_TABLE(pTable), pVBox_R,3, 4, 0, 1);

    pFrame_L1 = gtk_frame_new("[HELIOS]_Current state");
    pFrame_L2 = gtk_frame_new("[HELIOS]_Mission state");
    pFrame_L3 = gtk_frame_new("[HELIOS]_Mission overview");
    pFrame_M  = gtk_frame_new("[HELIOS]_Mission Area");
    pFrame_R1 = gtk_frame_new("[HELIOS]_Control mode");
    pFrame_R2 = gtk_frame_new("[HELIOS]_Instruments");

    //*--------------------> Left part
    gtk_box_pack_start(GTK_BOX(pVBox_L), pFrame_L1, FALSE, TRUE, 10);
        gtk_container_add(GTK_CONTAINER(pFrame_L1), pVBox_L1);
            gtk_box_pack_start(GTK_BOX(pVBox_L1), pPose,TRUE,TRUE,10);
            gtk_box_pack_start(GTK_BOX(pVBox_L1), pSpeed,TRUE,TRUE,10);
            gtk_box_pack_start(GTK_BOX(pVBox_L1), pCap,TRUE,TRUE,10);
            gtk_box_pack_start(GTK_BOX(pVBox_L1), pDepth,TRUE,TRUE,10);
            gtk_box_pack_start(GTK_BOX(pVBox_L1), pCellspower,TRUE,TRUE,10);
    gtk_box_pack_start(GTK_BOX(pVBox_L), pFrame_L2, FALSE, TRUE, 10);
        gtk_container_add(GTK_CONTAINER(pFrame_L2), pVBox_L2);
            gtk_box_pack_start(GTK_BOX(pVBox_L2), pTarget,TRUE,TRUE,0);
            gtk_box_pack_start(GTK_BOX(pVBox_L2), pCompletion,TRUE,TRUE,0);
            gtk_box_pack_start(GTK_BOX(pVBox_L2), pClearWP,TRUE,TRUE,10);
            gtk_box_pack_start(GTK_BOX(pVBox_L2), pRefreshWP,TRUE,TRUE,10);
    gtk_box_pack_start(GTK_BOX(pVBox_L), pFrame_L3, TRUE, TRUE, 10);
        pScrollbar = gtk_scrolled_window_new(NULL, NULL);
        gtk_container_add(GTK_CONTAINER(pFrame_L3), pScrollbar);
        gtk_scrolled_window_add_with_viewport(GTK_SCROLLED_WINDOW(pScrollbar), text_view);
        gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(pScrollbar), GTK_POLICY_NEVER, GTK_POLICY_ALWAYS);
    //*--------------------> Middle part
    gtk_box_pack_start(GTK_BOX(pVBox_M), pFrame_M, TRUE, TRUE, 10);
        gtk_container_add(GTK_CONTAINER(pFrame_M), pVBox_M1);
            //gtk_box_pack_start(GTK_BOX(pVBox_M1), pMap,TRUE,TRUE,10);
            gtk_box_pack_start(GTK_BOX(pVBox_M1), pDraw,TRUE,TRUE,10);
            gtk_widget_set_size_request (pDraw ,910, 730);

            
    //*--------------------> Right part
    for(int i=0;i<4;i++){
            instruments[i] = gtk_drawing_area_new();
            gtk_widget_set_size_request (instruments[i], 200, 200);            
        }

    GtkWidget* pSubtable = gtk_table_new(4,2,TRUE);    
    gtk_container_set_border_width (GTK_CONTAINER (pVBox_R2), 25);
    //gtk_container_add(GTK_CONTAINER(pVBox_R2), GTK_WIDGET(pSubtable));

        
    gtk_table_attach_defaults(GTK_TABLE(pSubtable), instruments[0], 0, 1, 0, 1);
    gtk_table_attach_defaults(GTK_TABLE(pSubtable), instruments[1], 1, 2, 0, 1);
    gtk_table_attach_defaults(GTK_TABLE(pSubtable), instruments[2], 0, 1, 1, 2);
    gtk_table_attach_defaults(GTK_TABLE(pSubtable), instruments[3], 1, 2, 1, 2);
    gtk_table_attach_defaults(GTK_TABLE(pSubtable), pReverse1, 0, 1, 2, 3);
    gtk_table_attach_defaults(GTK_TABLE(pSubtable), pReverse2, 1, 2, 2, 3);




    gtk_box_pack_start(GTK_BOX(pVBox_R), pFrame_R1, FALSE, TRUE, 10);
        gtk_container_add(GTK_CONTAINER(pFrame_R1), pVBox_R1);
            gtk_box_pack_start(GTK_BOX(pVBox_R1), pMode,TRUE,TRUE,10);
            gtk_box_pack_start(GTK_BOX(pVBox_R1), pAlarm,TRUE,TRUE,10);
            gtk_box_pack_start(GTK_BOX(pVBox_R1), pToogleMode,TRUE,TRUE,10);
 
    gtk_box_pack_start(GTK_BOX(pVBox_R), pFrame_R2, FALSE, TRUE, 10);
        gtk_container_add(GTK_CONTAINER(pFrame_R2), pSubtable);
            //gtk_box_pack_start(GTK_BOX(pVBox_R2), pSubtable,FALSE,FALSE,10);

    
    //*_____________________________________ EVENTS____________________________________________________

    g_signal_connect(G_OBJECT(pWindow), "destroy", G_CALLBACK(gtk_main_quit), NULL);
    g_signal_connect(G_OBJECT(pToogleMode), "toggled", G_CALLBACK(OnToggle),pMode) ;
    WpData util1 = {text_view,&wplst};
    g_signal_connect(G_OBJECT(pRefreshWP),"clicked",G_CALLBACK(loadWP),(gpointer*)&util1);
    widget3_t util2 = {&wplst,text_view,pDraw};
    
    g_signal_connect(G_OBJECT(pClearWP),"clicked",G_CALLBACK(clearWP),&util2);

    widget2_t util3 = {&wplst,pRefreshWP};
    g_signal_connect (G_OBJECT (pDraw), "button-press-event",G_CALLBACK(button_press_event_cb),&util3);
    g_signal_connect (G_OBJECT (pDraw), "draw",G_CALLBACK(init_window),NULL);
    
    g_signal_connect (G_OBJECT (instruments[0]), "draw",G_CALLBACK (draw_speed), NULL);
    g_signal_connect (G_OBJECT (instruments[1]), "draw",G_CALLBACK (draw_course), NULL);
    g_signal_connect (G_OBJECT (instruments[2]), "draw",G_CALLBACK (draw_M1),NULL);
    g_signal_connect (G_OBJECT (instruments[3]), "draw",G_CALLBACK (draw_M2),NULL);

    //*_______________________________________________________________________________________________
    
    gtk_widget_show_all(pWindow);
    //*_____________________________________UPDATE____________________________________________________
    g_timeout_add (1000/UFREQUENCY, (GSourceFunc)setSpeed, pSpeed);
    g_timeout_add (1000/UFREQUENCY, (GSourceFunc)setCourse, pCap); 
    g_timeout_add (1000/UFREQUENCY, (GSourceFunc)setDepth, pDepth); 
    g_timeout_add (1000/UFREQUENCY, (GSourceFunc)setBattery, pCellspower); 
    g_timeout_add (1000/UFREQUENCY, (GSourceFunc)setDir1,pReverse1);
    g_timeout_add (1000/UFREQUENCY, (GSourceFunc)setDir2,pReverse2);
    g_timeout_add (1000/UFREQUENCY, (GSourceFunc)setWPTarget,pTarget);
    g_timeout_add (1000/UFREQUENCY, (GSourceFunc)setGPSCoordinate,pPose);
    g_timeout_add (1000/UFREQUENCY, (GSourceFunc)setAlarm,pAlarm);
    g_timeout_add (1000/UFREQUENCY, (GSourceFunc)setCompletion,pCompletion);
    //*_______________________________________________________________________________________________
    //setSpeed(10.0,pSpeed);
    
    /*
    setCourse(120.0,pCap);
    setDepth(90.0,pDepth);
    setBattery(57,pCellspower);
    setHorizon(325.25,pHorizon);*/
    //setGPSCoordinate("40° 26′ 46″ N 79° 58′ 56″ W",pPose);
    //setWPTarget("40° 26′ 46″ N 79° 58′ 56″ W",pTarget);
    //setCompletion(7,10,pCompletion);

    ///////////////////////////



    pthread_t new_thread;

    if(pthread_create(&new_thread,NULL,lasthope,(void*)argv)){
        perror("launching thread failed");
        return 1;
    }
    gtk_main();
    return 0;
}

void* lasthope(void * arg){
    //Mat image;
    char** argv= (char**)arg;
    int argc = 1;
    ros::init(argc,argv, "Reception_Data");
    ros::NodeHandle nh;

    ros::Subscriber bat    = nh.subscribe("Tension_batteries", 1000, batterieCallback);
    ros::Subscriber motg   = nh.subscribe("Int_puis_mot_G", 1000, moteurGCallback);
    ros::Subscriber motd   = nh.subscribe("Int_puis_mot_D", 1000, moteurDCallback);
    ros::Subscriber vectx  = nh.subscribe("Pose_vect_X", 1000, vectorCallback);
    ros::Subscriber twist  = nh.subscribe("Twist_vect_X",1000,twistCallback);
    ros::Subscriber fl  = nh.subscribe("f",1000,fCallback);
    ros::Subscriber num    = nh.subscribe("numero", 1000, numeroCallback);//int64
    //ros::Subscriber cam    = nh.subscribe("usb_cam/image_raw/compressed",1000,webcamCallback);

    ros::Publisher etat    = nh.advertise<std_msgs::Bool>("Bool_Etat", 1000);
    ros::Publisher send_wp = nh.advertise<nav_msgs::Path>("Path_WayPoints",1000);
    connected = true;


    //image = cv_ptr->image;
    ros::Rate loop_rate(20);
    while(1){
        connected = ros::master::check();
        printf("test %d",ros::master::check());
        send_wp.publish(waypoint);
        std_msgs::Bool msg;
        msg.data = bEtat;
        etat.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    pthread_exit(NULL);
}

void OnToggle(GtkWidget *pToggle, gpointer data){
    bEtat = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(pToggle));
    ControlMode = bEtat?AUTOPILOT:MANUAL;
    bEtat?gtk_label_set_text(GTK_LABEL((GtkWidget*)data),"AUTOPILOT ON"):gtk_label_set_text(GTK_LABEL((GtkWidget*)data),"AUTOPILOT OFF");
    bEtat?gtk_label_set_markup(GTK_LABEL((GtkWidget*)data), "<span face=\"Verdana\" foreground=\"#006400\" size=\"xx-large\"><b>AUTOPILOT ON</b></span>"):gtk_label_set_markup(GTK_LABEL((GtkWidget*)data), "<span face=\"Verdana\" foreground=\"#ff0000\" size=\"xx-large\"><b>AUTOPILOT OFF</b></span>");

    return;
}

void setDir1(GtkWidget* dir1){
    char* cmd = (char*)calloc(100,sizeof(char));
    float speed = pow(pow(X.xp,2)+pow(X.yp,2),0.5);
    if(speed<0)
        sprintf(cmd,"<span face=\"Verdana\" foreground=\"#68838B\" size=\"x-large\"><u><b>Moteur G :</b></u>   : Marche AR </span>");
    else
        sprintf(cmd,"<span face=\"Verdana\" foreground=\"#68838B\" size=\"x-large\"><u><b>Moteur G :</b></u>   : Marche AV </span>");
    gtk_label_set_text(GTK_LABEL(dir1),cmd);
    gtk_label_set_use_markup(GTK_LABEL(dir1), TRUE);
}

void setDir2(GtkWidget* dir1){
    char* cmd = (char*)calloc(100,sizeof(char));
    float speed = pow(pow(X.xp,2)+pow(X.yp,2),0.5);
    if(speed<0)
        sprintf(cmd,"<span face=\"Verdana\" foreground=\"#68838B\" size=\"x-large\"><u><b>Moteur D :</b></u>   : Marche AR </span>");
    else
        sprintf(cmd,"<span face=\"Verdana\" foreground=\"#68838B\" size=\"x-large\"><u><b>Moteur D :</b></u>   : Marche AV </span>");
    gtk_label_set_text(GTK_LABEL(dir1),cmd);
    gtk_label_set_use_markup(GTK_LABEL(dir1), TRUE);
}

void setSpeed(GtkWidget* speedLabel){

    char* cmd = (char*)calloc(100,sizeof(char));
    //ROS_INFO("%f\n", s);
    sprintf(cmd,"<span face=\"Verdana\" foreground=\"#68838B\" size=\"x-large\"><u><b>Speed</b></u>   : %.2f m/s </span>",pow(pow(X.xp,2)+pow(X.y,2),0.5));
    gtk_label_set_text(GTK_LABEL(speedLabel),cmd);
    gtk_label_set_use_markup(GTK_LABEL(speedLabel), TRUE);
    return;
}

void setAlarm(GtkWidget* speedLabel){
    char* cmd = (char*)calloc(300,sizeof(char));
    printf("Connected value %d\n",connected);
    if(connected)
        sprintf(cmd,"<span face=\"Verdana\" foreground=\"#00FF00\" size=\"x-large\"><u><b>LINK DETECTED</b></u></span>");
    else   
        sprintf(cmd,"<span face=\"Verdana\" foreground=\"#FF0000\" size=\"x-large\"><u><b>LINK NOT FOUND</b></u></span>"); 
    gtk_label_set_text(GTK_LABEL(speedLabel),cmd);
    gtk_label_set_use_markup(GTK_LABEL(speedLabel), TRUE);
}

void setCourse(GtkWidget* courseLabel){
    char* cmd = (char*)calloc(100,sizeof(char));
    sprintf(cmd,"<span face=\"Verdana\" foreground=\"#68838B\" size=\"x-large\"><u><b>Course</b></u>   : %.2f ° </span>",X.theta);
    gtk_label_set_text(GTK_LABEL(courseLabel),cmd);
    gtk_label_set_use_markup(GTK_LABEL(courseLabel), TRUE);
    return;
}

void setDepth(GtkWidget* depthLabel){
    char* cmd = (char*)calloc(100,sizeof(char));
    sprintf(cmd,"<span face=\"Verdana\" foreground=\"#68838B\" size=\"x-large\"><u><b>Depth</b></u>   : %.2f m </span>",X.depth);
    gtk_label_set_text(GTK_LABEL(depthLabel),cmd);
    gtk_label_set_use_markup(GTK_LABEL(depthLabel), TRUE);
    return;
}

void setBattery(GtkWidget* batteryLabel){
    char* cmd = (char*)calloc(100,sizeof(char));
    sprintf(cmd,"<span face=\"Verdana\" foreground=\"#68838B\" size=\"x-large\"><u><b>Battery</b></u>   : %.2f \% </span>",bat_pourcentage);
    gtk_label_set_text(GTK_LABEL(batteryLabel),cmd);
    gtk_label_set_use_markup(GTK_LABEL(batteryLabel), TRUE);
    return;
}


void setGPSCoordinate(GtkWidget* targetLabel){
    char* cmd = (char*)calloc(200,sizeof(char));
    sprintf(cmd,"<span face=\"Verdana\" foreground=\"#68838B\" size=\"x-large\"><u><b>Position</b></u>   : %.2f  %.2f</span>",X.x,X.y);
    gtk_label_set_text(GTK_LABEL(targetLabel),cmd);
    gtk_label_set_use_markup(GTK_LABEL(targetLabel), TRUE);
    return;
}

void setWPTarget(GtkWidget* targetLabel){
    char* cmd = (char*)calloc(300,sizeof(char));
    if (numero+1<waypoint.poses.size()){
        sprintf(cmd,"<span face=\"Verdana\" foreground=\"#68838B\" size=\"x-large\"><u><b>Next Waypoint :</b></u> \n%.3F N %.3F E </span>",waypoint.poses[numero].pose.position.x,waypoint.poses[numero].pose.position.y);
    }else{
        sprintf(cmd,"<span face=\"Verdana\" foreground=\"#68838B\" size=\"x-large\"><u><b>Next Waypoint :</b></u> \n No more waypoint</span>");
    }
    gtk_label_set_text(GTK_LABEL(targetLabel),cmd);
    gtk_label_set_use_markup(GTK_LABEL(targetLabel), TRUE);
    return;   
}

void setCompletion(GtkWidget* stepcnt){
    char* cmd = (char*)calloc(300,sizeof(char));
    sprintf(cmd,"<span face=\"Verdana\" foreground=\"#68838B\" size=\"x-large\"><u><b>Mission Completion:</b></u> \n%d/%d  </span>",numero,waypoint.poses.size());
    gtk_label_set_text(GTK_LABEL(stepcnt),cmd);
    gtk_label_set_use_markup(GTK_LABEL(stepcnt), TRUE);
    return;
}


void loadWP(GtkButton *button,gpointer data){
    FILE *fp;
    GtkTextBuffer *buffer;
    GtkTextIter start;
    GtkTextIter end;
    gchar lecture[1024];

    WpData* d = (WpData*) data;

    GtkWidget* w = (GtkWidget*) d->w1;
    list_t* wplist = (list_t*) d->l;
    
    buffer = gtk_text_view_get_buffer(GTK_TEXT_VIEW(w));
    gtk_text_buffer_get_start_iter(buffer,&start);
    gtk_text_buffer_get_end_iter(buffer,&end);
    gtk_text_buffer_delete(buffer, &start, &end);
    fp = fopen("Waypoints.txt","rt");
    while(fgets(lecture, 1024, fp))
    {
        gtk_text_buffer_get_end_iter(buffer,&end);
        gtk_text_buffer_insert(buffer, &end, g_locale_to_utf8(lecture, -1, NULL, NULL, NULL), -1);
    }

    for(int i=0;i<wplist->size;i++){
        printf("WP %d point de (lat=%f,lon=%f)\n",i,wplist->array[i].xt,wplist->array[i].yt);
    }
    printf("\n");
    update_wp(wplist);
    printf("\n");
    fclose(fp);
    return;
}

void clearWP(GtkButton *button,gpointer data){
    cairo_surface_t* pMap = cairo_image_surface_create_from_png ("//home/maxime/workspaceRos/src/visualisation/tind.png");
    widget3_t* d = (widget3_t*) data;
    list_t* wplist = (list_t*) d->w1;
    GtkWidget* txt = (GtkWidget*)d->w2;
    GtkWidget* sc = (GtkWidget*)d->w3;
    GtkTextBuffer *buffer;
    GtkTextIter start;
    GtkTextIter end;
    buffer = gtk_text_view_get_buffer(GTK_TEXT_VIEW(txt));
    gtk_text_buffer_get_start_iter(buffer,&start);
    gtk_text_buffer_get_end_iter(buffer,&end);
    gtk_text_buffer_delete(buffer, &start, &end);
    empty_lst(wplist);
    cairo_t *cr = gdk_cairo_create (gtk_widget_get_window (sc));
    cairo_set_source_surface (cr, pMap, 0, 0);
    cairo_paint(cr);
    cairo_set_source_rgb (cr, 1, 0, 0);
    cairo_destroy(cr);
    FILE* fp;
    fp = fopen("Waypoints.txt","w");
    fclose(fp);
    return;
}

void switchtocockpit(GtkButton *b,gpointer data){

}

void on_window_destroy(GtkWidget *widget, gpointer window){
    gtk_widget_destroy(GTK_WIDGET(window));
}



static gboolean button_press_event_cb (GtkWidget *widget, GdkEventButton *event,gpointer data){
 gint x, y,xt,yt;
 FILE* fp;

 widget2_t* d = (widget2_t*)data;
 list_t* wplist = (list_t*)d->w1;
 GtkWidget* button = (GtkWidget*)d->w2;

 lambert93_t lamb;
 WGS_t w;

 cairo_surface_t* pMap = cairo_image_surface_create_from_png ("//home/maxime/workspaceRos/src/visualisation/tind.png");
 
  if (event->button == GDK_BUTTON_PRIMARY)
    {
      x = (gint)event->x;
      y = (gint)event->y;
      xt = floorf((5000*x)/910);
      yt = floorf((4000*y)/730);
      lamb = Pixels_TO_Lambert93(xt, yt);
      w = Lambert93_To_WGPS(lamb);

      coord_t newwp = {x,y,w.lat,w.lon,wplist->size+1};

      fp = fopen("Waypoints.txt","a");
      fprintf (fp, "Waypoint %d (%.3f°,%.3f°)\n",newwp.wp,w.lat,w.lon);
      fprintf (fp, "\n");
      fclose(fp);
      add_lst(wplist,newwp);
      cairo_t *cr = gdk_cairo_create (gtk_widget_get_window (widget));
      cairo_set_source_surface (cr, pMap, 0, 0);
      cairo_paint(cr);
      cairo_set_source_rgb (cr, 1, 0, 0);
      cairo_set_line_width (cr,10);
      for(int i =0;i<wplist->size;i++)
        cairo_arc(cr, wplist->array[i].x, wplist->array[i].y, 2, 0, 2 * M_PI);
      
      cairo_stroke (cr);
      cairo_destroy(cr);
      
    }

  return TRUE;
}
void init_lst(list_t* l){

    l->size = 0;
    l->array = (coord_t*)calloc(1,sizeof(coord_t));
}

int add_lst(list_t* l, coord_t c){
    l->size+=1;
    l->array = (coord_t*)realloc(l->array,l->size*sizeof(coord_t));
    l->array[l->size-1]=c;
    return 1;
}

int empty_lst(list_t *l){
    l->size=0;
    //free(l->size);
    l->array = (coord_t*)realloc(l->array,(l->size+1)*sizeof(coord_t));
}

void out_list(list_t* l){
    if(!l->size)
        return;
    for(int i =0; i<l->size;i++){
        printf("WP %d(%f,%f)\n",l->array[i].wp,l->array[i].xt,l->array[i].yt);
    }

}

lambert93_t Pixels_TO_Lambert93(int x, int y){
  lambert93_t newmeasure = {xLambertMin + x, yLambertMin - y};
  return newmeasure;
}


WGS_t Lambert93_To_WGPS(lambert93_t measure){
	double lambertE = measure.x;
    double lambertN = measure.y;
    double delX = lambertE - XS;
    double delY = lambertN - YS;
    double gamma = atan(-delX / delY);
    double R = sqrt(delX * delX + delY * delY);
    double latiso = log(C / R) / n;
    double sinPhiit0 = tanh(latiso + GRS80E * atanh(GRS80E * sin(1)));
    double sinPhiit1 = tanh(latiso + GRS80E * atanh(GRS80E * sinPhiit0));
    double sinPhiit2 = tanh(latiso + GRS80E * atanh(GRS80E * sinPhiit1));
    double sinPhiit3 = tanh(latiso + GRS80E * atanh(GRS80E * sinPhiit2));
    double sinPhiit4 = tanh(latiso + GRS80E * atanh(GRS80E * sinPhiit3));
    double sinPhiit5 = tanh(latiso + GRS80E * atanh(GRS80E * sinPhiit4));
    double sinPhiit6 = tanh(latiso + GRS80E * atanh(GRS80E * sinPhiit5));

    double latRad = asin(sinPhiit6);
    double longRad = (gamma / n + LONG_0 / 180 * M_PI);

    WGS_t result = {latRad / M_PI * 180,longRad / M_PI * 180+3};

    return result;

}

static  gboolean update_text(GtkWidget *label)
{
gchar *text;
static int i = 0;

if (GTK_IS_LABEL(label)){
    text = g_strdup_printf ("<span face=\"Verdana\" foreground=\"#68838B\" size=\"x-large\"><u><b>Speed</b></u>   : %i m/s </span>", ++i);
    gtk_label_set_text (GTK_LABEL(label), text);
    gtk_label_set_use_markup(GTK_LABEL(label), TRUE);
    g_free (text);
    return TRUE;
}

else
return FALSE;

}

gboolean draw_speed (GtkWidget *widget, cairo_t *cr, gpointer data){
  guint width, height;
  GdkRGBA color;
  GtkStyleContext* context;
  double sw,sh;
  cairo_matrix_t mat;
  double angle;

  cairo_surface_t* background = cairo_image_surface_create_from_png ("/home/maxime/workspaceRos/src/visualisation/AirSpeedIndicator_Background2.png");
  cairo_surface_t* needle = cairo_image_surface_create_from_png ("/home/maxime/workspaceRos/src/visualisation/AirSpeedNeedle.png");

  context = gtk_widget_get_style_context (widget);
  width = gtk_widget_get_allocated_width (widget);
  height = gtk_widget_get_allocated_height (widget);
  sw= cairo_image_surface_get_width(needle);
  sh= cairo_image_surface_get_height(needle);
  float speed = pow(pow(X.xp,2)+pow(X.yp,2),0.5);
  angle = (speed*0.1)*( 8*G_PI)/5/8 +G_PI;

  gtk_render_background (context, cr, 0, 0, width, height);


  //__________________________FOND_________________________
  cairo_set_source_surface (cr, background, 0, 0); // fond
  cairo_paint(cr);
  //_______________________________________________________
  
  //_______________________AIGUILLE________________________
  cairo_matrix_init_identity (&mat);
  cairo_translate(cr,100,100);
  cairo_rotate(cr,angle);
  cairo_translate(cr,-100,-100);


  cairo_set_source_surface (cr, needle,90,24);     // aiguille
  cairo_paint(cr);
  //______________________________________________________

  gtk_style_context_get_color (context,gtk_style_context_get_state (context),&color);
  gdk_cairo_set_source_rgba (cr, &color);

  gtk_widget_queue_draw(widget);
 return FALSE;
}

gboolean draw_course (GtkWidget *widget, cairo_t *cr, gpointer data){
  guint width, height;
  GdkRGBA color;
  GtkStyleContext* context;
  double sw,sh;
  cairo_matrix_t mat;
  double angle;

  angle = X.theta*( 8*G_PI)/5/8;
  cairo_surface_t* background = cairo_image_surface_create_from_png ("/home/maxime/workspaceRos/src/visualisation/HeadingIndicator_Background.png");
  cairo_surface_t* needle = cairo_image_surface_create_from_png ("/home/maxime/workspaceRos/src/visualisation/HeadingIndicator_Aircraft.png");

  context = gtk_widget_get_style_context (widget);
  width = gtk_widget_get_allocated_width (widget);
  height = gtk_widget_get_allocated_height (widget);
  sw= cairo_image_surface_get_width(needle);
  sh= cairo_image_surface_get_height(needle);


  gtk_render_background (context, cr, 0, 0, width, height);


  //__________________________FOND_________________________
  cairo_set_source_surface (cr, background, 0, 0); // fond
  cairo_paint(cr);
  //_______________________________________________________
  
  //_______________________AIGUILLE________________________
  cairo_matrix_init_identity (&mat);
  cairo_translate(cr,100,100);
  cairo_rotate(cr,angle);
  cairo_translate(cr,-100,-100);


  cairo_set_source_surface (cr, needle,48,20);     // aiguille
  cairo_paint(cr);
  //______________________________________________________

  gtk_style_context_get_color (context,gtk_style_context_get_state (context),&color);
  gdk_cairo_set_source_rgba (cr, &color);

  gtk_widget_queue_draw(widget);
 return FALSE;
}

gboolean draw_M1 (GtkWidget *widget, cairo_t *cr, gpointer data){
  guint width, height;
  GdkRGBA color;
  GtkStyleContext* context;
  double sw,sh;
  cairo_matrix_t mat;
  double angle;

  angle = moteurG*M_PI/50 + M_PI;
  cairo_surface_t* background = cairo_image_surface_create_from_png ("/home/maxime/workspaceRos/src/visualisation/Puissance_Background2.png");
  cairo_surface_t* needle = cairo_image_surface_create_from_png ("/home/maxime/workspaceRos/src/visualisation/AirSpeedNeedle.png");

  context = gtk_widget_get_style_context (widget);
  width = gtk_widget_get_allocated_width (widget);
  height = gtk_widget_get_allocated_height (widget);
  sw= cairo_image_surface_get_width(needle);
  sh= cairo_image_surface_get_height(needle);

  gtk_render_background (context, cr, 0, 0, width, height);


  //__________________________FOND_________________________
  cairo_set_source_surface (cr, background, 0, 0); // fond
  cairo_paint(cr);
  //_______________________________________________________
  
  //_______________________AIGUILLE________________________
  cairo_matrix_init_identity (&mat);
  cairo_translate(cr,100,100);
  cairo_rotate(cr,angle);
  cairo_translate(cr,-100,-100);


  cairo_set_source_surface (cr, needle,90,24);     // aiguille
  cairo_paint(cr);
  //______________________________________________________

  gtk_style_context_get_color (context,gtk_style_context_get_state (context),&color);
  gdk_cairo_set_source_rgba (cr, &color);

  gtk_widget_queue_draw(widget);
 return FALSE;
}


gboolean draw_M2 (GtkWidget *widget, cairo_t *cr, gpointer data){
  guint width, height;
  GdkRGBA color;
  GtkStyleContext* context;
  double sw,sh;
  cairo_matrix_t mat;
  double angle;

  angle = moteurD*M_PI/50 + M_PI;
  cairo_surface_t* background = cairo_image_surface_create_from_png ("/home/maxime/workspaceRos/src/visualisation/Puissance_Background2.png");
  cairo_surface_t* needle = cairo_image_surface_create_from_png ("/home/maxime/workspaceRos/src/visualisation/AirSpeedNeedle.png");

  context = gtk_widget_get_style_context (widget);
  width = gtk_widget_get_allocated_width (widget);
  height = gtk_widget_get_allocated_height (widget);
  sw= cairo_image_surface_get_width(needle);
  sh= cairo_image_surface_get_height(needle);

  gtk_render_background (context, cr, 0, 0, width, height);


  //__________________________FOND_________________________
  cairo_set_source_surface (cr, background, 0, 0); // fond
  cairo_paint(cr);
  //_______________________________________________________
  
  //_______________________AIGUILLE________________________
  cairo_matrix_init_identity (&mat);
  cairo_translate(cr,100,100);
  cairo_rotate(cr,angle);
  cairo_translate(cr,-100,-100);


  cairo_set_source_surface (cr, needle,90,24);     // aiguille
  cairo_paint(cr);
  //______________________________________________________

  gtk_style_context_get_color (context,gtk_style_context_get_state (context),&color);
  gdk_cairo_set_source_rgba (cr, &color);

  gtk_widget_queue_draw(widget);
 return FALSE;
}

gboolean init_window (GtkWidget *widget, cairo_t *cr, gpointer data){
  guint width, height;
  GdkRGBA color;
  GtkStyleContext* context;

  cairo_surface_t* background = cairo_image_surface_create_from_png ("/home/maxime/workspaceRos/src/visualisation/tind.png");


  context = gtk_widget_get_style_context (widget);
  width = gtk_widget_get_allocated_width (widget);
  height = gtk_widget_get_allocated_height (widget);


  gtk_render_background (context, cr, 0, 0, width, height);


  //__________________________FOND_________________________
  cairo_set_source_surface (cr, background, 0, 0); // fond
  cairo_paint(cr);
  //_______________________________________________________


  gtk_style_context_get_color (context,gtk_style_context_get_state (context),&color);
  gdk_cairo_set_source_rgba (cr, &color);

 return FALSE;
}



void update_wp(list_t* wplist){
    waypoint.poses.resize(wplist->size);

    for(unsigned int i=0; i < wplist->size; i++){
        waypoint.poses[i].pose.position.x = wplist->array[i].xt;
        waypoint.poses[i].pose.position.y = wplist->array[i].yt;
    }
    ROS_INFO("Update %d",waypoint.poses.size());
    /*for(int i=0;i<wplist->size;i++){
        printf("WP %d point de (lat=%f,lon=%f)\n",i,waypoint.poses[i].pose.position.x,waypoint.poses[i].pose.position.y);
    }*/

}
