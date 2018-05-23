#include "Helios.h"

int ControlMode = AUTOPILOT;
float s = 0;

int main(int argc, char** argv){
    pthread_t uthread;
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
    GtkWidget* pHorizon;       // Label for Horizon
    GtkWidget* pTarget;        // Label for Target
    GtkWidget* pCompletion;    // Label for Completion
    GtkWidget* pMode;          // Label for Mode selection
    ////////////////////////////////////////////////////////
    GtkWidget *pScrollbar;      // ScrollViewPort
    ////////////////////////////////////////////////////////
    GtkWidget* pButtonCM;      // Cockpit View Enable 
    GtkWidget* pClearWP;       // Clear Waypoint
    GtkWidget* pRefreshWP;     // Update Waypoints
    GtkWidget* pToogleMode;    // Toogle control mode
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

    pSpeed      = gtk_label_new("<span face=\"Verdana\" foreground=\"#68838B\" size=\"x-large\"><u><b>Speed</b></u>   : v m/s </span>");
    gtk_label_set_use_markup(GTK_LABEL(pSpeed), TRUE);

    pCap        = gtk_label_new("<span face=\"Verdana\" foreground=\"#68838B\" size=\"x-large\"><u><b>Course</b></u>  : %.2f ° </span>");
    gtk_label_set_use_markup(GTK_LABEL(pCap), TRUE);

    pDepth      = gtk_label_new("<span face=\"Verdana\" foreground=\"#68838B\" size=\"x-large\"><u><b>Depth</b></u>   : %.2f m </span>");
    gtk_label_set_use_markup(GTK_LABEL(pDepth), TRUE);

    pCellspower = gtk_label_new("<span face=\"Verdana\" foreground=\"#68838B\" size=\"x-large\"><u><b>Battery</b></u>   : %.2f % </span>");
    gtk_label_set_use_markup(GTK_LABEL(pCellspower), TRUE);

    pHorizon    = gtk_label_new("Horizon : h m   ");
    gtk_label_set_use_markup(GTK_LABEL(pHorizon), TRUE);

    pTarget     = gtk_label_new("<span face=\"Verdana\" foreground=\"#68838B\" size=\"x-large\"><u><b>Next Waypoint :</b></u> \n %s  </span>");
    gtk_label_set_use_markup(GTK_LABEL(pTarget), TRUE);

    pCompletion = gtk_label_new("<span face=\"Verdana\" foreground=\"#68838B\" size=\"x-large\"><u><b>Mission Completion:</b></u> \n%d/%d  </span>");
    gtk_label_set_use_markup(GTK_LABEL(pCompletion), TRUE);

    pMode       = gtk_label_new("<span face=\"Verdana\" foreground=\"#ff0000\" size=\"xx-large\"><b>AUTOPILOT OFF</b></span>");
    gtk_label_set_use_markup(GTK_LABEL(pMode), TRUE);
    //*_______________________________________________________________________________________________
    //*____________________________________BUTTON_INIT________________________________________________
    pButtonCM   = gtk_button_new_with_label("Cockpit View");
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
    pFrame_R2 = gtk_frame_new("[HELIOS]_Embedded camera");

    //*--------------------> Left part
    gtk_box_pack_start(GTK_BOX(pVBox_L), pFrame_L1, FALSE, TRUE, 10);
        gtk_container_add(GTK_CONTAINER(pFrame_L1), pVBox_L1);
            gtk_box_pack_start(GTK_BOX(pVBox_L1), pPose,TRUE,TRUE,10);
            gtk_box_pack_start(GTK_BOX(pVBox_L1), pSpeed,TRUE,TRUE,10);
            gtk_box_pack_start(GTK_BOX(pVBox_L1), pCap,TRUE,TRUE,10);
            gtk_box_pack_start(GTK_BOX(pVBox_L1), pDepth,TRUE,TRUE,10);
            gtk_box_pack_start(GTK_BOX(pVBox_L1), pCellspower,TRUE,TRUE,10);
            gtk_box_pack_start(GTK_BOX(pVBox_L1), pHorizon,TRUE,TRUE,10);
            gtk_box_pack_start(GTK_BOX(pVBox_L1), pButtonCM,TRUE,TRUE,10);
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
    gtk_box_pack_start(GTK_BOX(pVBox_R), pFrame_R1, FALSE, TRUE, 10);
        gtk_container_add(GTK_CONTAINER(pFrame_R1), pVBox_R1);
            gtk_box_pack_start(GTK_BOX(pVBox_R1), pMode,TRUE,TRUE,10);
            gtk_box_pack_start(GTK_BOX(pVBox_R1), pToogleMode,TRUE,TRUE,10);
 
    gtk_box_pack_start(GTK_BOX(pVBox_R), pFrame_R2, FALSE, TRUE, 10);
        gtk_container_add(GTK_CONTAINER(pFrame_R2), pVBox_R2);
            gtk_box_pack_start(GTK_BOX(pVBox_R2), pGStreamer,FALSE,FALSE,10);

    
    //*_____________________________________ EVENTS____________________________________________________

    g_signal_connect(G_OBJECT(pWindow), "destroy", G_CALLBACK(gtk_main_quit), NULL);
    g_signal_connect(G_OBJECT(pToogleMode), "toggled", G_CALLBACK(OnToggle),pMode) ;
    WpData util1 = {text_view,&wplst};
    g_signal_connect(G_OBJECT(pRefreshWP),"clicked",G_CALLBACK(loadWP),(gpointer*)&util1);
    widget3_t util2 = {&wplst,text_view,pDraw};
    
    g_signal_connect(G_OBJECT(pClearWP),"clicked",G_CALLBACK(clearWP),&util2);
    g_signal_connect(G_OBJECT(pButtonCM ), "clicked", G_CALLBACK(switchtocockpit), pWindow);
    widget2_t util3 = {&wplst,pRefreshWP};
    g_signal_connect (G_OBJECT (pDraw), "button-press-event",G_CALLBACK(button_press_event_cb),&util3);
    

    //*_______________________________________________________________________________________________
    
    gtk_widget_show_all(pWindow);
    //*_____________________________________UPDATE____________________________________________________
    g_timeout_add (1000/UFREQUENCY, (GSourceFunc)setSpeed, pSpeed);
    g_timeout_add (1000/UFREQUENCY, (GSourceFunc)setCourse, pCap); 
    g_timeout_add (1000/UFREQUENCY, (GSourceFunc)setDepth, pDepth); 
    g_timeout_add (1000/UFREQUENCY, (GSourceFunc)setBattery, pCellspower); 
    g_timeout_add (1000/UFREQUENCY, (GSourceFunc)setHorizon, pHorizon);
    //*_______________________________________________________________________________________________
    //setSpeed(10.0,pSpeed);
    
    /*
    setCourse(120.0,pCap);
    setDepth(90.0,pDepth);
    setBattery(57,pCellspower);
    setHorizon(325.25,pHorizon);*/
    setGPSCoordinate("40° 26′ 46″ N 79° 58′ 56″ W",pPose);
    setWPTarget("40° 26′ 46″ N 79° 58′ 56″ W",pTarget);
    setCompletion(7,10,pCompletion);

    ///////////////////////////

    
    gtk_main();

    
    return EXIT_SUCCESS;
}

void OnToggle(GtkWidget *pToggle, gpointer data){
    gboolean bEtat;
    bEtat = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(pToggle));
    ControlMode = bEtat?AUTOPILOT:MANUAL;
    bEtat?gtk_label_set_text(GTK_LABEL((GtkWidget*)data),"AUTOPILOT ON"):gtk_label_set_text(GTK_LABEL((GtkWidget*)data),"AUTOPILOT OFF");
    bEtat?gtk_label_set_markup(GTK_LABEL((GtkWidget*)data), "<span face=\"Verdana\" foreground=\"#006400\" size=\"xx-large\"><b>AUTOPILOT ON</b></span>"):gtk_label_set_markup(GTK_LABEL((GtkWidget*)data), "<span face=\"Verdana\" foreground=\"#ff0000\" size=\"xx-large\"><b>AUTOPILOT OFF</b></span>");

    return;
}

void setSpeed(GtkWidget* speedLabel){
    s++;
    char* cmd = (char*)calloc(100,sizeof(char));
    sprintf(cmd,"<span face=\"Verdana\" foreground=\"#68838B\" size=\"x-large\"><u><b>Speed</b></u>   : %.2f m/s </span>",s);
    gtk_label_set_text(GTK_LABEL(speedLabel),cmd);
    gtk_label_set_use_markup(GTK_LABEL(speedLabel), TRUE);
    return;
}

void setCourse(GtkWidget* courseLabel){
    s++;
    char* cmd = (char*)calloc(100,sizeof(char));
    sprintf(cmd,"<span face=\"Verdana\" foreground=\"#68838B\" size=\"x-large\"><u><b>Course</b></u>   : %.2f ° </span>",s);
    gtk_label_set_text(GTK_LABEL(courseLabel),cmd);
    gtk_label_set_use_markup(GTK_LABEL(courseLabel), TRUE);
    return;
}

void setDepth(GtkWidget* depthLabel){
    s++;
    char* cmd = (char*)calloc(100,sizeof(char));
    sprintf(cmd,"<span face=\"Verdana\" foreground=\"#68838B\" size=\"x-large\"><u><b>Depth</b></u>   : %.2f ° </span>",s);
    gtk_label_set_text(GTK_LABEL(depthLabel),cmd);
    gtk_label_set_use_markup(GTK_LABEL(depthLabel), TRUE);
    return;
}

void setBattery(GtkWidget* batteryLabel){
    s++;
    char* cmd = (char*)calloc(100,sizeof(char));
    sprintf(cmd,"<span face=\"Verdana\" foreground=\"#68838B\" size=\"x-large\"><u><b>Battery</b></u>   : %.2f \% </span>",s);
    gtk_label_set_text(GTK_LABEL(batteryLabel),cmd);
    gtk_label_set_use_markup(GTK_LABEL(batteryLabel), TRUE);
    return;
}

void setHorizon(GtkWidget* horizonLabel){
    s++;
    char* cmd = (char*)calloc(100,sizeof(char));
    sprintf(cmd,"<span face=\"Verdana\" foreground=\"#68838B\" size=\"x-large\"><u><b>Horizon</b></u>   : %.2f m </span>",s);
    gtk_label_set_text(GTK_LABEL(horizonLabel),cmd);
    gtk_label_set_use_markup(GTK_LABEL(horizonLabel), TRUE);
    return;
}

void setGPSCoordinate(char* coordinate, GtkWidget* targetLabel){
    s++;
    char* cmd = (char*)calloc(200,sizeof(char));
    sprintf(cmd,"<span face=\"Verdana\" foreground=\"#68838B\" size=\"x-large\"><u><b>Position</b></u>   : %s  </span>",s);
    gtk_label_set_text(GTK_LABEL(targetLabel),cmd);
    gtk_label_set_use_markup(GTK_LABEL(targetLabel), TRUE);
    return;
}

void setWPTarget(char* coordinate, GtkWidget* targetLabel){
    char* cmd = (char*)calloc(300,sizeof(char));
    sprintf(cmd,"<span face=\"Verdana\" foreground=\"#68838B\" size=\"x-large\"><u><b>Next Waypoint :</b></u> \n%s  </span>",coordinate);
    gtk_label_set_text(GTK_LABEL(targetLabel),cmd);
    gtk_label_set_use_markup(GTK_LABEL(targetLabel), TRUE);
    return;   
}

void setCompletion(int step, int nn, GtkWidget* stepcnt){
    char* cmd = (char*)calloc(300,sizeof(char));
    sprintf(cmd,"<span face=\"Verdana\" foreground=\"#68838B\" size=\"x-large\"><u><b>Mission Completion:</b></u> \n%d/%d  </span>",step,nn);
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
    out_list(wplist);
    printf("\n");
    fclose(fp);
    return;
}

void clearWP(GtkButton *button,gpointer data){
    cairo_surface_t* pMap = cairo_image_surface_create_from_png ("tind.png");
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
        GtkWidget* pimg1  = gtk_image_new_from_file("indicator.jpg");
        GtkWidget* pimg2  = gtk_image_new_from_file("indicator.jpg");
        GtkWidget* pimg3  = gtk_image_new_from_file("indicator.jpg");
        GtkWidget* pimg4  = gtk_image_new_from_file("indicator.jpg");
        GtkWidget* pimg5  = gtk_image_new_from_file("indicator.jpg");
        GtkWidget* pimg6  = gtk_image_new_from_file("indicator.jpg");
        GtkWidget* pimg7  = gtk_image_new_from_file("indicator.jpg");
        GtkWidget* pimg8  = gtk_image_new_from_file("indicator.jpg");
        GtkWidget* pimg9  = gtk_image_new_from_file("indicator.jpg");
        GtkWidget* pimg10 = gtk_image_new_from_file("indicator.jpg");
        GtkWidget* pSubtable = gtk_table_new(6,2,TRUE);

        GtkWidget *window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
        GtkWidget *button = gtk_button_new_with_label("Close cockpit view");

        
        gtk_window_set_title (GTK_WINDOW (window), ">>>Cockpit view<<<");
        gtk_container_set_border_width (GTK_CONTAINER (window), 25);
        gtk_container_add(GTK_CONTAINER(window), GTK_WIDGET(pSubtable));

        
        gtk_table_attach_defaults(GTK_TABLE(pSubtable), pimg1, 0, 1, 0, 1);
        gtk_table_attach_defaults(GTK_TABLE(pSubtable), pimg2, 1, 2, 0, 1);
        gtk_table_attach_defaults(GTK_TABLE(pSubtable), pimg3, 0, 1, 1, 2);
        gtk_table_attach_defaults(GTK_TABLE(pSubtable), pimg4, 1, 2, 1, 2);
        gtk_table_attach_defaults(GTK_TABLE(pSubtable), pimg5, 0, 1, 2, 3);
        gtk_table_attach_defaults(GTK_TABLE(pSubtable), pimg6, 1, 2, 2, 3);
        gtk_table_attach_defaults(GTK_TABLE(pSubtable), pimg7, 0, 1, 3, 4);
        gtk_table_attach_defaults(GTK_TABLE(pSubtable), pimg8, 1, 2, 3, 4);
        gtk_table_attach_defaults(GTK_TABLE(pSubtable), pimg9, 0, 1, 4, 5);
        gtk_table_attach_defaults(GTK_TABLE(pSubtable), pimg10,1, 2, 4, 5);
        gtk_table_attach_defaults(GTK_TABLE(pSubtable), button,0, 2, 5, 6);

        
        

        g_signal_connect (G_OBJECT (window), "destroy", 
                          G_CALLBACK (on_window_destroy), window);
        
        g_signal_connect (G_OBJECT (button), "clicked", 
                          G_CALLBACK (on_window_destroy), window);
                                       
        
        gtk_widget_show_all (window); 
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

 cairo_surface_t* pMap = cairo_image_surface_create_from_png ("tind.png");
 
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

    double longRad = asin(sinPhiit6);
    double latRad = gamma / n + LONG_0 / 180 * M_PI;

    WGS_t result = {latRad / M_PI * 180,longRad / M_PI * 180};

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