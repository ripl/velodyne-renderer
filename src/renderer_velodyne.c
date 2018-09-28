#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <glib.h>
#include <gdk/gdkkeysyms.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include <gsl/gsl_blas.h>

#include <bot_core/bot_core.h>
#include <bot_vis/viewer.h>
#include <bot_vis/gtk_util.h>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>
#include <velodyne/velodyne.h>
#include <path_utils/path_util.h>

#include "lcmtypes/senlcm_velodyne_t.h"
#include "lcmtypes/senlcm_velodyne_list_t.h"
#include "renderer_velodyne.h"

#define DTOR M_PI/180
#define RTOD 180/M_PI

#define MAX_REFRESH_RATE_USEC 30000 // about 1/30 of a second

#define RENDERER_NAME "Velodyne"
#define VELODYNE_DATA_CIRC_SIZE 10
#define POSE_DATA_CIRC_SIZE 200

#define PARAM_COLOR_MENU "Color"
#define PARAM_HISTORY_LENGTH "Scan Hist. Len."
#define PARAM_HISTORY_FREQUENCY "Scan Hist. Freq."
#define PARAM_POINT_SIZE "Point Size"
#define PARAM_POINT_ALPHA "Point Alpha"
#define GAMMA 0.1

typedef struct _rate_t {
    double current_hz;
    int64_t last_tick;
    int64_t tick_count;
} rate_t;

enum {
    COLOR_Z,
    COLOR_INTENSITY,
    COLOR_NONE,
};


typedef struct _pose_data_t pose_data_t;
struct _pose_data_t {
    double pose[6];
    double motion[6];
    int64_t utime;
};

typedef struct _RendererVelodyne RendererVelodyne;
struct _RendererVelodyne {
    BotRenderer renderer;
    //pose_t *pose;

    lcm_t *lcm;
    BotParam *param;
    BotFrames *frames;

    bot_core_pose_t *bot_pose_last;

    int64_t last_collector_utime;

    int have_data;

    velodyne_calib_t *calib;
    velodyne_laser_return_collector_t *collector;

    BotPtrCircular   *velodyne_data_circ;
    int64_t 	      last_velodyne_data_utime;
    int64_t           last_pose_utime;

    BotViewer         *viewer;
    BotGtkParamWidget *pw;

    GMutex *mutex;
};

// convenience function to get the bot's position in the local frame
static int
frames_vehicle_pos_local (BotFrames *frames, double pos[3])
{
    double pos_body[3] = {0, 0, 0};
    return bot_frames_transform_vec (frames, "body", "local", pos_body, pos);
}

static int
process_velodyne (const senlcm_velodyne_t *v, RendererVelodyne *self)
{
    g_assert(self);

    int do_push_motion = 0; // only push motion data if we are starting a new collection or there is a new pose
    double hist_spc = bot_gtk_param_widget_get_double (self->pw, PARAM_HISTORY_FREQUENCY);

    // Is this a scan packet?
    if (v->packet_type == SENLCM_VELODYNE_T_TYPE_DATA_PACKET) {

        velodyne_laser_return_collection_t *lrc =
            velodyne_decode_data_packet_old(self->calib, v->data, v->datalen, v->utime);

        int ret = velodyne_collector_push_laser_returns (self->collector, lrc);

        velodyne_free_laser_return_collection (lrc);

        if (VELODYNE_COLLECTION_READY == ret) {

            velodyne_laser_return_collection_t *lrc =
                velodyne_collector_pull_collection (self->collector);

            // if enough time has elapsed since the last scan push it onto the circular buffer
            if (abs (lrc->utime - self->last_velodyne_data_utime) > (int64_t)(1E6/hist_spc)) {
                bot_ptr_circular_add (self->velodyne_data_circ, lrc);
                self->last_velodyne_data_utime = lrc->utime;
            } else {
                // memory leak city if this isnt here as soon as you increase the history spacing
                velodyne_free_laser_return_collection (lrc);
            }

            //starting a new collection
            do_push_motion = 1;
        }
        else if(VELODYNE_COLLECTION_READY_LOW == ret) {
            fprintf(stderr,"Low packet - ignoring");

            velodyne_laser_return_collection_t *lrc =
                velodyne_collector_pull_collection (self->collector);

            velodyne_free_laser_return_collection (lrc);
        }
    }

    // Update the Velodyne's state information (pos, rpy, linear/angular velocity)
    if (do_push_motion) {

        if (!self->bot_pose_last)
            return 0;

        // push new motion onto collector
        velodyne_state_t state;

        state.utime = v->utime;

        // find sensor pose in local/world frame
        /*
         * double x_lr[6] = {self->pose->x, self->pose->y, self->pose->z,
         *                   self->pose->r, self->pose->p, self->pose->h};
         * double x_ls[6] = {0};
         * ssc_head2tail (x_ls, NULL, x_lr, self->x_vs);
         */

        BotTrans velodyne_to_local;
        bot_frames_get_trans_with_utime (self->frames, "VELODYNE", "local", v->utime, &velodyne_to_local);

        memcpy (state.xyz, velodyne_to_local.trans_vec, 3*sizeof(double));
        bot_quat_to_roll_pitch_yaw (velodyne_to_local.rot_quat, state.rph);

        // Compute translational velocity
        //
        // v_velodyne = v_bot + r x w
        BotTrans velodyne_to_body;
        bot_frames_get_trans (self->frames, "VELODYNE", "body", &velodyne_to_body);

        double v_velodyne[3];
        double r_body_to_velodyne_local[3];
        bot_quat_rotate_to (self->bot_pose_last->orientation, velodyne_to_body.trans_vec, r_body_to_velodyne_local);

        // vel_rot = r x w
        double vel_rot[3];
        bot_vector_cross_3d (r_body_to_velodyne_local, self->bot_pose_last->rotation_rate, vel_rot);

        bot_vector_add_3d (self->bot_pose_last->vel, vel_rot, state.xyz_dot);


        // Compute angular rotation rate
        memcpy (state.rph_dot, self->bot_pose_last->rotation_rate, 3*sizeof(double));

        //state.xyz[0] = x_ls[0]; state.xyz[1] = x_ls[1]; state.xyz[2] = x_ls[2];
        //state.rph[0] = x_ls[3]; state.rph[1] = x_ls[4]; state.rph[2] = x_ls[5];

        // move velocities and rates into sensor frame
        //double O_sv[9];
        //double rph_vs[3] = {self->x_vs[3], self->x_vs[4], self->x_vs[5]};
        //so3_rotxyz (O_sv, rph_vs);
        //double t_vs[3] = {self->x_vs[0], self->x_vs[1], self->x_vs[2]};
        ////uvw_sensor = O_sv * [uvw - skewsym(t_vs)*abc];

        // pose fields NEVER POPULATED
        //double abc[3] = {self->pose->a,
        //                 self->pose->b,
        //                 self->pose->c};
        //double uvw[3] = {self->pose->u,
        //                 self->pose->v,
        //                 self->pose->w};
        //double skewsym[9] = { 0,       -t_vs[2],  t_vs[1],
        //                      t_vs[2],  0,       -t_vs[0],
        //                     -t_vs[1],  t_vs[0],  0    };
        //GSLU_VECTOR_VIEW (uvw_sensor,3, {0});
        //gsl_vector_view abc_v = gsl_vector_view_array (abc, 3);
        //gsl_vector_view uvw_v = gsl_vector_view_array (uvw, 3);
        //gsl_matrix_view O_sv_v = gsl_matrix_view_array (O_sv, 3, 3);
        //gsl_matrix_view skewsym_v = gsl_matrix_view_array (skewsym, 3, 3);
        //skewsym(t_vs)*abc;
        //gslu_mv (&uvw_sensor.vector, &skewsym_v.matrix, &abc_v.vector); // uvw_sensor.vector is all zero since abc contains zeros
        //[uvw - skewsym(t_vs)*abc]
        //gsl_vector_sub (&uvw_v.vector, &uvw_sensor.vector);
        //uvw_sensor = O_sv * [uvw - skewsym(t_vs)*abc];
        //gslu_mv (&uvw_sensor.vector, &O_sv_v.matrix, &uvw_v.vector);

        // sensor frame rates
        //GSLU_VECTOR_VIEW (abc_sensor, 3, {0});
        //gsl_matrix_view O_vs_v = gsl_matrix_view_array (O_sv, 3, 3);
        //gsl_matrix_transpose (&O_vs_v.matrix);
        //gslu_mv (&abc_sensor.vector, &O_vs_v.matrix, &abc_v.vector);

        //rotate velodyne body velocities into local frame
        //double R_sl[9];
        //so3_rotxyz (R_sl, state.rph); //state.rph = rph_ls
        //gsl_matrix_view R_sl_v = gsl_matrix_view_array (R_sl, 3, 3);
        //GSLU_VECTOR_VIEW (xyz_dot_sensor,3, {0});
        //gslu_mv (&xyz_dot_sensor.vector, &R_sl_v.matrix, &uvw_sensor.vector);
        //memcpy (&(state.xyz_dot), xyz_dot_sensor.vector.data, 3*sizeof (double));

        // set euler rates
        //so3_body2euler (abc_sensor.vector.data, state.rph, state.rph_dot, NULL);

        velodyne_collector_push_state (self->collector, state);
        do_push_motion = 0;
    }

    return 1;
}


static void
on_velodyne_list (const lcm_recv_buf_t *rbuf, const char *channel,
                  const senlcm_velodyne_list_t *msg, void *user)
{
    RendererVelodyne *self = (RendererVelodyne *)user;

    static int64_t last_redraw_utime = 0;
    int64_t now = bot_timestamp_now();

    for (int i=0; i < msg->num_packets; i++)
        process_velodyne (&(msg->packets[i]), self);

    if ((now - last_redraw_utime) > MAX_REFRESH_RATE_USEC) {
        bot_viewer_request_redraw( self->viewer );
        last_redraw_utime = now;
    }

    return;
}

static void
on_velodyne (const lcm_recv_buf_t *rbuf, const char *channel,
             const senlcm_velodyne_t *msg, void *user)
{
    RendererVelodyne *self = (RendererVelodyne *)user;
    g_assert(self);

    process_velodyne (msg, self);

    return;
}

void
circ_free_velodyne_data(void *user, void *p) {

    velodyne_laser_return_collection_t *lrc = p;
    velodyne_free_laser_return_collection (lrc);
}

void
circ_free_pose_data(void *user, void *p) {

    pose_data_t *pose = p;
    free (pose);
}


static void
renderer_velodyne_destroy (BotRenderer *renderer)
{
    if (!renderer)
        return;

    RendererVelodyne *self = (RendererVelodyne *) renderer->user;
    if (!self)
        return;

    if (self->velodyne_data_circ)
        bot_ptr_circular_destroy (self->velodyne_data_circ);

    if (self->calib)
        velodyne_calib_free (self->calib);
    if (self->collector)
        velodyne_laser_return_collector_free (self->collector);

    free (self);
}

static void
renderer_velodyne_draw (BotViewer *viewer, BotRenderer *renderer)
{
    RendererVelodyne *self = (RendererVelodyne*)renderer->user;
    g_assert(self);

    int colormode = bot_gtk_param_widget_get_enum(self->pw, PARAM_COLOR_MENU);
    if(colormode == COLOR_NONE)
        return;

    double bot_pos[3];
    if (!frames_vehicle_pos_local (self->frames, bot_pos))
        colormode = COLOR_INTENSITY; // Render intensity if there is no pose for height

    // draw the velodyne point cloud
    int hist_len = bot_gtk_param_widget_get_int (self->pw, PARAM_HISTORY_LENGTH);

    for (unsigned int cidx = 0;
         cidx < bot_ptr_circular_size(self->velodyne_data_circ) && cidx < hist_len;
         cidx++) {

        velodyne_laser_return_collection_t *lrc = bot_ptr_circular_index(self->velodyne_data_circ, cidx);

        double sensor_to_local[12];

        if (!bot_frames_get_trans_mat_3x4 (self->frames, "VELODYNE",
                                           "local",
                                           sensor_to_local)) {
            fprintf (stderr, "Error getting bot_frames transformation from VELODYNE to local!\n");
            return;
            }
        /*if (!bot_frames_get_trans_mat_3x4_with_utime (self->frames, "VELODYNE",
                                                      "local", lrc->utime,
                                                      sensor_to_local)) {
            fprintf (stderr, "Error getting bot_frames transformation from VELODYNE to local!\n");
            return;
            }*/

        fprintf( stdout, "sensor_to_local=%.2f\n", sensor_to_local[11] );

        glPushAttrib (GL_DEPTH_BUFFER_BIT | GL_POINT_BIT | GL_CURRENT_BIT);
        glEnable (GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glPointSize(bot_gtk_param_widget_get_double(self->pw, PARAM_POINT_SIZE));
        glBegin(GL_POINTS);

        double alpha = bot_gtk_param_widget_get_double(self->pw, PARAM_POINT_ALPHA);

        //fprintf(stderr,"LASER returns : %d\n", lrc->num_lr);

        int chunk_size = 32;// * 12;

        for (int s = 0; s < lrc->num_lr; s++) {
            velodyne_laser_return_t *lr = &(lrc->laser_returns[s]);

            if(s % chunk_size == 0){
                //updated the sensor_to_local transform
                if (!bot_frames_get_trans_mat_3x4_with_utime (self->frames, "VELODYNE",
                                                              "local", lr->utime,
                                                              sensor_to_local)) {
                    fprintf (stderr, "Error getting bot_frames transformation from VELODYNE to local!\n");
                    return;
                }
            }


            //fprintf(stderr, "\t %d - %d : %f\n", lr->physical, lr->logical, lr->phi);
            double local_xyz[3];
            bot_vector_affine_transform_3x4_3d (sensor_to_local, lr->xyz, local_xyz);

            switch (colormode)  {
            case COLOR_INTENSITY: {
                double v = lr->intensity;
                glColor4d(v, v, v, alpha);
                break;
            }
            case COLOR_Z: {
                double z = local_xyz[2] - bot_pos[2];
                double Z_MIN = 0, Z_MAX = 3;
                double z_norm_scale = 1 / (Z_MAX-Z_MIN);
                double z_norm = (z - Z_MIN) * z_norm_scale;
                glColor3fv(bot_color_util_jet(z_norm));
                break;
            }
            default:
                break;
            }

            glVertex3d (local_xyz[0], local_xyz[1], local_xyz[2]);



        }

        glEnd();
        glPopAttrib();
    }
}

static void
on_param_widget_changed (BotGtkParamWidget *pw, const char *name, void *user)
{
    RendererVelodyne *self = user;
    bot_viewer_request_redraw (self->viewer);
}

static void
on_load_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererVelodyne *self = user_data;
    bot_gtk_param_widget_load_from_key_file (self->pw, keyfile, self->renderer.name);
}

static void
on_save_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererVelodyne *self = user_data;
    bot_gtk_param_widget_save_to_key_file (self->pw, keyfile, self->renderer.name);
}

static void
on_bot_pose (const lcm_recv_buf_t *buf, const char *channel,
             const bot_core_pose_t *msg, void *user) {

    RendererVelodyne *self =  (RendererVelodyne *) user;

    g_mutex_lock (self->mutex);

    if (self->bot_pose_last)
        bot_core_pose_t_destroy (self->bot_pose_last);
    self->bot_pose_last = bot_core_pose_t_copy (msg);

    g_mutex_unlock (self->mutex);
}


static RendererVelodyne *
renderer_velodyne_new (BotViewer *viewer, BotParam * param, lcm_t *_lcm)
{
    RendererVelodyne *self = (RendererVelodyne*) calloc (1, sizeof (*self));

    self->viewer = viewer;

    BotRenderer *renderer = &self->renderer;
    renderer->draw = renderer_velodyne_draw;
    renderer->destroy = renderer_velodyne_destroy;
    renderer->widget = bot_gtk_param_widget_new();
    renderer->name = RENDERER_NAME;
    renderer->user = self;
    renderer->enabled = 1;

    self->lcm = _lcm;// bot_lcm_get_global (NULL);
    if (!self->lcm) {
        fprintf (stderr,"Error: setup_renderer_laser() failed to get global lcm object\n");
        renderer_velodyne_destroy (renderer);
        return NULL;
    }

    self->param = param;
    if (!self->param) {
        fprintf (stderr,"Error: setup_renderer_laser() failed to get BotParam instance\n");
        renderer_velodyne_destroy (renderer);
        return NULL;
    }

    self->frames = bot_frames_get_global (self->lcm, self->param);

    char key[256] = {'\0'};

    snprintf (key, sizeof(key), "%s.channel", "calibration.velodyne");
    char *lcm_channel = bot_param_get_str_or_fail (self->param, key);

    char lcm_channel_list[256];
    snprintf (lcm_channel_list, sizeof(lcm_channel_list), "%s_LIST", lcm_channel);

    char *velodyne_model = bot_param_get_str_or_fail (self->param, "calibration.velodyne.model");
    char *calib_file = bot_param_get_str_or_fail (self->param, "calibration.velodyne.intrinsic_calib_file");

    char calib_file_path[2048];

    sprintf(calib_file_path, "%s/%s", getConfigPath(), calib_file);

    if (0 == strcmp (velodyne_model, VELODYNE_HDL_32E_MODEL_STR))
        self->calib = velodyne_calib_create (VELODYNE_SENSOR_TYPE_HDL_32E, calib_file_path);
    else if (0 == strcmp (velodyne_model, VELODYNE_HDL_64E_S1_MODEL_STR))
        self->calib = velodyne_calib_create (VELODYNE_SENSOR_TYPE_HDL_64E_S1, calib_file_path);
    else if (0 == strcmp (velodyne_model, VELODYNE_HDL_64E_S2_MODEL_STR))
        self->calib = velodyne_calib_create (VELODYNE_SENSOR_TYPE_HDL_64E_S2, calib_file_path);
    else if (0 == strcmp (velodyne_model, VELODYNE_VLP_16_MODEL_STR))
        self->calib = velodyne_calib_create (VELODYNE_SENSOR_TYPE_VLP_16, calib_file_path);
    else
        fprintf (stderr, "ERROR: Unknown Velodyne model \'%s\'", velodyne_model);

    free (velodyne_model);
    free (calib_file);

    self->collector = velodyne_laser_return_collector_create (1, 0, 2* M_PI); // full scan

    // print to debug calibration parsing
    // velodyne_calib_dump (self->calib);


    self->mutex = g_mutex_new ();

    self->velodyne_data_circ = bot_ptr_circular_new (VELODYNE_DATA_CIRC_SIZE,
                                                     circ_free_velodyne_data, self);

    self->pw = BOT_GTK_PARAM_WIDGET (renderer->widget);

    gtk_widget_show_all (renderer->widget);
    g_signal_connect (G_OBJECT (self->pw), "changed",
                      G_CALLBACK (on_param_widget_changed), self);
    g_signal_connect (G_OBJECT (viewer), "load-preferences",
                      G_CALLBACK (on_load_preferences), self);
    g_signal_connect (G_OBJECT (viewer), "save-preferences",
                      G_CALLBACK (on_save_preferences), self);

    bot_gtk_param_widget_add_enum(self->pw, PARAM_COLOR_MENU,
                                  BOT_GTK_PARAM_WIDGET_MENU, COLOR_Z,
                                  "Height", COLOR_Z,
                                  "Intensity", COLOR_INTENSITY,
                                  NULL);

    bot_gtk_param_widget_add_int(self->pw, PARAM_HISTORY_LENGTH,
                                 BOT_GTK_PARAM_WIDGET_SLIDER, 0,
                                 VELODYNE_DATA_CIRC_SIZE, 1, 1);


    bot_gtk_param_widget_add_double(self->pw, PARAM_HISTORY_FREQUENCY,
                                    BOT_GTK_PARAM_WIDGET_SLIDER,
                                    0.1, 10, 0.1, 0.5);

    bot_gtk_param_widget_add_double(self->pw, PARAM_POINT_SIZE,
                                    BOT_GTK_PARAM_WIDGET_SLIDER,
                                    0.5, 10, 0.5, 1.0);

    bot_gtk_param_widget_add_double(self->pw, PARAM_POINT_ALPHA,
                                    BOT_GTK_PARAM_WIDGET_SLIDER,
                                    0, 1, 0.5, 1.0);

    //senlcm_velodyne_t_subscribe (self->lcm, lcm_channel, on_velodyne, self);
    senlcm_velodyne_list_t_subscribe (self->lcm, lcm_channel_list, on_velodyne_list, self);

    // Subscribe to the POSE message
    bot_core_pose_t_subscribe (self->lcm, "POSE", on_bot_pose, self);

    free (lcm_channel);

    return self;
}

void
setup_renderer_velodyne (BotViewer *viewer, int priority, BotParam * param, lcm_t *lcm)
{
    RendererVelodyne *self = renderer_velodyne_new (viewer, param, lcm);
    bot_viewer_add_renderer (viewer, &self->renderer, priority);
}
