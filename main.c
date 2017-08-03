#include <string.h>
#include <gtk/gtk.h>

#include <bot_core/bot_core.h>
#include <bot_vis/bot_vis.h>

#include <bot_param/param_client.h>
#include <bot_param/param_util.h>
#include <hr_common/path_util.h>
#include "renderer_velodyne.h"

typedef struct {
    BotViewer *viewer;
    lcm_t *lcm;
} state_t;

/////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
    gtk_init(&argc, &argv);
    glutInit(&argc, argv);
    g_thread_init(NULL);

    setlinebuf(stdout);

    state_t app;
    memset(&app, 0, sizeof(app));

    BotViewer *viewer = bot_viewer_new("Velodyne-Viewer");
    app.viewer = viewer;
    app.lcm = lcm_create(NULL);
    bot_glib_mainloop_attach_lcm(app.lcm);

    BotParam * param;
    if (!(param = bot_param_get_global(app.lcm, 0))) {
        fprintf(stderr,"No server found : Reading from file\n");
        char config_path[2048];
        sprintf(config_path, "%s/wheelchair.cfg", getConfigPath());
        param = bot_param_new_from_file(config_path);

        if(!param){
            fprintf (stderr, "Unable to get BotParam instance\n");
            return 0;
        }
    }

    // setup renderers
    bot_viewer_add_stock_renderer(viewer, BOT_VIEWER_STOCK_RENDERER_GRID, 1);
    setup_renderer_velodyne(viewer, 0, param, app.lcm);

    // load saved preferences
    char *fname = g_build_filename(g_get_user_config_dir(), ".velodyne-viewerrc", NULL);
    bot_viewer_load_preferences(viewer, fname);

    // run the main loop
    gtk_main();

    // save any changed preferences
    bot_viewer_save_preferences(viewer, fname);
    free(fname);

    // cleanup
    bot_viewer_unref(viewer);

    return 0;
}
