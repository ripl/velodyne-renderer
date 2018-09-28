#ifndef __VIEWER_RENDERERS_H__
#define __VIEWER_RENDERERS_H__

#ifdef __cplusplus
extern "C" {
#endif


void
setup_renderer_velodyne (BotViewer *viewer, int priority, BotParam * param, lcm_t *lcm);

void
own_view_handler (char *id_key);

static inline void 
draw_axis (double meters_per_grid)
{
    double s = meters_per_grid*0.5;
    glLineWidth (fmax (meters_per_grid*0.05,  2)); 
    glBegin (GL_LINES);
      glColor3f (1.0,0.0,0.0); glVertex3f (0.0,0.0,0.0); glVertex3f (s,0.0,0.0);
      glColor3f (0.0,1.0,0.0); glVertex3f (0.0,0.0,0.0); glVertex3f (0.0,s,0.0);
      glColor3f (0.0,0.0,1.0); glVertex3f (0.0,0.0,0.0); glVertex3f (0.0,0.0,s);
    glEnd ();
}

#ifdef __cplusplus
}
#endif

#endif // __VIEWER_RENDERERS_H__
