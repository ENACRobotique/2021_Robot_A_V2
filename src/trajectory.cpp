#include "trajectory.h"
#include <vector>
#include <iostream>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159
#endif


Coord l_pts[] = {
    {-1250.0,500.0,BEGIN},
    {-1200.0,500.0,ECOCUP},
    {-1200.0,100.0,ECOCUP},
    //{-100.0,100.0, RELEASE}
    //{0.0,0.0,TURNPOINT},
    {-1250.0,500.0,END}
};


Point::Point() {
    
}

Point::Point(float x0, float y0)
{
    x = x0;
    y = y0;
};

float Point::get_dist(Point p)
{
    return sqrt(pow(x + p.x, 2) + pow(y + p.y, 2));
}

float Point::get_theta(Point p)
{ // donne le relevement du Point p/r à p
    if ((x - p.x) > 0)
    {
        return M_PI / 2 - atan((y - p.y) / (x - p.x));
    }
    else if ((x - p.x) < 0)
    {
        return 3 * M_PI / 2 - atan((y - p.y) / (x - p.x));
    }
    else
    {
        return 0.0;
    }
};

Waypoint::Waypoint() {

}

Waypoint::Waypoint(float x0, float y0, WP_type t_type) : Point(x0, y0), type(t_type){

                                                                        };

Trajectory::Trajectory()
{
    pos=0;
    unsigned int i_loc=0;
    for (i_loc=0;i_loc<sizeof(l_pts);i_loc++) {
        trajectory.push_back(Waypoint(l_pts[i_loc].x,l_pts[i_loc].y,l_pts[i_loc].type));
    };

};

Waypoint Trajectory::get_current_WP()
{
    return trajectory[pos];
}

Waypoint Trajectory::get_next_WP()
{
    if (pos < trajectory.size())
    {
        pos++;
        return trajectory[pos];
    }
    else
    {
        return trajectory[pos];
    }
}
float Trajectory::get_rad()
{
    return trajectory[pos].get_theta(trajectory[pos - 1]);
}
float Trajectory::currentX_track(float x0, float y0)
{
    Point p = Point(x0, y0);
    return p.get_dist(trajectory[pos - 1]) * sin(p.get_theta(trajectory[pos - 1]) - trajectory[pos].get_theta(trajectory[pos - 1]));
}
float Trajectory::currentX_angle(float x0, float y0)
{
    Point p = Point(x0, y0);
    return p.get_theta(trajectory[pos - 1]) - get_rad();
}

Trajectory traj1 = Trajectory();
