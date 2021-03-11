/*
 * trajectory.h
 *
 * Created on : 9 mars 2021
 * Author : Liam
 */

#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>

typedef enum
{
    TURNPOINT,
    ECOCUP,
    END,
    BEGIN
} WP_type;

class Point
{
public:
    Point();
    Point(float x, float y);
    float get_theta(Point p);
    float get_dist(Point p);
    float x, y;
};

class Waypoint : public Point
{
public:
    Waypoint();
    Waypoint(float x, float y, WP_type type);
    int scalaire(float x, float y, float x2, float y2);

private:
    //float x,y;
    WP_type type;
};

class Trajectory
{
public:
    Trajectory();
    Waypoint get_next_WP();
    float get_rad();
    float currentX_track(float x0, float y0);
    float currentX_angle(float x0, float y0);

private:
    std::vector<Waypoint> trajectory;
    int pos;
};

extern Trajectory traj;

#endif /* TRAJECTORY_H_ */
