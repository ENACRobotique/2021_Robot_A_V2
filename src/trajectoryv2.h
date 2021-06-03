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
    ECOCUP_RED,
    ECOCUP_GREEN,
    RELEASE_RED,
    RELEASE_GREEN,
    END,
    BEGIN
} WP_type;

typedef struct {
    float x;
    float y;
    WP_type type;
    bool bouger;//ne pas se déplacer
    bool anglealeatoire;//prend en compte la suite
    float angle;
}Coord;

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
    Waypoint(float x, float y, WP_type type, bool bouger, bool angleAleatoire, float anglefinal);
    int scalaire(float x, float y, float x2, float y2);
    WP_type type;
    bool bouger;
    bool angleAleatoire;
    float angle;
private:
    //float x,y;
   
};

class Trajectory
{
public:
    Trajectory();
    Waypoint get_current_WP();
    Waypoint get_next_WP();
    float get_rad();
    float currentX_track(float x0, float y0);
    float currentX_angle(float x0, float y0);
    int pos;

private:
    std::vector<Waypoint> trajectory;
};

extern Trajectory traj1;

#endif /* TRAJECTORY_H_ */
