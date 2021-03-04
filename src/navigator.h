/*
 * trajectory.h
 *
 * Created on : 8 avril 2018
 * Author : Maxime
 */

#ifndef TRAJ_H_
#define TRAJ_H_

typedef enum {  TURN,
				DISPLACEMENT,
				THROW,
				BRAKE,
				CAP}Move_type;

typedef enum { INITIAL_TURN,
				  CRUISE,
				  STOPPED}Move_state;

class Navigator{
public:
	Navigator();
	void move_to(float x, float y);
	void step_forward(float d);
	void step_backward(float d);
	void turn_to(float theta);
	void throw_to(float x, float y, float theta);
	float compute_cons_speed();
	float compute_cons_omega();
	void update();
	void forceStop();
	bool isTrajectoryFinished();
	bool moveForward();
	bool caperror();
	void adjust_rot(float delta_theta);

private:
	bool turn_done;
	bool displacement_done;
	bool trajectory_done;
	bool error_cap;
	float x_target;
	float y_target;
	float theta_target;
	int MAX_ACCEL_OMEGA, MAX_OMEGA;
	bool cup_detected, cup_ready;
	int compt_rot;
	Move_type move_type;
	Move_state move_state;
	float v_r; //read IR_sel
	int a_r;

	float center_axes(float angle);
	float center_radian(float angle);
	int scalaire(float x,float y,float x2,float y2);
};

extern Navigator navigator;

#endif /* TRAJ_H_ */
