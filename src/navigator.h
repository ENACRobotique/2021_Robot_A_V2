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
				BRAKE}Move_type;

typedef enum { INITIAL_TURN,
				  CRUISE,
				  VELOCITY,
				  STOPPED}Move_state;

class Navigator{
public:
	Navigator();
	void move_to(float x, float y);
	void move(float v, float omega);//simple contôle moteur en vitesse
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

private:
	bool turn_done;
	bool displacement_done;
	bool trajectory_done;
	float x_target;
	float y_target;
	float theta_target;
	float v_target, omega_target;
	Move_type move_type;
	Move_state move_state;

	float center_axes(float angle);
	float center_radian(float angle);
	int scalaire(float x,float y,float x2,float y2);
};

extern Navigator navigator;

#endif /* TRAJ_H_ */
