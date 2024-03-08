

#ifndef FURBS_H
#define FURBS_H

using namespace std;

struct Furbs_vel_params {
	float max_acc = 1;
	float max_vel = 0.7;
	float time_interval = 0.05;
	float dist_margin = 0.03;
	float min_vel = 0.04;
};

/**
 * Helper controller for missions
 * */
class Furbs
{
public:

	/** setup default values and read data from ini file*/
	void setup();

	/**
		Goes for x meters and optionally follow the line.
	*/
	void go_for(float meters,  bool follow_line, Furbs_vel_params p);

	//void go_to(const char * msg, UTime & msgTime);

	/**
	* terminate */
	void terminate();

public:
	Furbs_vel_params vel;
	
	float heading_vel = 0.5;
	float heading_threshold = 20;
	float heading_buildup_remove = 0.5;

private:
	//float position_x;
	//float position_y;

};

/**
 * Make this visible to the rest of the software */
extern Furbs furbs;

#endif