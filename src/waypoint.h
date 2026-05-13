#ifndef __WAYOINT_H__
#define __WAYPOINT_H__ 

struct Pose {
  float x;
  float y;
  float theta;
  float margin;
};

void waypoint_init_all();
void waypoint_add_pose(Pose p);
void waypoint_add_pose_front(Pose p);
struct Pose waypoint_pop_pose();

#endif /* __WAYPOINT_H__ */

