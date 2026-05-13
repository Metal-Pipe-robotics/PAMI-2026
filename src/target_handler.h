#ifndef _TARGET_HANLER_H_ 
#define _TARGET_HANLER_H_ 

int is_cmd_finished_pos(float x, float y, float z);
float get_angle_to_target(float x, float y, float z);
void exec_next_cmd(float x, float y, float z);
void def_targets();

#endif /* _TARGET_HANLER_H_ */
