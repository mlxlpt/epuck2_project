#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

//start the PI regulator thread
void regulator_start(void);
void regulator_position(bool reset);
void distance_stop(bool reset);

#endif /* PI_REGULATOR_H */
