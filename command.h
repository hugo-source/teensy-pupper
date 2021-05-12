#ifndef COMMAND_H
#define COMMAND_H

#include <math.h>

class Command {
  // Stores movement command
  public:
    float horizontal_velocity[2] = {0};
    float yaw_rate = 0.0;
    float height = -0.16;
    float pitch = 0.0;
    float roll = 0.0;
    bool activation = true;
        
    bool hop_event = false;
    bool trot_event = false;
    bool activate_event = true;

};  


#endif
