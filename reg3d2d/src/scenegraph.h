#ifndef _SG_WRAP_
#define _SG_WRAP_

#include "scene_nodes.h"

extern "C" void init_scenegraph();
extern "C" SG_node* get_noderef(int);

#endif
