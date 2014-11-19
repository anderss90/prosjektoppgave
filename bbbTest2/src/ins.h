/*
 * ins.h
 *
 *  Created on: Oct 2, 2014
 *      Author: anderss90
 */

#ifndef INS_H_
#define INS_H_

#include <stdint.h>
#include "common_includes.h"

int ins_init();
int ins_reset();
int ins_update();
int ins_get_state(state_t * returned_state);

#endif /* INS_H_ */
