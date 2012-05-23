/**
\brief A timer module with only a single compare value. Can be used to replace
       the "bsp_timer" and "radiotimer" modules with the help of abstimer.

\author Xavi Vilajosana <xvilajosana@eecs.berkeley.edu>, May 2012.
\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, May 2012.
*/

#ifndef __SCTIMER_H
#define __SCTIMER_H

#include "stdint.h"

//=========================== define ==========================================

//=========================== typedef =========================================
typedef uint8_t (*sctimer_cbt)(void);

//=========================== variables =======================================


//=========================== prototypes ======================================

void sctimer_init();
void sctimer_schedule(uint16_t val);
uint16_t sctimer_getValue();
void sctimer_setCb(sctimer_cbt cb);

#endif