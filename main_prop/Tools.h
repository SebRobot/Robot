/*
 * Tools.h
 *
 *  Created on: 11 janv. 2016
 *      Author: Sebastien Malissard
 */

#ifndef TOOLS_H_
#define TOOLS_H_

#include "PositionManager.h"
#include "TrajectoryManager.h"

#define dCAST(a) ((double) (a))
#define iCAST(a) ((int) (a))
#define dPOW2(a) (dCAST(1 << (a)))

#define FixedPoint2Double(a, b)  (dCAST(a) / dPOW2(b))    // a = fixed point number, b = number of digits after the radix point

extern PositionManager pm;
extern TrajectoryManager tm;

#endif /* TOOLS_H_ */
