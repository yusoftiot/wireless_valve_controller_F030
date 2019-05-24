/*
 * version.h
 *
 *  Created on: Jun 4, 2018
 *      Author: yuri
 */

#ifndef VERSION_H_
#define VERSION_H_

#include <time.h>

/**
 *
 */

// -DCOMPILE_TIME=`date '+%s'`- doues not working
// -DCOMPILE_TIME="$(date '+%s')"
// or insert for _COMPILE_TIME="$(date '+%s')"

#define VERSION         __DATE__ " " __TIME__ " "
//#define VERSION        _COMPILE_TIME ", "
// -D__TIMESTAMP__=$(date +'"%Y-%m-%dT%H:%M:%S"')

//#define VERSION         __TIMESTAMP__


#endif /* VERSION_H_ */
