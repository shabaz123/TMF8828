//

#ifndef COMMON_UTILS_H_
#define COMMON_UTILS_H_

/* generic headers */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define BIT_SHIFT_8  (8u)
#define SIZE_64      (64u)

#define LVL_ERR      (1u)       /* error conditions   */


// create an APP_PRINT macro that calls printf
#define APP_PRINT(fn_, ...)      printf((fn_), ##__VA_ARGS__);
#define APP_ERR_PRINT(fn_, ...)  if(LVL_ERR)\
        printf("[ERR] In Function: %s(), %s",__FUNCTION__,(fn_),##__VA_ARGS__);


// APP_PRINT is used to print the message to the console using normal printf c function
//#define APP_PRINT(fn_, ...)      printf((fn_), ##__VA_ARGS__);
//#define APP_ERR_PRINT(fn_, ...)  if(LVL_ERR)\
//        printf("[ERR] In Function: %s(), %s",__FUNCTION__,(fn_),##__VA_ARGS__);



#endif /* COMMON_UTILS_H_ */
