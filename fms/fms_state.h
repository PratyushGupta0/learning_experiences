#include<opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include<math.h>

#include "fms.h"

class fms;

class fms_state
{
public:
	virtual void enter(fms* fms) = 0;
	virtual void toggle(fms* fms) = 0;
	virtual void exit(fms* fms) = 0;
	virtual ~fms_state() {}
};