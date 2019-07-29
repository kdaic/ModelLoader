/*
 * Copyright (c) 2019, kdaic
 * Original source:
 *   https://github.com/fkanehiro/openhrp3/blob/3.1.9/hrplib/hrpModel/ModelLoaderUtil.h
 * Copyright (c) 2008, AIST, the University of Tokyo and General Robotix Inc.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 * Contributors:
 * National Institute of Advanced Industrial Science and Technology (AIST)
 */

/**
   @author Shin'ichiro Nakaoka
   @author kdaic
*/

#ifndef HRPMODEL_MODEL_LOADER_UTIL_H_INCLUDED
#define HRPMODEL_MODEL_LOADER_UTIL_H_INCLUDED

#ifdef __WIN32__
#pragma warning(disable:4996)
#endif

#include <string>
#include <sstream>

#include "Body.h"


namespace hrp
{
    HRPMODEL_API bool loadBodyFromModelLoader(BodyPtr body, const char* url, bool loadGeometryForCollisionDetection = false);

};


#endif
