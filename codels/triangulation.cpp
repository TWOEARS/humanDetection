/* Copyright (c) 2015, LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "triangulation.h"
#include "stdio.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;

void triangulationFromDisparity(sequence_detectionH_people *result, sequence_detectionH_centers frameA, sequence_detectionH_centers frameB, float Fx, float Fy, float Cx, float Cy, float T)
{
    int i, j, m, n, disparity;
   
    result->_length = 0;
    n = 0;
    for(i=0; i<frameA._length; i++)
    {
        for(j=0; j<frameB._length; j++)
        {
            if(frameA._buffer[i].ID == frameB._buffer[j].ID)
            {
                result->_length++;
                disparity = frameA._buffer[i].x - frameB._buffer[j].x;
                result->_buffer[n].ID = frameA._buffer[i].ID;
                result->_buffer[n].coordinates.z = (Fx*T) / disparity;
                result->_buffer[n].coordinates.x = (result->_buffer[n].coordinates.z*(frameA._buffer[i].x-Cx))/Fx;
                result->_buffer[n].coordinates.y = (result->_buffer[n].coordinates.z*(frameA._buffer[i].y-Cy))/Fy;
                result->_buffer[n].coordinates.azimuth = atan(result->_buffer[n].coordinates.x/result->_buffer[n].coordinates.z) * (180/pi);
                n++;
                break;
            }
        }
    }
}
