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

#include "Ports.h"
#include "stdio.h"
#include <sys/time.h>

struct timeval tv;

void publishPort(const humandetection_Humans *Humans, int nbframe,
                 sequence_detectionH_people People, genom_context self)
{
    int i, j;
    detectionH_people aux;

    /*   **1**  **2**  **3**  **4**  **5** **6**   */
    /*   | <-   |                          |new    */
    /*   **2**  **3**  **4**  **5**  **6** **7**   */

    for(i=0; i<Humans->data(self)->frame._length-1; i++)
    {
        //Move frameNumber
        Humans->data(self)->frame._buffer[i].frameNumber = Humans->data(self)->frame._buffer[i+1].frameNumber;

        //Move time
        Humans->data(self)->frame._buffer[i].time.sec = Humans->data(self)->frame._buffer[i+1].time.sec;
        Humans->data(self)->frame._buffer[i].time.usec = Humans->data(self)->frame._buffer[i+1].time.usec;
        //Move people
       genom_sequence_reserve(&(Humans->data(self)->frame._buffer[i].people), Humans->data(self)->frame._buffer[i+1].people._length);
        Humans->data(self)->frame._buffer[i].people._length = Humans->data(self)->frame._buffer[i+1].people._length; 
        for(j=0; j<Humans->data(self)->frame._buffer[i].people._length; j++)
        {
            Humans->data(self)->frame._buffer[i].people._buffer[j].ID = Humans->data(self)->frame._buffer[i+1].people._buffer[j].ID;
            Humans->data(self)->frame._buffer[i].people._buffer[j].coordinates.x = Humans->data(self)->frame._buffer[i+1].people._buffer[j].coordinates.x;
            Humans->data(self)->frame._buffer[i].people._buffer[j].coordinates.y = Humans->data(self)->frame._buffer[i+1].people._buffer[j].coordinates.y;
            Humans->data(self)->frame._buffer[i].people._buffer[j].coordinates.z = Humans->data(self)->frame._buffer[i+1].people._buffer[j].coordinates.z;
            Humans->data(self)->frame._buffer[i].people._buffer[j].coordinates.azimuth = Humans->data(self)->frame._buffer[i+1].people._buffer[j].coordinates.azimuth;
        }
    }

    //Move new (lastest) data to the 'end' of the port.
    Humans->data(self)->frame._buffer[Humans->data(self)->frame._length-1].frameNumber = nbframe;
    gettimeofday(&tv, NULL);
    Humans->data(self)->frame._buffer[Humans->data(self)->frame._length-1].time.sec = tv.tv_sec;
    Humans->data(self)->frame._buffer[Humans->data(self)->frame._length-1].time.usec = tv.tv_usec;
    genom_sequence_reserve(&(Humans->data(self)->frame._buffer[Humans->data(self)->frame._length-1].people), People._length);
    Humans->data(self)->frame._buffer[Humans->data(self)->frame._length-1].people._length = People._length; 

    for(i=0; i<People._length; i++)
    {
        Humans->data(self)->frame._buffer[Humans->data(self)->frame._length-1].people._buffer[i].ID = People._buffer[i].ID;
        Humans->data(self)->frame._buffer[Humans->data(self)->frame._length-1].people._buffer[i].coordinates.x = People._buffer[i].coordinates.x;
        Humans->data(self)->frame._buffer[Humans->data(self)->frame._length-1].people._buffer[i].coordinates.y = People._buffer[i].coordinates.y;
        Humans->data(self)->frame._buffer[Humans->data(self)->frame._length-1].people._buffer[i].coordinates.z = People._buffer[i].coordinates.z;
        Humans->data(self)->frame._buffer[Humans->data(self)->frame._length-1].people._buffer[i].coordinates.azimuth = People._buffer[i].coordinates.azimuth;
    }

    //Arrange the IDs from min to max.
    for(i=0; i<Humans->data(self)->frame._buffer[Humans->data(self)->frame._length-1].people._length; i++)
    {
        for(j=0; j<Humans->data(self)->frame._buffer[Humans->data(self)->frame._length-1].people._length-1; j++)
        {
            if(Humans->data(self)->frame._buffer[Humans->data(self)->frame._length-1].people._buffer[i].ID < Humans->data(self)->frame._buffer[Humans->data(self)->frame._length-1].people._buffer[j].ID)
            {
                aux.ID = Humans->data(self)->frame._buffer[Humans->data(self)->frame._length-1].people._buffer[i].ID;
                aux.coordinates.x = Humans->data(self)->frame._buffer[Humans->data(self)->frame._length-1].people._buffer[i].coordinates.x;
                aux.coordinates.y = Humans->data(self)->frame._buffer[Humans->data(self)->frame._length-1].people._buffer[i].coordinates.y;
                aux.coordinates.z = Humans->data(self)->frame._buffer[Humans->data(self)->frame._length-1].people._buffer[i].coordinates.z;
                aux.coordinates.azimuth = Humans->data(self)->frame._buffer[Humans->data(self)->frame._length-1].people._buffer[i].coordinates.azimuth;

                Humans->data(self)->frame._buffer[Humans->data(self)->frame._length-1].people._buffer[i].ID = Humans->data(self)->frame._buffer[Humans->data(self)->frame._length-1].people._buffer[j].ID;
                Humans->data(self)->frame._buffer[Humans->data(self)->frame._length-1].people._buffer[i].coordinates.x = Humans->data(self)->frame._buffer[Humans->data(self)->frame._length-1].people._buffer[j].coordinates.x;
                Humans->data(self)->frame._buffer[Humans->data(self)->frame._length-1].people._buffer[i].coordinates.y = Humans->data(self)->frame._buffer[Humans->data(self)->frame._length-1].people._buffer[j].coordinates.y;
                Humans->data(self)->frame._buffer[Humans->data(self)->frame._length-1].people._buffer[i].coordinates.z = Humans->data(self)->frame._buffer[Humans->data(self)->frame._length-1].people._buffer[j].coordinates.z;
                Humans->data(self)->frame._buffer[Humans->data(self)->frame._length-1].people._buffer[i].coordinates.azimuth = Humans->data(self)->frame._buffer[Humans->data(self)->frame._length-1].people._buffer[j].coordinates.azimuth;

                Humans->data(self)->frame._buffer[Humans->data(self)->frame._length-1].people._buffer[j].ID = aux.ID;
                Humans->data(self)->frame._buffer[Humans->data(self)->frame._length-1].people._buffer[j].coordinates.x = aux.coordinates.x;
                Humans->data(self)->frame._buffer[Humans->data(self)->frame._length-1].people._buffer[j].coordinates.y = aux.coordinates.y;
                Humans->data(self)->frame._buffer[Humans->data(self)->frame._length-1].people._buffer[j].coordinates.z = aux.coordinates.z;
                Humans->data(self)->frame._buffer[Humans->data(self)->frame._length-1].people._buffer[j].coordinates.azimuth = aux.coordinates.azimuth;
            }
        }
    }

    Humans->write(self);
}
