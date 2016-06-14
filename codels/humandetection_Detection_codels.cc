#include "achumandetection.h"

#include "humandetection_c_types.h"

#include <iostream>
#include <ctime>
#include <fstream>
#include <time.h>
#include <sys/time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "triangulation.h"
#include "Ports.h"

#define cameratype      3   //3: RGB, 4:RGBD (MORSE)
#define MAX_CLASSIFIERS 5
#define MAX_DETECTIONS  20

using namespace cv;
using namespace std;

cv::Mat MasterFrame, SlaveFrame;
cv::Mat MasterResc, SlaveResc;

clock_t start, finish;

CascadeClassifier detect_cascade;
vector<String> detect_cascade_names;
uint16_t loadedClassifier=0;
std::vector<cv::Rect> detectionsMaster, detectionsSlave;
/* --- Task Detection --------------------------------------------------- */

void detect(Mat frame, std::vector<Rect> &faces)
{
    int i, div=2;
    Mat frame_gray;
    resize(frame, frame_gray, Size(frame.cols/div,frame.rows/div));
    cvtColor(frame_gray, frame_gray, CV_BGR2GRAY);
    equalizeHist( frame_gray, frame_gray );
    detect_cascade.detectMultiScale(frame_gray, faces, 1.2, 2, 0|CV_HAAR_SCALE_IMAGE, Size(24, 24));

    for(i=0; i<faces.size(); i++)
    {
        faces[i].x = faces[i].x * div;    
        faces[i].y = faces[i].y * div;
        faces[i].width = faces[i].width * div;
        faces[i].height = faces[i].height * div;
    }
}

/* --- Activity RunDetection -------------------------------------------- */

/** Codel InitRunDetection of activity RunDetection.
 *
 * Triggered by humandetection_start.
 * Yields to humandetection_exec, humandetection_ether.
 * Throws humandetection_error_videoCap.
 */
genom_event
InitRunDetection(humandetection_ids *ids, uint32_t frameHistory,
                 const char *classifiers,
                 const humandetection_LeftCameraParameters *LeftCameraParameters,
                 const humandetection_RightCameraParameters *RightCameraParameters,
                 const humandetection_Humans *Humans,
                 const humandetection_Classifiers *Classifiers,
                 genom_context self)
{
    int i, count=0;
    FILE *pFile;
    char tmp[150];
    size_t len;

    ids->nbframe = 0;

    if(Humans->data(self)->frame._length == 0)
    {
        genom_sequence_reserve(&(Humans->data(self)->frame), frameHistory);
        Humans->data(self)->frame._length = frameHistory;
    }

    // Retrieve classifier's paths 
    pFile = fopen(classifiers, "r");
    if(pFile==NULL)
    {
        printf("--(!)Error loading file %s\n", classifiers); 
        return humandetection_ether;
    };
    count = 0;
    while(fgets(tmp, 150, pFile) != NULL)
    {
        count++;
        len = strlen(tmp);
        if(len>0 && tmp[len-1]=='\n')
            tmp[--len] = '\0';
        detect_cascade_names.push_back(tmp);
    }
    fclose(pFile);

    //Use frontal face detection by default.
    loadedClassifier=0;

    // Load the cascade
    if(!detect_cascade.load(detect_cascade_names[loadedClassifier]) && count>0)
    {
        printf("--(!)Error loading\n"); 
        return humandetection_ether;
    };

    // Initialize IDS.
    if(genom_sequence_reserve(&(ids->Classifiers.classifiers), detect_cascade_names.size()))
    {
        printf("--(!)Error initializing IDS.\n"); 
        return humandetection_ether;
    };
    ids->Classifiers.classifiers._length = detect_cascade_names.size();

    // Copy cascade's names to IDS.
    for(i=0; i<detect_cascade_names.size(); i++)
    {
        len = strlen(detect_cascade_names[i].c_str());
        ids->Classifiers.classifiers._buffer[i] = (char *)malloc(len);
        strcpy(ids->Classifiers.classifiers._buffer[i], detect_cascade_names[i].c_str());
    }

    // Initialize Port Classifiers
    if(genom_sequence_reserve(&(Classifiers->data(self)->classifiers), ids->Classifiers.classifiers._length))
    {
        printf("--(!)Error initializing Port Classifiers.\n"); 
        return humandetection_ether;
    };
    Classifiers->data(self)->classifiers._length = ids->Classifiers.classifiers._length;

    //Copy data from IDS to Port Classifiers.
    for(i=0; i<Classifiers->data(self)->classifiers._length; i++)
    {
        len = strlen(ids->Classifiers.classifiers._buffer[i]);
        Classifiers->data(self)->classifiers._buffer[i] = (char *)malloc(len);
        strcpy(Classifiers->data(self)->classifiers._buffer[i], ids->Classifiers.classifiers._buffer[i]);
    }
    Classifiers->data(self)->loaded = loadedClassifier;
    Classifiers->write(self);

    cv::namedWindow("Master Camera", cv::WINDOW_NORMAL);
    cv::namedWindow("Slave Camera", cv::WINDOW_NORMAL);

    // Read the camera parameters to retrieve Projection matrix
    LeftCameraParameters->read(self);
    RightCameraParameters->read(self);
    if(LeftCameraParameters->data(self) != NULL && RightCameraParameters->data(self) != NULL)
    {
        memmove(ids->LeftCameraParameters.K, LeftCameraParameters->data(self)->K, 9*sizeof(double));
        memmove(ids->LeftCameraParameters.R, LeftCameraParameters->data(self)->R, 9*sizeof(double));
        memmove(ids->LeftCameraParameters.P, LeftCameraParameters->data(self)->P, 12*sizeof(double));

        memmove(ids->RightCameraParameters.K, RightCameraParameters->data(self)->K, 9*sizeof(double));
        memmove(ids->RightCameraParameters.R, RightCameraParameters->data(self)->R, 9*sizeof(double));
        memmove(ids->RightCameraParameters.P, RightCameraParameters->data(self)->P, 12*sizeof(double));
    }
    else
    {
        std::cout << "Camera parametrs could not be retrieved." << endl;
        return humandetection_ether;
    }

    // Allocate space in the IDS for detections.
    if(ids->MasterDetectionCenters._length == 0)
        genom_sequence_reserve(&(ids->MasterDetectionCenters), MAX_DETECTIONS);
    if(ids->SlaveDetectionCenters._length == 0)
        genom_sequence_reserve(&(ids->SlaveDetectionCenters), MAX_DETECTIONS);
    if(ids->People._length == 0)
        genom_sequence_reserve(&(ids->People), MAX_DETECTIONS);

    srand(time(NULL));
    start = clock();

    return humandetection_exec;
}

/** Codel ExecRunDetection of activity RunDetection.
 *
 * Triggered by humandetection_exec.
 * Yields to humandetection_exec, humandetection_stop.
 * Throws humandetection_error_videoCap.
 */
genom_event
ExecRunDetection(humandetection_ids *ids,
                 const humandetection_MasterCamera *MasterCamera,
                 const humandetection_SlaveCamera *SlaveCamera,
                 const humandetection_Humans *Humans,
                 float templateMatchingThreshold,
                 float disparityThreshold, genom_context self)
{
    int i, j, n;
    char *text;
    float Fx, Fy, Cx, Cy, T;

    text = (char *) malloc(3*sizeof(char));

    MasterCamera->read(self);
    SlaveCamera->read(self);
    if(MasterCamera->data(self) != NULL && SlaveCamera->data(self) != NULL)
    {
        ids->nbframe++;
        // Cameras are RGB and the detection is on BGR.
        MasterFrame = Mat(MasterCamera->data(self)->height,MasterCamera->data(self)->width,CV_8UC3, MasterCamera->data(self)->data._buffer);
        cv::cvtColor(MasterFrame, MasterFrame, CV_RGB2BGR);
        resize(MasterFrame, MasterFrame, Size(MasterFrame.cols/2,MasterFrame.rows/2));
        SlaveFrame = Mat(SlaveCamera->data(self)->height,SlaveCamera->data(self)->width,CV_8UC3, SlaveCamera->data(self)->data._buffer);
        cv::cvtColor(SlaveFrame, SlaveFrame, CV_RGB2BGR);
        resize(SlaveFrame, SlaveFrame, Size(SlaveFrame.cols/2,SlaveFrame.rows/2));

        //-- Detections on Master --//
        detect(MasterFrame, detectionsMaster);
        // Show detections
        for (i=0; i<detectionsMaster.size(); i++)
            cv::rectangle(MasterFrame, detectionsMaster.at(i), cv::Scalar(0, 0, 255), 4);
        std::stringstream ssMaster;
        ssMaster << ids->nbframe;
        cv::putText(MasterFrame, ssMaster.str(), cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0));
        sprintf(text, "%d", (int) detectionsMaster.size());
        cv::putText(MasterFrame, text, cv::Point(10,80), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,255,0));
 

        //-- Detections on Slave --//
        detect(SlaveFrame, detectionsSlave);
        // Show detections
        for (i=0; i<detectionsSlave.size(); i++)
            cv::rectangle(SlaveFrame, detectionsSlave.at(i), cv::Scalar(0, 0, 255), 4);
        std::stringstream ssSlave;
        ssSlave << ids->nbframe;
        cv::putText(SlaveFrame, ssSlave.str(), cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,255,0));
        sprintf(text, "%d", (int) detectionsSlave.size());
        cv::putText(SlaveFrame, text, cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,255,0));

        //-- Match detections from Master to Slave (Template Matching) --//
        cv::Mat tmpl, match, result;
        int match_method = CV_TM_CCOEFF_NORMED;
        n = 0;
        for(i=0; i<detectionsMaster.size(); i++)
        {
            tmpl = MasterFrame(detectionsMaster.at(i));
            for(j=0; j<detectionsSlave.size(); j++)
            {
                match = SlaveFrame(detectionsSlave.at(j));
                cv::resize(match, match, Size(tmpl.cols, tmpl.rows));
                int result_cols = match.cols - tmpl.cols + 1;
                int result_rows = match.rows - tmpl.rows + 1;
                result.create( result_rows, result_cols, CV_32FC1);
                matchTemplate(match, tmpl, result, match_method);
                double minVal; double maxVal; Point minLoc; Point maxLoc;
                minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
                int ul, ur, dif;
                ul = detectionsMaster.at(i).y;
                ur = detectionsSlave.at(j).y;
                dif = ul - ur;
                if(dif<0)
                    dif = (-1)*dif;
                if(maxVal>templateMatchingThreshold && dif<=disparityThreshold)
                {
                    ids->MasterDetectionCenters._buffer[n].x = (uint32_t) (detectionsMaster.at(i).x+(uint32_t)(detectionsMaster.at(i).width/2));
                    ids->MasterDetectionCenters._buffer[n].y = (uint32_t) (detectionsMaster.at(i).y+(uint32_t)(detectionsMaster.at(i).height/2));

                    ids->SlaveDetectionCenters._buffer[n].x = (uint32_t) (detectionsSlave.at(j).x+(uint32_t)(detectionsSlave.at(j).width/2));
                    ids->SlaveDetectionCenters._buffer[n].y = (uint32_t) (detectionsSlave.at(j).y+(uint32_t)(detectionsSlave.at(j).height/2));

                    // Show the matching detections
                    cv::rectangle(MasterFrame, detectionsMaster.at(i), cv::Scalar(0, 255, 0), 4);
                    cv::rectangle(SlaveFrame, detectionsSlave.at(j), cv::Scalar(0, 255, 0), 4);

                    n++;

                    break;
                }
            }
        }
        ids->MasterDetectionCenters._length = n;
        ids->SlaveDetectionCenters._length = n;

        /* Triangulation */
        if(n>0)
        {
            //As the input images were resized to half as soon as they were read, x, y of 
            //rectangules have to be multiplied by 2.
            for(i=0; i<ids->MasterDetectionCenters._length; i++)
            {
                ids->MasterDetectionCenters._buffer[i].x *=2;
                ids->MasterDetectionCenters._buffer[i].y *=2;
                ids->MasterDetectionCenters._buffer[i].ID = i;
            }
            for(i=0; i<ids->SlaveDetectionCenters._length; i++)
            {
                ids->SlaveDetectionCenters._buffer[i].x *=2;
                ids->SlaveDetectionCenters._buffer[i].y *=2;
                ids->SlaveDetectionCenters._buffer[i].ID = i;
            }

            Fx = ids->LeftCameraParameters.P[0];
            Fy = ids->LeftCameraParameters.P[5];
            Cx = ids->LeftCameraParameters.P[2];
            Cy = ids->LeftCameraParameters.P[6];
            T = (-1)*(ids->RightCameraParameters.P[3]/Fx);

            triangulationFromDisparity(&ids->People, ids->MasterDetectionCenters, ids->SlaveDetectionCenters, Fx, Fy, Cx, Cy, T);

            publishPort(Humans, ids->nbframe, ids->People, self);
        }

        cv::imshow("Master Camera", MasterFrame);
        cv::imshow("Slave Camera", SlaveFrame); 

        if(cv::waitKey(30) == -1)
        {
            return humandetection_exec;
        }
        else
        {
            return humandetection_stop;
        }

    }

    return humandetection_exec;
}

/** Codel StopRunDetection of activity RunDetection.
 *
 * Triggered by humandetection_stop.
 * Yields to humandetection_ether.
 * Throws humandetection_error_videoCap.
 */
genom_event
StopRunDetection(humandetection_ids *ids,
                 const humandetection_Humans *Humans,
                 genom_context self)
{
    int i;

    MasterFrame.release();
    SlaveFrame.release();

    cv::destroyWindow("Master Camera");
    cv::destroyWindow("Slave Camera");

    finish = clock();
    std::cout << "time = " << (float)(finish - start)/CLOCKS_PER_SEC << std::endl;
    std::cout << "FPS = " << (float)(CLOCKS_PER_SEC*(float)(ids->nbframe))/(finish - start) << std::endl;

    // Clear allocated memory for the Port.
    for(i=0; i<Humans->data(self)->frame._length; i++)
    {
        genom_sequence_reserve(&(Humans->data(self)->frame._buffer[i].people), 0);
        Humans->data(self)->frame._buffer[i].people._length = 0;
    }
    genom_sequence_reserve(&(Humans->data(self)->frame), 0);
    Humans->data(self)->frame._length = 0;

    detect_cascade_names.resize(0);

    return humandetection_ether;
}


/* --- Activity ChangeClassifier ---------------------------------------- */

/** Codel InitChangeClassifier of activity ChangeClassifier.
 *
 * Triggered by humandetection_start.
 * Yields to humandetection_ether.
 */
genom_event
InitChangeClassifier(uint16_t classifier,
                     const humandetection_Classifiers *Classifiers,
                     genom_context self)
{
    /* Load the cascade */
    if(classifier<0 || classifier>MAX_CLASSIFIERS)
        loadedClassifier = 0;   //If wrong number is entered, the default (frontal face) classifier will be loaded.
    else
        loadedClassifier = classifier;
    if(!detect_cascade.load(detect_cascade_names[loadedClassifier]))
    {
        printf("--(!)Error loading\n"); 
        return humandetection_ether;
    };

    Classifiers->data(self)->loaded = loadedClassifier;
    Classifiers->write(self);

    return humandetection_ether;
}
