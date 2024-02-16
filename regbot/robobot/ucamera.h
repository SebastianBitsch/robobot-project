/**********************************************************
 * Software developed by AVA ( Ava Group of the University of Cordoba, ava  at uco dot es)
 * Main author Rafael Munoz Salinas (rmsalinas at uco dot es)
 * This software is released under BSD license as expressed below
 * -------------------------------------------------------------------
 * Copyright (c) 2013, AVA ( Ava Group University of Cordoba, ava  at uco dot es)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *   must display the following acknowledgement:
 *
 *   This product includes software developed by the Ava group of the University of Cordoba.
 *
 * 4. Neither the name of the University nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY AVA ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL AVA BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************/


/***************************************************************************
*   Copyright (C) 2016 by DTU (Christian Andersen)                        *
*   jca@elektro.dtu.dk                                                    *
*                                                                         *
*   This program is free software; you can redistribute it and/or modify  *
*   it under the terms of the GNU Lesser General Public License as        *
*   published by the Free Software Foundation; either version 2 of the    *
*   License, or (at your option) any later version.                       *
*                                                                         *
*   This program is distributed in the hope that it will be useful,       *
*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
*   GNU Lesser General Public License for more details.                   *
*                                                                         *
*   You should have received a copy of the GNU Lesser General Public      *
*   License along with this program; if not, write to the                 *
*   Free Software Foundation, Inc.,                                       *
*   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
***************************************************************************/

#ifndef UCAMERA_H
#define UCAMERA_H


#include <iostream>
#include <sys/time.h>
#include <thread>
// #include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>
//#include <string>

#include "urun.h"
#include "ubridge.h"
#include "utime.h"
#include "u2dline.h"
// this should be defined in the CMakeList.txt ? or ?
#ifdef raspicam_CV_LIBS
#define PI_CAM
#ifdef PI_CAM
#include <raspicam/raspicam.h>
#include <raspicam/raspicam_cv.h>
#endif
#endif

#define PI_CAM

using namespace std;

//// forward declaration
//class UCamera;






/**
 * The camera class has the functions
 * to open, close, configure and
 * capture images from the raspberry pi
 * camera */
class UCamera : public URun
{ // raw camera functions
public:
  int imgAverage;
  // flag to save an image to disk
  bool saveImage;
  // flag to calibrate camera
  bool calibrate;
  // flag to find ArUco Markers
  // bool ArUco;
  // bool ArUcoDone;
  // string robotCommand;

/**
 * The struct ArUcoVal contains the values related to one ArUco marker
 * \var markerId is the ID of the marker ectracted from the parameter markerIds found på the cv::aruco::detectMarkers function
 * \var frame is the picture where the markers are detected.
 * \var rVec and tVec are also found by cv::aruco::detectMarkers.
 * \var label is a sting printet on the picture with rVec, tVec ect.
 * \var robotTurnCommand is a string that can be sendt to the robot, so that it turn and is possitioned parallel to a on of the markers axises
 * \var robotDriveCommand drives the robot the marker
 * \var rob_Ar_H is the homogeneous transformation and rotation matrix that transform a point in the marker frame to a piont in the robot frame.
 * \var done indikate that the robotTurnCommand has been found
 * \var markerUnitVecX, markerUnitVecY, markerUnitVecZ; is the unitvectors of the marker represented in the robot frame
 * \var disteance2marker id the disteance from the robot to the marker in meters
 * \var angle2markerX and angle2markerY are the angle between the robot and the x or y axis of the mar frame respectetly 
 * */
  struct ArUcoVal_t {
    ArUcoVal_t(): done(false){}
    int markerId;
    cv::Mat frame;
    cv::Vec3d rVec, tVec;
    string label;
    string robotTurnCommand;
    string robotDriveCommand;
    cv::Mat rob_Ar_H;
    bool done;
    
    cv::Vec3d markerUnitVecX, markerUnitVecY, markerUnitVecZ;
    double disteance2marker;
    double angle2markerY; // assumen that the marker is om the ground and X and Y of the marker are in the same plane as x and y of the robot
    double angle2markerX;
    
  } arUcoVal[100]; //the length is determent by that we use cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_100


  const cv::Size chessboardDimensions  = cv::Size(6,8);
  const float calibrationSqaureDimensions = 0.032f; // meters
  const string calibrtionFileName = "ILoveCameraCalibration";//calibration/CameraMatrix.txt";

  cv::Mat cameraMatrix;
  cv::Mat distanceCoefficients;
  //cv::Mat rob_Ar_H30;

public:
  /** Constructor */
  UCamera(UBridge * reg);
  /** destructor */
  ~UCamera();
  void stop();


  /**
   * find ArUco */
  int startWebcamMonitoring();

private:
  // pointer to regbot interface
  UBridge * regbot;
  // read thread handle
//   thread * th1;
//   // set true to stop thread
//   bool th1stop;
  // image saved number
  int imageNumber = 0;
  // time image was taken
  UTime imTime, im2Time;
  // logfile for images
  FILE * logImg = NULL;

#ifdef PI_CAM
protected:
  /**
   * The raw raspberry pi camera device, as defined by the
   * Ava Group of the University of Cordoba */
#ifdef raspicam_CV_LIBS
  raspicam::RaspiCam_Cv Camera;



#else
  //cv::VideoCapture camera(0);

#endif

  //cv::VideoCapture vid();
  //~ if(!vid.isOpened())
    //~ {
        //~ return -1;
    //~ }


  /**
   * Save image to flashdisk */
  void saveImageToFlash(cv::Mat& im);

  /**
   * load Camera matrix */
  bool loadCameraCalibration(string name, cv::Mat& cameraMatrix, cv::Mat& distanceCoefficients);




  /**
   * Calibrations functions */
  void createKnownBoardPosition(cv::Size boardSize, float squareEdgeLength, vector<cv::Point3f>& corners);
  void getChessboardCorners(vector<cv::Mat> images, vector<vector<cv::Point2f>>& allFoundCorners, bool showResults = false);
  void cameraCalibration(vector<cv::Mat> calibrationImages, cv::Size boardSize, float squareEdgeLength, cv::Mat& cameraMatrix, cv::Mat& distanceCoefficients);
  bool saveCameraCalibration(string name, cv::Mat cameraMatrix,cv::Mat distanceCoefficients);
  void cameraCalibrationProcess(cv::Mat& cameraMatrix,cv::Mat& distanceCoefficients);

  /**
   * Aruco functions */
  void display3Labels(string label, string label1, string label2, cv::Mat& frame);
  string extractSingleMarker(cv::Vec3d rotationVectors, cv::Vec3d translationVectors, int markerId, cv::Mat rob_Ar_H);
  vector<double> calcVec(cv::Mat rob_Ar_H);

  /**
   * calculate angle between a vector and the x-axis in 2D */
  double angleCalc( cv::Vec3d vec1);
  
  /**
   * convert radians to degree */
  double rad2deg( double a);

  
  /** 
   * This function finds a turn command, that turns the robot so that it's x-axis (forward) is parrallel with the a-axis of the block (ArUco marker 30).
   * \param tempArUcoVal is a struct with all the available values for a specific marker
   * \return  tempArUcoVal with the robotTurnCommand updated */
  ArUcoVal_t alignWithX(ArUcoVal_t tempArUcoVal);

  /** 
   * Save the unitvector of the marker frame represented in the robot frame
   * \param tempArUcoVal is a struct with all the available values for a specific marker*
   * \returns tempArUcoVal with the markerUnitVecX, markerUnitVecY, markerUnitVecZ updated */
  ArUcoVal_t saveMarkerUnitVec(ArUcoVal_t tempArUcoVal);
  
  /**
   * print out the values of tempArUcoVal to a log-file
   * \param tempArUcoVal[100] is an array of the struct ArUcoVal_t containing values for the markers 1-100.*/
  void logArUcoVal(ArUcoVal_t tempArUcoVal[100]);
  
  /**
   * Calculate the disteance from the origo of the robot frame to origo of marker frame
   * \param rob_Ar_H is a homogeneous transformation matrix from the ArUco markers frame to robot frame 
   * \returns the distance*/
  double distance(cv::Mat rob_Ar_H);

    /**
   * Compute homogeneous transformation matrix.
   * \param rVec is a rotation vector of the estimated possition af a single marker
   * \param tVec is a translation vector of the estimated possition af a single marker
   * \param rob_Ar_H is a homogeneous transformation matrix, that transform points in the ArUco markers coordinate system to the robot coordinate system.*/
  void markerToRobotCoordinate(cv::Vec3d rVec, cv::Vec3d tVec, cv::Mat& rob_Ar_H);



  /** do some image processing
   * \param im is first image with no line
   * \param im2 is second image with laser line
   * \param imTime is tiomestamp of first image,
   * \param im2Time is tiomestamp of second image,
   * \returns true if line found. */
//  bool imageLineProcess(cv::Mat * im, cv::Mat * im2, UTime imTime, UTime im2Time);
 /**
  * Ransac lines */
//  U2Dline findMainLine(std::vector<cv::Point> &linePoints);

#ifdef raspicam_CV_LIBS

  /** text support for exposure */
  raspicam::RASPICAM_EXPOSURE getExposureFromString ( string str )
  {
    if ( str=="OFF" ) return raspicam::RASPICAM_EXPOSURE_OFF;
    if ( str=="AUTO" ) return raspicam::RASPICAM_EXPOSURE_AUTO;
    if ( str=="NIGHT" ) return raspicam::RASPICAM_EXPOSURE_NIGHT;
    if ( str=="NIGHTPREVIEW" ) return raspicam::RASPICAM_EXPOSURE_NIGHTPREVIEW;
    if ( str=="BACKLIGHT" ) return raspicam::RASPICAM_EXPOSURE_BACKLIGHT;
    if ( str=="SPOTLIGHT" ) return raspicam::RASPICAM_EXPOSURE_SPOTLIGHT;
    if ( str=="SPORTS" ) return raspicam::RASPICAM_EXPOSURE_SPORTS;
    if ( str=="SNOW" ) return raspicam::RASPICAM_EXPOSURE_SNOW;
    if ( str=="BEACH" ) return raspicam::RASPICAM_EXPOSURE_BEACH;
    if ( str=="VERYLONG" ) return raspicam::RASPICAM_EXPOSURE_VERYLONG;
    if ( str=="FIXEDFPS" ) return raspicam::RASPICAM_EXPOSURE_FIXEDFPS;
    if ( str=="ANTISHAKE" ) return raspicam::RASPICAM_EXPOSURE_ANTISHAKE;
    if ( str=="FIREWORKS" ) return raspicam::RASPICAM_EXPOSURE_FIREWORKS;
    return raspicam::RASPICAM_EXPOSURE_AUTO;
  }
  /** text support for white balance */
  raspicam::RASPICAM_AWB getAwbFromString ( string str )
  {
    if ( str=="OFF" ) return raspicam::RASPICAM_AWB_OFF;
    if ( str=="AUTO" ) return raspicam::RASPICAM_AWB_AUTO;
    if ( str=="SUNLIGHT" ) return raspicam::RASPICAM_AWB_SUNLIGHT;
    if ( str=="CLOUDY" ) return raspicam::RASPICAM_AWB_CLOUDY;
    if ( str=="SHADE" ) return raspicam::RASPICAM_AWB_SHADE;
    if ( str=="TUNGSTEN" ) return raspicam::RASPICAM_AWB_TUNGSTEN;
    if ( str=="FLUORESCENT" ) return raspicam::RASPICAM_AWB_FLUORESCENT;
    if ( str=="INCANDESCENT" ) return raspicam::RASPICAM_AWB_INCANDESCENT;
    if ( str=="FLASH" ) return raspicam::RASPICAM_AWB_FLASH;
    if ( str=="HORIZON" ) return raspicam::RASPICAM_AWB_HORIZON;
    return raspicam::RASPICAM_AWB_AUTO;
  }
#endif
public:
  /** Capture an image and load image to cv::Mat structure
   * \param image is the destination for the image
   * \returns timestamp when the image was grabbed. */
  timeval capture(cv::Mat &image);
  /**
   * Configure camera */
  bool setupCamera()
  { // image should be a multible of 320x240
    // or the image will be cropped
    int w = (2592/640)*320; // (2592/320)*320; // 320*5; //
    int h = (1992/480)*240; // (1992/240)*240; // 240*3; //
    printf("image size %dx%d\n", h, w);

#ifdef raspicam_CV_LIBS
    //Camera.setWidth(w);
    //Camera.setHeight(h);
    //Camera.setExposure(getExposureFromString ("AUTO"));
    //Camera.setVideoStabilization(false);
    //Camera.setAWB(getAwbFromString("AUTO"));
    //Camera.setMetering(raspicam::RASPICAM_METERING_SPOT);
    cout<<"Connecting to camera"<<endl;
    if ( !Camera.open() ) {
      cerr<<"Error opening camera"<<endl;
      return false;
    }
    for (int i = 0; i < 30; i++)
      // just to make sure camera settins har reached steady state
      Camera.grab();
    cout<<"Connected to pi-camera ='"<<Camera.getId() <<
          "' bufs="<<"Camera.getImageBufferSize( )"<<" bytes\r\n";
#endif

    //cv::Mat cameraMatrix = cv::Mat::eye(3,3, CV_64F);
    //cv::Mat distanceCoefficients;
    ////cout << "start cameraCalibrationProcess" << endl;
    //cameraCalibrationProcess(cameraMatrix, distanceCoefficients);
    //loadCameraCalibration(calibrtionFileName, cameraMatrix, distanceCoefficients);

    return true;
  }
  /**
   * OpenCV image capture */
//   static cv::Mat GetImageFromCamera(cv::VideoCapture& camera);
  /**
   * method to do the image analysis - see ucamera.cpp */
  void run();
#endif
};

/**
 * ArUco pose info */
class UArUco : public UCamera
{

private:
  vector<cv::Vec3d> rVec, tVec;
  vector<int> markerIds;
public:

  // constructor
  UArUco();


  cv::Vec3d getRVec(int markerId)
  {
    return rVec[markerId];
  }

  cv::Vec3d getTVec(int markerId)
  {
    return tVec[markerId];
  }
  //
  //void decode(char * msg);
  ////
  //virtual void subscribe();

};
#endif
