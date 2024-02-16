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

/***************************************************************************
*   August 2019 ArUco functions and Camera calibration functions added    *
*   by Michael Teglgaard                                                  *
*   s130067@student.dtu.dk                                                *
*                                                                         *
*   Part of this code is inpired by code from the YouTube playlist        *
*   OpenCv Basic https://youtu.be/HNfPbw-1e_w                             *
***************************************************************************/

 #include <iostream>
// #include <sys/time.h>
// #include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include "urun.h"
#include "ucamera.h"
#include "ubridge.h"
#include "utime.h"


using namespace std;

//#define PI_CAM
#ifndef PI_CAM
// use when testing code without a camera, or not on a raspberry
UCamera::UCamera(UBridge * reg)
{
  printf("No camera support - set '#define PI_CAM' in ucamera.h to enable\n");
  th1 = NULL;
  th1stop = false;
}

/** destructor */
UCamera::~UCamera()
{
  stop();
}

#endif

void UCamera::stop()
{
  th1stop = true;
  if (th1 != NULL)
    th1->join();
#ifdef raspicam_CV_LIBS
  Camera.release();
#endif
  printf("Finished, camera closed\n");
}


#if defined PI_CAM

/** Constructor */
UCamera::UCamera(UBridge * reg)
{
  th1 = NULL;
  th1stop = false;
  imgAverage = 0;
  saveImage = false;
  calibrate = false;
  //ArUco     = false;
  //ArUcoDone = false;
  loadCameraCalibration(calibrtionFileName, cameraMatrix, distanceCoefficients); //cv::Mat::eye(3,3, CV_64F);

//   lineImage = false;
  regbot = reg;
  bool isOK = setupCamera();
  if (isOK)
  {
    const int MNL = 100;
    char date[MNL];
    char name[MNL];
    th1 = new thread(runObj, this);
    printf("Camera started OK\n");
    cout << "cameraMatrix: \n" << cameraMatrix << endl;
    imTime.now();
    imTime.getForFilename(date);
    // construct filename
    snprintf(name, MNL, "log_image_%s.txt", date);
    logImg = fopen(name, "w");
    if (logImg != NULL)
    {
      UTime t;
      t.setTime(regbot->info->bootTime);
      const int MSL = 50;
      char s[MSL];
      fprintf(logImg, "%% data log started at %s\n", t.getDateTimeAsString(s));
      fprintf(logImg, "%% 1 Time [sec]\n");
      fprintf(logImg, "%% 2 Regbot time [sec]\n");
      fprintf(logImg, "%% 3 image number\n");
      fprintf(logImg, "%% 4 image file name\n");
      fflush(logImg);
    }
    else
      printf("Failed to open image logfile\n");
  }
  else
    printf("Camera setup failed\n");
}

/** destructor */
UCamera::~UCamera()
{
  stop();
  if (logImg != NULL)
    fclose(logImg);
}



/**
  * Implementation of capture and timestamp image */
timeval UCamera::capture(cv::Mat &image)
{
  timeval imageTime;
#ifdef raspicam_CV_LIBS
  Camera.grab();
  gettimeofday(&imageTime, NULL);
  image.create(Camera.get(CV_CAP_PROP_FRAME_HEIGHT), Camera.get(CV_CAP_PROP_FRAME_WIDTH), CV_8UC3);
  Camera.retrieve ( image);
#else
  gettimeofday(&imageTime, NULL);
#endif
  return imageTime;
}

void UCamera::run()
{
  cv::Mat im, frame;
  cv::Mat im2;
  cv::Mat imd; // differense image
//   int lineState = 0;
//   UTime imTime, im2Time;
//   bool isOK = false;
  printf("# camera thread started\n");


  saveImage = false;
  calibrate = false;

  while (not th1stop)
  {
    // capture RGB image to a Mat structure
    imTime = capture(im);
    frame = im;
    int sum = 0; // of red
    int n = 0;
    for (int row = 2; row < im.rows; row+=55)
    {
      for (int col= 2; col < im.cols; col+=15)
      {
        n++;
        cv::Vec3b pix = im.at<cv::Vec3b>(row, col);
        sum += pix.val[2]; // format is BGR, so use red
      }
//       printf("# row=%d, n=%d sum=%d, avg=%d\n", row, n, sum, sum/n);
    }
    imgAverage = sum/n;
    if (saveImage)
    {
      printf("# saving - avg=%d\n", imgAverage);
      saveImageToFlash(im);
//       cv::imwrite("savedImage.png", im);
      saveImage = false;
      printf("Image saved\n");
    }
     usleep(10000);
  }
}


void UCamera::saveImageToFlash(cv::Mat& im)
{
  const int MNL = 100;
  char date[MNL];
  char name[MNL] = "savedImage.png";
  saveImage = false;
  // use date in filename
  // get date as string
  printf("# Trying to save\n");
//   float imgTime = regbot->info.getTime();
  imTime.getForFilename(date);
  // construct filename
  snprintf(name, MNL, "image_%s_%03d.png", date, imageNumber);
  // convert to RGB
  cv::cvtColor(im, im, cv::COLOR_BGR2RGB);
  // make PNG option - compression 9
  vector<int> compression_params;
  compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(9);
  // save image
  cv::imwrite(name, im, compression_params);
  // debug message
  printf("Saved image to: %s\n", name);
  if (logImg != NULL)
  { // save to image logfile
    fprintf(logImg, "%ld.%03ld %.3f %d '%s'\n", imTime.getSec(), imTime.getMilisec(), regbot->info->regbotTime, imageNumber, name);
    fflush(logImg);
  }
  imageNumber++;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////// ArUco functions ///////////////////7////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

int UCamera::startWebcamMonitoring()
{

  cv::Mat frame;
  const float arucoSqaureDimensions = 0.100;      //meters
  const float arucoSqaureDimensionsSmall = 0.020; //meters
  vector<int> markerIds;
  vector<vector<cv::Point2f>> markerCorners, rejectedcandidates;


  cv::aruco::DetectorParameters parameters;

  cv::Ptr < cv::aruco::Dictionary> markerDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_100); //seach DICT on docs.opencv.org


  //cv::namedWindow("Webcam", CV_WINDOW_AUTOSIZE);

  vector<cv::Vec3d> rotationVectors, translationVectors;
  cv::Vec3d rVec, tVec, rVec30, tVec30;

  string label2, label1, label;
  label =" aX= 0.00 aY= 0.00 aZ= 0.00 tX= 0.00 tY= 0.00 tZ= 0.00 ID      ";
  label1 = label;
  label2 = label;

  
  cout << "enter aruco" << "\n";
  vector<cv::Mat> savedImages;

  capture(frame);

  cv::aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds);
  cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
  cv::aruco::estimatePoseSingleMarkers(markerCorners, arucoSqaureDimensions, cameraMatrix, distanceCoefficients, rotationVectors, translationVectors);
  int i =0;
  for(unsigned int j = 0; j < markerIds.size(); j++)
  {
    i = markerIds[j];
    cout << "i: " << i << "  j: " << j << endl;
    arUcoVal[i].markerId = markerIds[j];
    arUcoVal[i].frame = frame; // might use frame.copyTo(arUcoVal[i].frame) insted
    arUcoVal[i].rVec = rotationVectors[j];
    arUcoVal[i].tVec = translationVectors[j];
    // scale down the small ArUco markers. (those which not are a whole multible of 10)
    if ( ((markerIds[j] % 10) != 0) )//  || ((markerIds[j] % 10) != 1))
    {   cout << "markerIds[j] % 10) != 0: " << markerIds[j] << endl;
        arUcoVal[i].tVec = translationVectors[j]* arucoSqaureDimensionsSmall/arucoSqaureDimensions;
    }
    cv::aruco::drawAxis(frame, cameraMatrix, distanceCoefficients, rotationVectors[j], arUcoVal[i].tVec, 0.03f); //X:red, Y:green, Z:blue.
    markerToRobotCoordinate(arUcoVal[i].rVec, arUcoVal[i].tVec, arUcoVal[i].rob_Ar_H);
    arUcoVal[i].label  = UCamera::extractSingleMarker(arUcoVal[i].rVec, arUcoVal[i].tVec, arUcoVal[i].markerId, arUcoVal[i].rob_Ar_H);
    arUcoVal[i]= saveMarkerUnitVec(arUcoVal[i]);
    arUcoVal[i]= UCamera::alignWithX(arUcoVal[i]);

    cout << arUcoVal[i].robotTurnCommand << endl;
    arUcoVal[i].disteance2marker = UCamera::distance(arUcoVal[i].rob_Ar_H);
    //UCamera::display3Labels(arUcoVal[i].label, arUcoVal[i].label, arUcoVal[i].label, arUcoVal[i].frame);
    arUcoVal[i].done = true;
  }

  
      

  //imshow("Webcam", frame);
  //cv::waitKey(2000);//1000 / 20);

  const int MNL = 100;
  char date[MNL];
  char name[MNL] = "ArUco_Image.png";  
  imTime.getForFilename(date);
  // construct filename
  //snprintf(name, MNL, "ArUco_Image%s_a.png", date); // uncomment this line to get a timestap in the file name, and not overwrite the old picture.
  std::string imageName = name;
  cout << imageName <<endl;
  cv::imwrite(imageName,frame); //save image from marker found 
  
  UCamera::logArUcoVal(arUcoVal);
  cout << "exit startWebcamMonitoring" << endl;
  return 1;

}

void UCamera::logArUcoVal(ArUcoVal_t tempArUcoVal[100])
{
  const int MNL = 100;
  char date[MNL];
  char name[MNL] = "ArUco_Log.txt";  
  imTime.getForFilename(date);
  // construct filename
  //snprintf(name, MNL, "log_ArUco%s_a.txt", date);// uncomment this line to get a timestap in the file name, and not overwrite the old log file.
  std::ofstream fout(name);
  for(unsigned int i = 0; i<100; i++)
  {
    fout << "i:" << i  << endl;
    fout << "markerId: " << tempArUcoVal[i].markerId << endl;
    fout << "rVec: \n" << tempArUcoVal[i].rVec << endl;
    fout << "tVec: \n" << tempArUcoVal[i].tVec << endl;
    fout << "label: " << tempArUcoVal[i].label << endl;
    fout << "robotTurnCommand: " << tempArUcoVal[i].robotTurnCommand << endl;
    fout << "robotDrive Command: " << tempArUcoVal[i].robotDriveCommand << endl;
    fout << "rob_Ar_H: \n" << tempArUcoVal[i].rob_Ar_H << endl;
    fout << "done: " << tempArUcoVal[i].done << endl;
    fout << "markerUnitVecX: \n" << tempArUcoVal[i].markerUnitVecX << endl; 
    fout << "markerUnitVecY: \n" << tempArUcoVal[i].markerUnitVecY << endl; 
    fout << "markerUnitVecZ: \n" << tempArUcoVal[i].markerUnitVecZ << endl;
    fout << "disteance2marker: " << tempArUcoVal[i].disteance2marker << endl;
    fout << "angle2markerX: " << tempArUcoVal[i].angle2markerX << endl;
    fout << "angle2markerY: " << tempArUcoVal[i].angle2markerY << endl;
    fout << "\n\n"  << endl;
  }
  fout.close();
  cout << name << endl;
}

void UCamera::display3Labels(string label, string label1, string label2, cv::Mat& frame)
{
  int baseline = 0;
  cv::Size textSize = getTextSize(label,
  cv::FONT_HERSHEY_PLAIN, // font face
  1.0, // font scale
  1, // thickness
  &baseline);
  // Draw the background rectangle.
  cv::rectangle(frame,
  cv::Point(10, 90), // lower left corner
  cv::Point(textSize.width + 10, 8),// upper right corner
  cv::Scalar(255, 255, 255), // color
  CV_FILLED); // Fill the rectangle
  putText(frame, label, cv::Point(10, 25),
  cv::FONT_HERSHEY_PLAIN, // font face
  1.0, // font scale
  cv::Scalar(0, 0, 0), // font color
  1); // thickness
  putText(frame, label1, cv::Point(10, 55),
  cv::FONT_HERSHEY_PLAIN, // font face
  1.0, // font scale
  cv::Scalar(0, 0, 0), // font color
  1); // thickness
  putText(frame, label2, cv::Point(10, 85),
  cv::FONT_HERSHEY_PLAIN, // font face
  1.0, // font scale
  cv::Scalar(0, 0, 0), // font color
  1); // thickness
}


string UCamera::extractSingleMarker(cv::Vec3d rotationVectors, cv::Vec3d translationVectors, int markerId, cv::Mat rob_Ar_H)
{
	std::string label;
  label =" aX= 0.00 aY= 0.00 aZ= 0.00 tX= 0.00 tY= 0.00 tZ= 0.00 ID   NaN   ";

	cv::Vec3d rVec = rotationVectors;
	cv::Vec3d tVec = translationVectors;

	vector<double> a = UCamera::calcVec(rob_Ar_H);
  rVec = rVec * 180/3.14159265;
	label =
	" aX=" + std::to_string(a[0]).substr(0, 5) +
	" aY=" + std::to_string(a[1]).substr(0, 5) +
	" aZ=" + std::to_string(a[2]).substr(0, 5) +
	" rX=" + std::to_string(rVec[0]).substr(0, 5) +
	" rY=" + std::to_string(rVec[1]).substr(0, 5) +
	" rZ=" + std::to_string(rVec[2]).substr(0, 5) +
	" tX=" + std::to_string(tVec[0]).substr(0, 5) +
	" tY=" + std::to_string(tVec[1]).substr(0, 5) +
	" tZ=" + std::to_string(tVec[2]).substr(0, 5) +
	" ID=" + std::to_string(markerId);

	return label;
}



// A function to convert the representation of an orientation from angle‐axis to XYZ angles.
vector<double> UCamera::calcVec(cv::Mat rob_Ar_H)
{
  cv::Mat R;
//  cv::Rodrigues(rvec, R); // convert to rotation matrix
  R = rob_Ar_H(cv::Rect(0,0,3,3));
  double ax, ay, az;
  ay = atan2(-R.at<double>(2, 0), pow(pow(R.at<double>(0, 0), 2) + pow(R.at<double>(1, 0), 2), 0.5));
  double cy = cos(ay);
  if (abs(cy) < 1e-9) 
  {
    // Degenerate solution.
    az = 0.0;
    ax = atan2(R.at<double>(0, 1), R.at<double>(1, 1));
    if (ay < 0) ax = -ax;
  } else 
  {
    az = atan2(R.at<double>(1, 0) / cy, R.at<double>(0, 0) / cy);
    ax = atan2(R.at<double>(2, 1) / cy, R.at<double>(2, 2) / cy);
  }
  // convert to degrees
  ax=ax* 180/3.14159265;
  ay=ay* 180/3.14159265;
  az=az* 180/3.14159265;

  vector<double> result;
  result.push_back(ax); result.push_back(ay); result.push_back(az);

  return result;
}

void UCamera::markerToRobotCoordinate(cv::Vec3d rVec, cv::Vec3d tVec, cv::Mat& rob_Ar_H)
{

	//combining rotation and translation Vector to a homegeneous transformation matrix
	cv::Mat R, camH;
	Rodrigues(rVec,R); 		 				                // 3 cols, 3 rows
	cv::Mat col = cv::Mat(tVec);  	              // 1 cols, 3 rows

	hconcat(R, col, camH);                        // 4 cols, 3 rows
	double tempRow[4] = {0,0,0,1};
	cv::Mat row = cv::Mat(1, 4, CV_64F, tempRow); // 4 cols, 1 row
	camH.push_back(row);                          // 4 cols, 4 rows

	//making a homegeneous transformation matrix from camera to robot robot_cam_H
	double_t tx 	= 0.158094; //meter
	double_t tz 	= 0.124882; //meter
  cv::Mat tranH = (cv::Mat_<double>(4,4) << 1,0,0,tx,    0,1,0,0,   0,0,1,tz,  0,0,0,1);

	double_t angle 	= 15+90; // degree positiv around xcam__axis
	double_t co 	= cos(angle* 3.14159265/180.0);
	double_t si 	= sin(angle* 3.14159265/180.0);
  cv::Mat rotxH = (cv::Mat_<double>(4,4) << 1,0,0,0,  0,co,-si,0,  0,si,co,0,  0,0,0,1);

  angle 	= 90; // 2nd rotation around temp zcam__axis
	co 	= cos(angle* 3.14159265/180.0);
	si 	= sin(angle* 3.14159265/180.0);
	tx 	= 0.158094; //meter
	tz 	= 0.124882; //meter
	cv::Mat rotzH = (cv::Mat_<double>(4,4) << co,-si,0,0,  si,co,0,0,  0,0,1,0,  0,0,0,1);

	cv::Mat rob_cam_H = tranH*rotzH*rotxH;
	rob_Ar_H = rob_cam_H*camH;
}

double UCamera::angleCalc( cv::Vec3d vec1)   
{
	double atan2Angle = atan2(vec1(1),vec1(2));
	return atan2Angle;
}

double UCamera::rad2deg( double a)
{
  return a*180/CV_PI;
}


 /* calculate the shortest distance to the marker */
double UCamera::distance(cv::Mat rob_Ar_H) 
{
  double sumOfSquares = pow(rob_Ar_H.at<double>(0,3), 2)+ pow( rob_Ar_H.at<double>(1,3), 2) +pow( rob_Ar_H.at<double>(2,3), 2);
  double squareroot   = pow(sumOfSquares,0.5);
  return squareroot;
}



UCamera::ArUcoVal_t UCamera::alignWithX(ArUcoVal_t tempArUcoVal)
{
  int x, y;
  int signX = 1;
  int signY = 1;
  cv::Mat aV;

  switch (tempArUcoVal.markerId % 10) {
    case 0://30: 
      x= 0;
      y= 2;
      signX = 1;
      signY = 1;
    break;
    case 1://31:
      x= 0;
      y= 2;
      signX = 1;
      signY = 1;
    break;
    case 3://33:
      // the x axis of 33 is parallel with the x-axis of 30
      // the z axis of 33 is parallel with the y-axis of 30  but other direction
      x= 0;
      y= 2;
      signX = +1;
      signY = -1;
    break;
    case 4://34:
      // the x axis of 34 is parallel with the x-axis of 30 but other direction
      // the z axis of 34 is parallel with the y-axis of 30
      x= 0;
      y= 2;
      signX = -1;
      signY = +1;
    break;
    case 5://35:
      // the z axis of 35 is parallel with the x-axis of 30
      // the x axis of 35 is parallel with the y-axis of 30
      x= 2;
      y= 0;
      signX = 1;
      signY = 1;
    break;
    case 6://36:
      // the z axis of 36 is parallel with the x-axis of 30  but other direction
      // the x axis of 36 is parallel with the y-axis of 30  but other direction
      x= 2;
      y= 0;
      signX = -1;
      signY = -1;
    break;
    default:
    std::cout << "Unknown marker: " << tempArUcoVal.markerId << '\n';
    return tempArUcoVal;
  }
  
  //calculate angle with markers X axis
  tempArUcoVal.rob_Ar_H.col(x).copyTo(aV); // extract column from rob_Ar_H  
  cv::Vec3d aV2((double*)aV.data);         // convert to a vector
  cv::Vec3d rV={1, 0, 0};
  rV = signX * rV;
  double agleXY  = angleCalc(aV2);//,rV);
	double agleXYd = rad2deg(agleXY);
  tempArUcoVal.angle2markerX = agleXY;

  //calculate angle with markers Y axis
  rV = signY * rV;
  tempArUcoVal.rob_Ar_H.col(y).copyTo(aV); // extract column from rob_Ar_H
  cv::Vec3d aV3((double*)aV.data);
  tempArUcoVal.angle2markerY = angleCalc(aV3);//,rV);

  // make string with command to robot 
  char buffer[100];
  string robotCmand;
  sprintf(buffer, "tr=0.05:turn= %.0f,time=10", agleXYd);
  tempArUcoVal.robotTurnCommand = buffer;

  return tempArUcoVal;
}

UCamera::ArUcoVal_t UCamera::saveMarkerUnitVec(ArUcoVal_t tempArUcoVal)
{
  cv::Mat aV;
  
  tempArUcoVal.rob_Ar_H.col(0).copyTo(aV);  // extract column from rob_Ar_H
  cv::Vec3d aVx((double*)aV.data);          // convert to a vector
  tempArUcoVal.markerUnitVecX = aVx;
  
  tempArUcoVal.rob_Ar_H.col(1).copyTo(aV);  // extract column from rob_Ar_H
  cv::Vec3d aVy((double*)aV.data);
  tempArUcoVal.markerUnitVecY=aVy;
  
  tempArUcoVal.rob_Ar_H.col(2).copyTo(aV);  // extract column from rob_Ar_H
  cv::Vec3d aVz((double*)aV.data);
  tempArUcoVal.markerUnitVecZ=aVz;

  return tempArUcoVal;
}


/////////////////////////////////// Calibration functions /////////////////////////////////////
bool UCamera::loadCameraCalibration(string name, cv::Mat& cameraMatrix, cv::Mat& distanceCoefficients)
{
  calibrate = false;
  ifstream inStream(name);
  if(inStream)
  {
    uint16_t rows;
    uint16_t columns;
    inStream >> rows;
    inStream >> columns;
    cameraMatrix = cv::Mat(cv::Size(columns, rows), CV_64F);

    for(int r = 0; r < rows; r++)
    {
      for(int c = 0; c < columns; c++)
      {
        double read = 0.0f;
        inStream >> read;
        cameraMatrix.at<double>(r,c) = read;
      }
    }
    //Disteance Coefficients
    inStream >> rows;
    inStream >> columns;

    distanceCoefficients = cv::Mat::zeros(rows,columns, CV_64F);


    for(int r = 0; r < rows; r++)
    {
      for(int c = 0; c < columns; c++)
      {
        double read = 0.0f;
        inStream >> read;
        distanceCoefficients.at<double>(r,c) = read;
      }
    }
    inStream.close();
    return true;
  }

  return false;

}

void UCamera::createKnownBoardPosition(cv::Size boardSize, float squareEdgeLength, vector<cv::Point3f>& corners)
{
  for(int i =0; i < boardSize.height; i++)
  {
    for(int j = 0; j < boardSize.width; j++)
    {
      corners.push_back(cv::Point3f(j* squareEdgeLength, i * squareEdgeLength, 0.0f));
    }
  }
}

void UCamera::getChessboardCorners(vector<cv::Mat> images, vector<vector<cv::Point2f>>& allFoundCorners, bool showResults)
{
   showResults = false;
    for( vector<cv::Mat>::iterator iter = images.begin(); iter != images.end(); iter++)
    {
        vector<cv::Point2f> pointBuf;
        bool found = findChessboardCorners( *iter, chessboardDimensions, pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
        if(found)
        {
            allFoundCorners.push_back(pointBuf);
        }
        if (showResults)
        {
            drawChessboardCorners(*iter, chessboardDimensions, pointBuf, found);
            imshow("Looking for Corners", *iter);
            cv::waitKey(0);
        }
    }
}

void UCamera::cameraCalibration(vector<cv::Mat> calibrationImages, cv::Size boardSize, float squareEdgeLength, cv::Mat& cameraMatrix, cv::Mat& distanceCoefficients)
{
    vector<vector<cv::Point2f>> checkerboardImageSpacePoints;
    getChessboardCorners(calibrationImages, checkerboardImageSpacePoints, false);

    vector<vector<cv::Point3f>> worldSpaceCornerPoints(1);

    createKnownBoardPosition(boardSize, squareEdgeLength, worldSpaceCornerPoints[0]);
    worldSpaceCornerPoints.resize(checkerboardImageSpacePoints.size(), worldSpaceCornerPoints[0]);

    vector<cv::Mat> rVectors, tVectors;
    distanceCoefficients = cv::Mat::zeros(8, 1, CV_64F);

    calibrateCamera(worldSpaceCornerPoints, checkerboardImageSpacePoints, boardSize, cameraMatrix, distanceCoefficients, rVectors, tVectors);
}

bool UCamera::saveCameraCalibration(string name, cv::Mat cameraMatrix,cv::Mat distanceCoefficients)
{
    ofstream outStream(name);
    if (outStream)
    {
        uint16_t rows = cameraMatrix.rows;
        uint16_t columns = cameraMatrix.cols;

        outStream << rows << endl;
        outStream << columns << endl;

        for(int r = 0; r< rows; r++)
        {
            for(int c =0; c< columns; c++)
            {
                double value = cameraMatrix.at<double>(r,c);
                outStream << value << endl;
            }
        }

        rows = distanceCoefficients.rows;
        columns = distanceCoefficients.cols;

        outStream << rows << endl;
        outStream << columns << endl;

         for(int r = 0; r< rows; r++)
        {
            for(int c =0; c< columns; c++)
            {
                double value = distanceCoefficients.at<double>(r,c);
                outStream << value << endl;
            }
        }

        outStream.close();
        return true;


    }
    return false;
}

void UCamera::cameraCalibrationProcess(cv::Mat& cameraMatrix,cv::Mat& distanceCoefficients)
{
  cv::Mat frame;
  cv::Mat drawToFrame;

    vector<cv::Mat> savedImages;

    vector<vector<cv::Point2f>> markerCorners, rejectedCandidates;


    int framesPerSecond = 20;

    cv::namedWindow("Webcam", CV_WINDOW_AUTOSIZE);
    while(true)
    {
        capture(frame);

        vector<cv::Vec2f> foundPoints;
        bool found = false;

        imshow("Webcam",frame);
        cv::waitKey(1000 / framesPerSecond);
        found = cv::findChessboardCorners(frame, chessboardDimensions, foundPoints, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
        frame.copyTo(drawToFrame);
        cv::drawChessboardCorners(drawToFrame, chessboardDimensions, foundPoints, found);
        if(found)
        {
            imshow("Webcam", drawToFrame);
        }
        else
            imshow("Webcam",frame);

        char character = cv::waitKey(1000 / framesPerSecond);

        switch(character)
        {
            case ' ':
                // saveing image
                if (found)
                {
                  cv::Mat temp;
                  frame.copyTo(temp);
                  savedImages.push_back(temp);
                  saveImageToFlash(temp);
                  cout << "Image saved"<< "\n";
                }
                break;
            case 13:
                // start calibration
                if (savedImages.size() >15)
                {
                cameraCalibration(savedImages, chessboardDimensions, calibrationSqaureDimensions, cameraMatrix, distanceCoefficients);
                saveCameraCalibration(calibrtionFileName, cameraMatrix, distanceCoefficients);
                cout << "calibarted and saved"<< "\n";
                }
                break;
            case 27:
                //exit
                cout << "Exit calibartion"<< "\n";
                return;
                break;
        }
    }
}




///////////////////////// End of calibration and AruCo functions  /////////////////////////////////////







// Read an image from the camera
// static cv::Mat UCamera::GetImageFromCamera(cv::VideoCapture& camera)
// {
//   cv::Mat frame;
//   camera >> frame;
//   return frame;
// }

// bool UCamera::imageLineProcess(cv::Mat * imOrg, cv::Mat * im2Org, UTime imTime, UTime im2Time)
// {
//   int w = imOrg->cols;
//   int h = imOrg->rows;
//   int stride = imOrg->step;
//   cv::Mat im;
//   cv::Mat im2;
//   cv::Mat dImg(h, w, CV_8UC1);
//   uint8_t * dst = dImg.data;
//   const int limit = 0;
//   printf("# image is %d x %d (stride=%d)\n", h, w, stride);
//   cv::cvtColor(*imOrg, im, cv::COLOR_RGB2BGR);
//   cv::cvtColor(*im2Org, im2, cv::COLOR_RGB2BGR);
//   int cmax = 0;
//   for (int row = h - h; row < h; row++)
//   {
//     dst = dImg.data + row * dImg.step;
//     uchar * px1 = im.data + row * stride;
//     uchar * px2 = im2.data + row * stride;
//     int v;
//     for (int col= 0; col < w; col++)
//     {
// //       cv::Vec3b pix1 = im->at<cv::Vec3b>(row, col);
// //       cv::Vec3b pix2 = im2->at<cv::Vec3b>(row, col);
//       // look in the red channel only (as line is red)
//       v = (px1[2] - px2[2] + px2[0] - px1[0]);
//       if (v < limit)
//         *dst = 0;
//       else if (v > 255+limit)
//         *dst = 255-limit;
//       else
//         *dst = v - limit;
//       if (*dst > cmax)
//         cmax = *dst;
//       dst++;
//       px2 += 3;
//       px1 += 3;
//     }
//     //       printf("# row=%d, n=%d sum=%d, avg=%d\n", row, n, sum, sum/n);
//   }
//   printf("# max value is %d\n", cmax);
//   cv::imwrite("savedDiffImgRaw1.png", im);
//   cv::imwrite("savedDiffImg.png", dImg);
//   cv::imwrite("savedDiffImgRaw2.png", im2);
//   // save array of points
//   cv::Mat d2, dla, th, th2;
//   cv::pyrDown(dImg, d2);
//   cv::Laplacian(d2, dla,CV_8U);
//   cv::imwrite("savedLaplacian.png", dla);
//   // skip the laplacian
//   cv::threshold(d2, th, cmax/20+5, 255, 0);
//   cv::imwrite("savedThreshold.png", th);
//   //
//   if (false)
//   {
//     int erosion_size = 1;
//     cv::Mat element = cv::getStructuringElement(0, cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
//                                                   cv::Point( erosion_size, erosion_size) );
//     cv::erode(th, th2, element);
//     cv::imwrite("savedThreshold2.png", th2);
//   }
//   else
//     th2 = th;
//   //
//   std::vector<cv::Point> linePoints;
//   linePoints.reserve(1000);
//   int rowStart = (th2.rows * 700) / dImg.rows;
//   for (int row = rowStart; row < th2.rows; row ++)
//   { // skip the first rows
//     dst = th2.data + row * th2.step;
//     for (int col= 0; col < th2.cols; col++)
//     {
//       if (*dst++)
//       {
//         linePoints.push_back(cv::Point(col, row));
//       }
//     }
// //     printf("# row %d pixels %d\n", row, linePoints.size());
//   }
//   printf("# found pixels %d\n", linePoints.size());
//   // first guess
//   {
//     U2Dline line1;
//     line1.set(linePoints);
//     cv::Point p1(0, cvRound(2.0 * line1.getY(0)));
//     cv::Point p2(th2.cols*2 - 1, cvRound(2.0 * line1.getY(th2.cols - 1)));
//     cv::line(im , p1, p2, cv::Scalar(255,0,0), 1, 8 );
//   }
//   // improve line
//   U2Dline mainLine;
//   mainLine = findMainLine(linePoints);
//   {
//     // get x,y points across image (with twice the resolution)
//     cv::Point p1(0, cvRound(2.0 * mainLine.getY(0)));
//     cv::Point p2(th2.cols*2 - 1, cvRound(2.0 * mainLine.getY(th2.cols - 1)));
//     printf("# main line from %d,%d to %d,%d\n", p1.x, p1.y, p2.x, p2.y);
//     cv::line(im , p1, p2, cv::Scalar(0,255,0), 3, 8 );
//   }
//   cv::imwrite("savedDiffImgRaw2ann.png", im);
//   //
//   if (false)
//   {
//     printf("# image max value is %d\n", cmax);
//     cv::threshold(dImg, th, cmax/10, 255, 0);
//     cv::imwrite("savedThreshold.png", th);
//     std::vector<cv::Vec2f> lines;
//     cv::HoughLines(th, lines, 2, 2*M_PI/180, 600);
//     printf("# hough found %d lines\n", lines.size());
//     int c = 255;
//     int lw = 3;
//     for( size_t i = 0; i < lines.size(); i++ )
//     {
//       float rho = lines[i][0];
//       float theta = lines[i][1];
//       printf("# line %d is rho=%g, theta=%g\n", i, rho, theta);
//       double a = cos(theta), b = sin(theta);
//       double x0 = a*rho, y0 = b*rho;
//       cv::Point pt1(cvRound(x0 + 2000*(-b)),
//                 cvRound(y0 + 2000*(a)));
//       cv::Point pt2(cvRound(x0 - 2000*(-b)),
//                 cvRound(y0 - 2000*(a)));
//       cv::line(im , pt1, pt2, cv::Scalar(c,255- c,0), lw, 8 );
//       if (i > 0)
//         break;
//       if (c > 30)
//         c-=10;
//       if (lw > 1)
//         lw--;
//     }
//     cv::imwrite("savedDiffImgRaw2ann.png", im);
//     printf("# image line done\n");
//     // detect edges
//     cv::Mat edg, blr;
//   //   cv::blur(dImg, blr, cv::Size(3,3));
//   //   cv::Canny(blr, edg, 50, 100);
//   //   cv::imwrite("savedBlur.png", blr);
//   //   cv::imwrite("savedCanny.png", edg);
//   }
//   return true;
// }
//
//
// U2Dline UCamera::findMainLine(std::vector<cv::Point> &linePoints)
// {
//   U2Dline line;
//   line.set(linePoints);
//   std::vector<cv::Point> pNear;
//   pNear.reserve(500);
//   float d;
//   line.print((char*)"main line");
//   for (int i = 0; i < (int)linePoints.size(); i++)
//   { // test if they are near, then again
//     // fit line and iterate until fine line
//     // maybe prefer higher rather than læower distance
//     cv::Point v = linePoints.at(i);
//     d = line.distanceSigned(v.x, v.y);
//     if (d < 1 and d > -50)
//     {
//       pNear.push_back(v);
// //       printf("# ok at %d,%d (%g pixels)\n", v.x, v.y, d);
//     }
//   }
//   printf("# reduced line points from %d to %d\n", linePoints.size(), pNear.size());
//   line.set(pNear);
//   return line;
// }


#endif
