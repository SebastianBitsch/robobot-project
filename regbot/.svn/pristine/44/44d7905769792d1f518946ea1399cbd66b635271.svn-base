/**
 * test for automatic path generation
 * 
 * initially divided into straight and circles
 * */


#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <fstream>
#include "u2dline.h"



///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

class UPoint
{
public:
  double x, y, t;
  //
  UPoint()
  {
    x = 0;
    y = 0;
    t = 0;
  }
  UPoint(double ix, double iy)
  {
    x = ix;
    y = iy;
    t = 0;
  }
  
  double dist(UPoint * p2)
  {
    double result = hypot(p2->x - x, p2->y - y);
    return result;
  }
};

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////


typedef std::vector<UPoint> PathVector;

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////


class UPath
{
public:
  PathVector path;
  double startTime;
  /**
   * constructor */
  UPath(int size)
  {
    path.reserve(size);
  }
  /** load odo from file */
  bool load(std::string filename, int ix, int iy, int first = 0, int last = 999999)
  {
    std::ifstream odofile(filename.c_str());
    std::string line;
    int maxIdx = ix;
    if (iy > ix)
      maxIdx = iy;
    int lineNum = 0;
    if (odofile.is_open())
    {
      printf("#is open\n");
      while(getline(odofile, line))
      {
        lineNum++;
        if (line.size() > 10)
        {
          if (isdigit(line[0]) or line[0] == '-' or line[0] == '+')
          {
            const char *p0, *p1;
            char *p2;
            UPoint v;
            double a;
            p0 = line.data();
            p1 = p0;
            int j = 1;
            if (lineNum == 1)
            {
              startTime = strtod(p1, &p2);
              printf("#starttime %g\n", startTime);
            }
            else
            {
              v.t = strtod(p1, &p2) - startTime;
            }
            while (p2 > p1 and (p2-p0) <line.size())
            {
              j++;
              p1 = p2;
              a = strtod(p1, &p2);
              if (j == ix)
                v.x = a;
              else if (j == iy)
                v.y = a;
              if (j >= maxIdx)
                break;
            }
            if (lineNum >= first)
            {
              path.push_back(v);
//               printf("# got line %d is t=%.3f, x=%.2f, y=%.2f\n", lineNum, v.t, v.x, v.y);
            }
            if (lineNum >= last)
              break;
          }
        }
      }
    }
    else
    {
      printf("# file '%s' not found\n", filename.c_str());
    }
    return odofile.is_open();
  }
  /**
   * save (used) points to a file */
  void savePoints()
  {
    FILE * fil;
    fil = fopen("points.txt","w");
    if (fil != NULL)
    { // write all positions
      fprintf(fil, "%% reduced line points\n");
      fprintf(fil, "%% 1: Timestamp\n");
      fprintf(fil, "%% 2: x\n");
      fprintf(fil, "%% 3: y\n");
      for (int i = 0; i < path.size(); i++)
      {
        UPoint a = path[i];
        fprintf(fil, "%.3f %.3f %.3f\n", a.t, a.x, a.y);
      }
      fclose(fil);
    }
  }
};

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

/**
 * circle segments */

class UArc
{
public:
  UPoint cent;
  double radius;
  UPoint pStart;
  UPoint pEnd;
  double angle; // in radians
  
public:
  /**
   * set circle parameters from 3 points
   * \param p1 = start point of arc
   * \param p2 = end point of arc
   * \param p2 = a third point on arc (e.g. between p1 and p2) */
  void setFrom3Points(UPoint * p1, UPoint * p2, UPoint * p3)
  { // save start and end points
    pStart = *p1;
    pEnd = *p2;
    // code copy from URL http://forums.codeguru.com/showthread.php?442320-draw-circle-from-3-points
    double ax,ay,bx,by,cx,cy,x1,y11,dx1,dy1,x2,y2,dx2,dy2;
    double ox,oy,dx,dy; // Variables Used and to Declared
    ax =p1->x ; 
    ay = p1->y; //first Point X and Y
    bx =p2->x; 
    by = p2->y; // Second Point X and Y
    cx =p3->x ; 
    cy =p3->y; // Third Point X and Y
    ////****************Following are Basic Procedure**********************/
    x1 = (bx + ax) / 2;
    y11 = (by + ay) / 2;
    dy1 = bx - ax;
    dx1 = -(by - ay);
    //***********************
    x2 = (cx + bx) / 2;
    y2 = (cy + by) / 2;
    dy2 = cx - bx;
    dx2 = -(cy - by);
    //****************************
    ox = (y11 * dx1 * dx2 + x2 * dx1 * dy2 - x1 * dy1 * dx2 - y2 * dx1 * dx2)/ (dx1 * dy2 - dy1 * dx2);
    oy = (ox - x1) * dy1 / dx1 + y11;
    //***********************************
    dx = ox - ax;
    dy = oy - ay;
    // save result
    radius = sqrt(dx * dx + dy * dy);   
    cent.x = ox;
    cent.y = oy;
  }
  
  /**
   * distance from circle 
   * \param p1 is point to test
   * \returns distance from circle - positive is outside */
  double distSigned(UPoint * p1)
  {
    double d = cent.dist(p1) - radius;
  }
  /**
   * calculate angle */
  void calcAngle()
  {
    if (radius < 1e4)
    {
      double a = pStart.dist(&pEnd);
      double b = pStart.dist(&cent);
      double c = pEnd.dist(&cent);
      double ca = (sqr(b) + sqr(c) - sqr(a))/(2* b * c);
      angle = acos(ca);
    }
    else
      angle = 0;
  }
  
  /**
   * test circles */
  void testCircles()
  {
    UPoint p1(1,-6);
    UPoint p2(2,1);
    UPoint p3(5,2);
    // c=(5,-3), r=5
    setFrom3Points(&p1, &p2, &p3);
  }
};


///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

class USegments
{
public:
  U2Dlined l1;
  std::vector<int> vip;
  double maxDev = 0.1;
  std::vector<UArc> arcs;
  PathVector pt;
  
  USegments()
  {
    vip.reserve(1000);
    arcs.reserve(1000);
  }
  /**
   * Set points on path */
  void setPath(PathVector path)
  {
    pt = path;
  }
  /**
   * Segment path into lines
   * find line from odometry path */
  void segment(double distLimit)
  {
    int i = 0;
    int idxp = 0; // index to point
    int dmaxIdx = 0;
    vip.push_back(idxp);
    UPoint p1 = pt[i];
    i += 2;
    UPoint p2 = pt[i], *pk;    
    while (i < pt.size() - 1)
    {
      double d, dmax = 0;
      int k;
      UPoint * a;
      l1.set2P(p1.x, p1.y, p2.x, p2.y);
      bool overLimit = false;
      for (k = idxp + 1; k < i; k++)
      { // test for too much curvature
        a = &pt[k];
        d = l1.distanceSigned(a->x, a->y);
        if (fabs(d) > dmax)
        {
          dmax = fabs(d);
          pk = a;
          dmaxIdx = k;
        }
        if (fabs(d) > distLimit)
        {
          overLimit = true;
          break;
        }
        //printf("# line %d, k=%d, dist=%.3f\n", int(vip.size()), k, d, l);
      }
      if (overLimit)
      { // too long line break
        printf("# add line %d, i=%d, d=%.3f, p1=(%.3f, %.3f), p2=(%.3f, %.3f), pk=(%.3f, %.3f, idx=%d,%d,k=%d)\n",
               (int)vip.size(), i, dmax, p1.x, p1.y, p2.x, p2.y, pk->x, pk->y, dmaxIdx, overLimit, k);
        vip.push_back(i);
        idxp = i;
        p1 = p2;
        i += 2;
        p2 = pt[i];
      }
      else
      { // try advancing end point
        i++;
        p2 = pt[i];
      }
//       if (i %100 == 0)
//       {
//         printf("# line=%d, i= %d, idxp=%d, dmax=%.3f, p1=(%.3f, %.3f), p2=(%.3f, %.3f), pk=(%.3f, %.3f, idx=%d, %d, k=%d)\n", 
//           (int)vip.size(), i, idxp, dmax, p1.x, p1.y, p2.x, p2.y, pk->x, pk->y, dmaxIdx, overLimit, k);
//       }
    }
    vip.push_back(i);
    FILE * fil;
    fil = fopen("line-points.txt","w");
    if (fil != NULL)
    { // write all positions
      fprintf(fil, "%% reduced line points\n");
      fprintf(fil, "%% 1: Timestamp\n");
      fprintf(fil, "%% 2: x\n");
      fprintf(fil, "%% 3: y\n");
      fprintf(fil, "%% 4: index\n");
      for (int i = 0; i < vip.size(); i++)
      {
        int j = vip[i];
        UPoint a = pt[j];
        fprintf(fil, "%.3f %.3f %.3f %d\n", a.t, a.x, a.y, j);
      }
      fclose(fil);
    }
  }
  /**
   * Estimate circle from points between i1 and i2 */
  void toCircleOne(UArc * c1, int i1, int i2)
  {
    UPoint * p1, *p2, *p3;
    p1 = &pt[i1];
    p2 = &pt[i2];
    int idx2 = (i1 + i2)/2;
    p3 = &pt[idx2];
    U2Dlined ln;
    ln.set2P(p1->x, p1->y, p2->x, p2->y);
    double d = ln.distanceSigned(p3->x, p3->y);
    c1->cent.t = p1->t;
    if (fabs(d) < 0.002)
    {
      c1->radius = 1e10;
      c1->cent.x = 0;
      c1->cent.y = 0;
    }
    else
    {
      c1->setFrom3Points(p1, p2, p3);
//       printf("# arc (%d-%d), radius=%.3f, cent=(%.3f, %.3f), p3=(%.3f, %.3f)\n", 
//              i1, i2, c1->radius, c1->cent.x, c1->cent.y, p3->x, p3->y);
    }
  }
  /**
   * test segments into circles */
  void toCircles(double distLimit)
  {
    UArc c1;
    UPoint * p1, *p2, *p3;
    int i1, i2;
    arcs.clear();
    for (int i = 0; i < (vip.size() - 1); i++)
    {
      i1 = vip[i];
      i2 = vip[i+1];
      toCircleOne(&c1, i1, i2);
      arcs.push_back(c1);
    }
    printf("finished arcs\n");
  }
  /**
   * */
  double arcVar(int idx)
  {
    int i1 = vip[idx], i2 = vip[idx+1];
    UPoint * p1;
    int n = i2 - i1;
    double d2 = 0;
    UArc * arc = &arcs[idx];
    for (int i = i1; i < i2; i++)
    {
      p1 = &pt[i];
      d2 += sqr(arc->distSigned(p1));
    }
    double v2 = d2/double(n);
    return v2;
  }
  /**
   * */
  double lineVar(int idx)
  {
    int i1 = vip[idx], i2 = vip[idx+1];
    UPoint * p1 = &pt[i1];
    UPoint * p2 = &pt[i2];
    int n = i2 - i1;
    double d2 = 0;
    U2Dlined * l1;
    l1->set2P(p1->x, p1->y, p2->x, p2->y);
    for (int i = i1; i < i2; i++)
    {
      p1 = &pt[i];
      d2 += l1->distanceSigned(p1->x, p1->y);
    }
    double v2 = d2/double(n);
    return v2;
  }
  /**
   * get segment variance
   * */
  void getVariances()
  {
    printf("# %d points, %d arcs\n", (int)vip.size(), (int)arcs.size());
    for (int i = 0; i < arcs.size(); i++)
    {
      double vc, vl;
      vc = arcVar(i);
      vl = lineVar(i);
      UArc * arc = &arcs[i];
      int i1 = vip[i];
      int i2 = vip[i+1];
      printf("# seg %d-%d (%d-%d (%d)), linevar=%g, arcvar=%g, radius=%.2f\n", 
             i, i+1, i1, i2, i2-i1, vl, vc, arc->radius);
    }
  }
  /**
   * save circles for matlab */
  void saveCircles()
  {
    FILE * fil;
    fil = fopen("circles.txt","w");
    fprintf(fil, "%% 1,2: time and duration\n");
    fprintf(fil, "%% 3,4: p1\n");
    fprintf(fil, "%% 5,6  p2\n");
    fprintf(fil, "%% 7,8  center\n");
    fprintf(fil, "%% 9    radius\n");
    fprintf(fil, "%% 10   angle in degrees\n");
    for (int i = 0; i < arcs.size(); i++)
    {
      UArc * arc = &arcs[i];
      arc->calcAngle();
      fprintf(fil, "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.2f %.4f\n",
             arc->pStart.t, arc->pEnd.t - arc->pStart.t, 
             arc->pStart.x, arc->pStart.y, 
             arc->pEnd.x, arc->pEnd.y, 
             arc->cent.x, arc->cent.y, arc->radius, arc->angle*180/M_PI);
    }
    fclose(fil);
  }
  
  /**
   * optimize points 
   * - that is move endpoint for this segment to a better variance */
  double optimize(int idx)
  {
//     p1 = &pt[vip[i]];
//     p2 = &pt[vip[i+1]];
//     p3 = &pt[vip[i+2]];
//     UArc arc1 = arcs[idx];
//     UArc arc2 = arcs[idx+1];
//     UPoint * p1 = &pt[i1];
//     UPoint * p2 = &pt[i2];
//     UPoint * p3 = &pt[i3];
//     U2Dlined * l1;
//     l1->set2P(p1->x, p1->y, p2->x, p2->y);
//     U2Dlined * l2;
//     l1->set2P(p2->x, p2->y, p3->x, p3->y);
    int i0 = vip[idx];
    double vc1[3]; // index 0 = current, 1=idx reduced, 2= idx increased
    double vc2[3];
    double vl1[3];
    double vl2[3];
    double varsum = 0;
    // udregn varians
    vc1[0] = fabs(arcVar(idx-1));
    vc2[0] = fabs(arcVar(idx));
    vl1[0] = fabs(lineVar(idx-1));
    vl2[0] = fabs(lineVar(idx));
    if (vip[idx] - vip[idx-1] > 5)
      vip[idx] = i0 - 3;
    else
      vip[idx] = i0;
    // udregn ny arc for inx-1 og idx
    UArc * arc1 = &arcs[idx-1];
    UArc * arc2 = &arcs[idx];
    // use radius befor move
    bool c1 = arc1->radius < 20;
    bool c2 = arc2->radius < 20;
    //
    toCircleOne(arc1, vip[idx-1], vip[idx]);
    toCircleOne(arc2, vip[idx], vip[idx+1]);
    // udregn varians
    vc1[1] = fabs(arcVar(idx-1));
    vc2[1] = fabs(arcVar(idx));
    vl1[1] = fabs(lineVar(idx-1));
    vl2[1] = fabs(lineVar(idx));
    // move the other way
    if (vip[idx+1] - vip[idx] > 5)
      vip[idx] = i0 + 3;
    else
      vip[idx] = i0;
    // udregn ny arc for inx-1 og idx
    toCircleOne(arc1, vip[idx-1], vip[idx]);
    toCircleOne(arc2, vip[idx], vip[idx+1]);
    // udregn varians
    vc1[2] = fabs(arcVar(idx-1));
    vc2[2] = fabs(arcVar(idx));
    vl1[2] = fabs(lineVar(idx-1));
    vl2[2] = fabs(lineVar(idx));
    int direction = 0;
    // test if down is good
    double difDown1;
    double difDown2;
    const char * type = "";
    if (c1 and c2)
    { // both circles
      difDown1 = vc1[1] - vc1[0];
      difDown2 = vc1[2] - vc1[0];
      if (difDown1 < 0 and difDown2 < 0)
        direction = -3;
      else if (difDown1 * difDown2 < 0)
      { // one is positive
        direction = -1;
      }
      type = "cc";
    }
    else if (c1)
    { // c1 is arc and 'c2' is not arc
      difDown1 = vc1[1] - vc1[0];
      difDown2 = vl1[2] - vl1[0];
      if (difDown1 < 0 and difDown2 < 0)
        direction = -3;
      else if (difDown1 * difDown2 < 0)
        direction = -1;
      type = "cl";
    }
    else if (c2)
    { // c1 is not arcm c2 is
      difDown1 = vl1[1] - vl1[0];
      difDown2 = vc1[2] - vc1[0];
      if (difDown1 < 0 and difDown2 < 0)
        direction = -3;
      else if (difDown1 * difDown2 < 0)
        direction = -1;
      type = "lc";
    }
    else
    { // both are lines
      difDown1 = vl1[1] - vl1[0];
      difDown2 = vl1[2] - vl1[0];
      if (difDown1 < 0 and difDown2 < 0)
        direction = -3;
      else if (difDown1 * difDown2 < 0)
        direction = -1;
      type = "ll";
    }
    double difUp1;
    double difUp2;
    if (abs(direction) <=1)
    { // may be goint up
      if (c1 and c2)
      { // both circles
        difUp1 = vc2[1] - vc2[0];
        difUp2 = vc2[2] - vc2[0];
        if (difUp1 < 0 and difUp2 < 0)
          direction = 3;
        else if (difUp1 * difUp2 < 0)
        { // one is positive
          if (difUp1 - difDown1 + difUp2 - difDown2 < 0)
            // better to go up
            direction = 1;
        }
      }
      else if (c1)
      { // c1 is arc and 'c2' is not arc
        difUp1 = vc2[1] - vc2[0];
        difUp2 = vl2[2] - vl2[0];
        if (difUp1 < 0 and difUp2 < 0)
          direction = 3;
        else if (difUp1 * difUp2 < 0)
        {
          if (difUp1 - difDown1 + difUp2 - difDown2 < 0)
            // better to go up
            direction = 1;
        }
      }
      else if (c2)
      { // c1 is not arcm c2 is
        difUp1 = vl2[1] - vl2[0];
        difUp2 = vc2[2] - vc2[0];
        if (difUp1 < 0 and difUp2 < 0)
          direction = 3;
        else if (difUp1 * difUp2 < 0)
        {
          if (difUp1 - difDown1 + difUp2 - difDown2 < 0)
            // better to go up
            direction = 1;
        }
      }
      else
      { // both are lines
        difUp1 = vl2[1] - vl2[0];
        difUp2 = vl2[2] - vl2[0];
        if (difUp1 < 0 and difUp2 < 0)
          direction = 3;
        else if (difUp1 * difUp2 < 0)
        {
          if (difUp1 - difDown1 + difUp2 - difDown2 < 0)
            // better to go up
            direction = 1;
        }
      }
    }
    if (direction == 0)
    {
      vip[idx] = i0;
      // udregn ny arc for inx-1 og idx
      toCircleOne(arc1, vip[idx-1], vip[idx]);
      toCircleOne(arc2, vip[idx], vip[idx+1]);
      printf("# idx %d is still (%s)\n", idx, type);
      varsum = vc1[0] + vc2[0] + vl1[0] + vl2[0];
    }
    else if (direction < 0)
    {
      vip[idx] = i0 - 3;
      // udregn ny arc for inx-1 og idx
      toCircleOne(arc1, vip[idx-1], vip[idx]);
      toCircleOne(arc2, vip[idx], vip[idx+1]);
      printf("# idx %d (%d - %d - %d) is going down to %d (%s)\n", idx, vip[idx-1], i0, vip[idx+1], vip[idx], type);
      varsum = vc1[1] + vc2[1] + vl1[1] + vl2[1];
    }
    else
    {
      printf("# idx %d (%d - %d - %d) is going up   to %d (%s)\n", idx, vip[idx-1], i0, vip[idx+1], vip[idx], type);
      varsum = vc1[2] + vc2[2] + vl1[2] + vl2[2];
    }
    return varsum;
  }
  
  /** optimize segment borders */
  void optimizeAll()
  {
    NB! virker ikke endnu
    - fejl i var udregning
    for (int j = 0; j< 5; j++)
    {
      double varSum = 0;
      for (int i = 1; i < arcs.size(); i++)
      {
        varSum += optimize(i);
      }
      printf("##### iter %d varsum=%f\n", j, varSum);
    }
  }
  /**
   * class end */
};

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////


int main(int argc, char **argv) 
{
  int startIdx = 35000;
  int endIndex= 45000;
  UPath p(endIndex - startIdx + 2);
  USegments sm;
  p.load("../mrc_log_20190913.log", 2, 3, startIdx, endIndex);
  printf("# found %d points\n", int(p.path.size()));
  p.savePoints();
  //
  const int MSL = 100;
  char s[MSL];
  int m;
  // debug
  printf("# hit enter\n");
  std::cin >> m;
  // debug end
  sm.setPath(p.path);
  printf("# starting segmenting to lines\n");
  sm.segment(0.2);
  printf("# starting segmenting to circles\n");
  sm.toCircles(0.2);
  printf("# calculate statistics\n");
  sm.getVariances();
  sm.saveCircles();
  //printf("circle test\n");
  //UArc c1;
  //c1.testCircles();
  printf("#Optimize\n");
  sm.optimizeAll();
  printf("# all done\n");
  
  return 0;
}
