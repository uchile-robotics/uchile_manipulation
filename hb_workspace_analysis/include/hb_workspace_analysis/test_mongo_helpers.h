#include <geometry_msgs/Pose.h>
#include <cstring>
 
const double TOL=1e-3;
using std::ostream;

namespace geometry_msgs
{

inline bool operator==(const Pose& p1, const Pose& p2)
{
  const Point& pos1 = p1.position;
  const Point& pos2 = p2.position;
  const Quaternion& q1 = p1.orientation;
  const Quaternion& q2 = p2.orientation;
  return (pos1.x == pos2.x) && (pos1.y == pos2.y) && (pos1.z == pos2.z) &&
    (q1.x == q2.x) && (q1.y == q2.y) && (q1.z == q2.z) && (q1.w == q2.w);
}

}

geometry_msgs::Quaternion createQuaternionMsgFromYaw(double yaw)
{
  geometry_msgs::Quaternion q;
  q.w = cos(yaw/2);
  q.z = sin(yaw/2);
  q.x = 0;
  q.y = 0;
  return q;
}


inline geometry_msgs::Pose makePose(const double x, const double y, const double theta)
{
  geometry_msgs::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.orientation = createQuaternionMsgFromYaw(theta);
  return p;
}

using std::string;

bool contains (const string& s1, const string& s2)
{
  return strstr(s1.c_str(), s2.c_str())!=NULL;
}

