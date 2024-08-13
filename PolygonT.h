#include <array>
#include <string>
#include <cstring>
#include <algorithm>
#include <cmath>
#define POLYGONINOUTSTATUS(code) \
  code(UNKNOWN) code(InPolygon) code(OnPolygonEdge) code(OutsidePolygon)

enum class PolygonTestResult {
#define ENUM_ITEM(x) x,
  POLYGONINOUTSTATUS(ENUM_ITEM)
#undef ENUM_ITEM
    STATECOUNT
};

//read like:
//enum class PolygonTestResult {
//    UNKNOWN,
//    InPolygon,
//    OnPolygonEdge,
//    OutsidePolygon,
//    STATECOUNT // total number of items
//};


const std::array<std::string,static_cast<int>(PolygonTestResult::STATECOUNT)> _enumItemStrings = {{
#define ITEM_STRING(x) #x,
  POLYGONINOUTSTATUS(ITEM_STRING)
#undef ITEM_STRING
}};


// Function to print the points of a polygon
template<typename POINTARRAY>
auto PrintPolygon( const POINTARRAY &polygon) ->void
{
  using POINTTYPE = typename POINTARRAY::value_type;
  std::for_each( polygon.cbegin(), polygon.cend(), [](const POINTTYPE & point ) { std::cout << point << ", ";});
}

////////////////////////////
//to check if a type T has member x
template<class T, class = void>
struct has_member_x : std::false_type {};

//It uses std::void_t to test if decltype(std::declval<T&>().x) is valid
template<class T>
struct has_member_x<T, std::void_t<decltype(std::declval<T&>().x)>> : std::true_type {};

//to check if a type T has method X()
template<class T, class = void>
struct has_member_X : std::false_type {};

template<class T>
struct has_member_X<T, std::void_t<decltype(std::declval<T&>().X())>> : std::true_type {};

//to check if a type T can be accessed via operator[]
template<class T, class = void>
struct has_bracket : std::false_type {};

template<class T>
struct has_bracket<T, std::void_t<decltype(std::declval<T&>()[0])>> : std::true_type {};

//  getX and getY to select the appropriate way to access coordinates from different types
template <typename T>
auto getX(const T& obj) -> std::enable_if_t<has_member_x<T>::value, decltype(obj.x)> {
  return obj.x;
}

template <typename T>
auto getX(const T& obj) -> std::enable_if_t<has_member_X<T>::value, decltype(obj.X())> {
  return obj.X();
}

template <typename T>
auto getX(const T& obj) -> std::enable_if_t<has_bracket<T>::value, decltype(obj[0])> {
  return obj[0];
}

template <typename T>
auto getY(const T& obj) -> std::enable_if_t<has_member_x<T>::value, decltype(obj.y)> {
  return obj.y;
}

template <typename T>
auto getY(const T& obj) -> std::enable_if_t<has_member_X<T>::value, decltype(obj.Y())> {
  return obj.Y();
}

template <typename T>
auto getY(const T& obj) -> std::enable_if_t<has_bracket<T>::value, decltype(obj[1])> {
  return obj[1];
}

//////////
//
// Potencially needed function
// Function to compute the intersection of two line segments
// Using parametric form of the lines: p = (1-t)p1 + tp2, q = (1-u)q1 + uq2
// Intersection occurs if 0 <= t, u <= 1

template<typename POINTTYPE>
auto SegmentIntersection(const POINTTYPE &seg1start, const POINTTYPE &seg1end, const POINTTYPE &seg2start, const POINTTYPE &seg2end,
    std::vector<POINTTYPE> &intersections) -> void
{   // TODO: return the intersection(s) of 2 line segments defined by seg1start, seg2end and seg2start, seg2end
  // If the line segments overlapped, the 2 end points of the overlapped segment shall be returned
  
  // Get coordinates of the segment endpoints
  //auto x1 = getX(seg1start), y1 = getY(seg1start);
  //auto x2 = getX(seg1end), y2 = getY(seg1end);
  //auto x3 = getX(seg2start), y3 = getY(seg2start);
  //auto x4 = getX(seg2end), y4 = getY(seg2end);
  POINTTYPE d1 = seg1start - seg1end; //1-2
  POINTTYPE d2 = seg2start - seg2end; //3-4
  // Compute the determinant
  //auto denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
  auto denom = getX(d1) * getY(d2) - getY(d1) * getX(d2);
  if (denom == 0) {
    // Lines are parallel, coincident, or the calculation is numerically unstable
    return;
  }

  
  // Calculate intersection factors t and u
  // t and u are the factors in the parametric line equations where the intersection occurs
  //auto t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / static_cast<double>(denom);
  //auto u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / static_cast<double>(denom);
  auto dstart = seg1start - seg2start;
  //auto dend = seg1end - seg2end;
  auto t = (getX(dstart) * getY(d2) - getY(dstart) * getX(d2))/static_cast<double>(denom);
  auto u = -(getX(d1) * getY(dstart) - getY(d1) * getX(dstart))/static_cast<double>(denom);
  //auto t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / static_cast<double>(denom);
  //auto u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / static_cast<double>(denom);

  if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
    // Intersection point
    auto ix = getX(seg1start) - t * getX(d1);
    auto iy = getY(seg1start) - t * getY(d1);
    intersections.emplace_back(ix, iy);
  }
  
  //if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
  // Intersection point
  //auto ix = x1 + t * (x2 - x1);
  //auto iy = y1 + t * (y2 - y1);
  //intersections.emplace_back(ix, iy);
  //}
  //Note: If POINTTYPE is int, the intersection coordinates will be integers. Consider using a floating point representation if precision is needed.
  //May consider:  Specialization for POINTTYPE = int
  //auto SegmentIntersection(const POINTTYPE &seg1start, const POINTTYPE &seg1end, const POINTTYPE &seg2start, const POINTTYPE &seg2end,
  //std::vector<std::pair<double, double>> &intersections) -> void
}

template<typename T>
bool isAlmostEqual(T a, T b, T epsilon = std::numeric_limits<T>::epsilon() * 100) {
    return std::fabs(a - b) <= epsilon;
}



// Polygon class template
template< typename POINTARRAY >
class PolygonT
{
  public:
    using POINTTYPE = typename POINTARRAY::value_type;
    PolygonT( POINTARRAY &points ) : _pointArray( points) {}
    virtual ~PolygonT() = default;


   /**
     * InPolygonTest - Determines the position of a point relative to a polygon.
     *
     * This function uses the ray-casting algorithm to determine if a point lies inside, outside,
     * The basic idea is to cast a ray from the point in a certain direction and count how many times the ray intersects
     * the edges of the polygon.
     *
     * Algorithm Details:
     * - Odd number of intersections: the point is inside the polygon.
     * - Even number of intersections: the point is outside the polygon.
     * - On the polygon edge: special condition handled separately.
     *
     * @param point The point to test.
     * @param printMessages Flag to control printing of messages.
     * @return A string representation of the point's status relative to the polygon.
     */
  
    
   auto InPolygonTest(const POINTTYPE &point, bool printMessages = true) const -> std::string
{
    PolygonTestResult ret = PolygonTestResult::UNKNOWN;
    int intersections = 0;
    auto px = getX(point), py = getY(point);

    for (size_t i = 0; i < _pointArray.size(); ++i) {
        const POINTTYPE &start = _pointArray[i];
        const POINTTYPE &end = _pointArray[(i + 1) % _pointArray.size()];

        auto sx = getX(start), sy = getY(start);
        auto ex = getX(end), ey = getY(end);
        POINTTYPE d = end - start;

        // Calculate py - sy to avoid redundant calculations
        auto py_sy = py - sy;

        // Check if the point is exactly on the edge
        if (isAlmostEqual(py_sy * getX(d), getY(d) * (px - sx)) &&
            std::min(sx, ex) <= px && px <= std::max(sx, ex) &&
            std::min(sy, ey) <= py && py <= std::max(sy, ey)) {
            ret = PolygonTestResult::OnPolygonEdge;
            break;
        }

        // Swap start and end points if necessary to ensure consistent direction
        if (sy > ey) {
            std::swap(sx, ex);
            std::swap(sy, ey);
            py_sy = py - sy;  // Recalculate py - sy after swapping
        }

        // Adjust py slightly to avoid exactly touching the vertex
        if (isAlmostEqual(py, sy) || isAlmostEqual(py, ey)) py += 1e-10;

        // Check if the point is between the y-values of the current segment
        if (py > sy && py < ey) {
            double xinters = py_sy * getX(d) / getY(d) + sx;
            if (px < xinters) {
                ++intersections;
            }
        }
    }

    // Determine the final result based on the number of intersections
    if (ret != PolygonTestResult::OnPolygonEdge) {
        if (intersections % 2 != 0) {
            ret = PolygonTestResult::InPolygon;
        } else {
            ret = PolygonTestResult::OutsidePolygon;
        }
    }

    // Optionally print messages for debugging or information
    if (printMessages) {
        std::cout << point << " in polygon ";
        PrintPolygon(_pointArray);
        std::cout << " is " << _enumItemStrings[static_cast<int>(ret)] << std::endl;
    }
    return _enumItemStrings[static_cast<int>(ret)];
} 
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
   /**
   *Clip a path (line segments) against the polygon
   * This function takes a series of line segments (defined by consecutive points in tobeclippedpath) 
   * and clips them against the edges of the polygon defined by _pointArray.
   * The resulting clipped segments are stored in the 'clipped' array.
   * @param tobeclippedpath The path to be clipped.
   * @param clipped The clipped path.
   */
    auto ClipSegments(const POINTARRAY &tobeclippedpath, POINTARRAY &clipped) -> void
    {    

      std::cout << "use ";
      PrintPolygon(_pointArray);
      std::cout << " to clip ";
      PrintPolygon(tobeclippedpath);
      std::cout<<std::endl;
      clipped.clear();
      if (tobeclippedpath.size() < 2) return;

      POINTTYPE lastAddedPoint;
      bool lastPointValid = false;

      for (size_t i = 0; i < tobeclippedpath.size() - 1; ++i) {
        POINTTYPE start = tobeclippedpath[i];
        POINTTYPE end = tobeclippedpath[i + 1];

        auto startStatus = InPolygonTest(start, false);
        auto endStatus = InPolygonTest(end, false);

        bool startInsideOrOnEdge = (startStatus == "InPolygon" || startStatus == "OnPolygonEdge");
        bool endInsideOrOnEdge = (endStatus == "InPolygon" || endStatus == "OnPolygonEdge");

        std::vector<POINTTYPE> intersections;
        
        // Check for intersections with each edge of the polygon
        for (size_t j = 0; j < _pointArray.size(); ++j) {
          const POINTTYPE &polyStart = _pointArray[j];
          const POINTTYPE &polyEnd = _pointArray[(j + 1) % _pointArray.size()];

          SegmentIntersection(start, end, polyStart, polyEnd, intersections);
        }
        auto square = [](const POINTTYPE& a){
            return getX(a) * getX(a) + getY(a) * getY(a);
        };
        // Lambda function to add points to the clipped path only if they are unique
        auto addUniquePoint = [&](const POINTTYPE &point) {
          if (!lastPointValid || getX(point) != getX(lastAddedPoint) || getY(point) != getY(lastAddedPoint)) {
            clipped.push_back(point);
            lastAddedPoint = point;
            lastPointValid = true;
          }
        };

        if (startInsideOrOnEdge) {
          addUniquePoint(start);
        }
        // Add intersections sorted by their distance from the start point
        // Not sure this sorting is necessary, but it ensures that the clipped path is continuous
         std::sort(intersections.begin(), intersections.end(), [&](const POINTTYPE &a, const POINTTYPE &b) {
            return square(a-start) < square(b-start);
        });

        for (const auto &intersection : intersections) {
          addUniquePoint(intersection);
        }
        // Add the end point if it is inside or on the edge of the polygon
        if (endInsideOrOnEdge) {
          addUniquePoint(end);
        }
      }

      std::cout << "Clipped path: ";
      PrintPolygon(clipped);
      std::cout << std::endl;

    }


  private:
    POINTARRAY &_pointArray;



};

