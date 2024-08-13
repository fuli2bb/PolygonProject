#include <array>
#include <vector>
#include <deque>
#include <iostream>
#include "PolygonT.h"
// To compile with g++:
//     g++ -o Polygon.exe Polygon.cpp
// The functions to implement are:
// PolygonT.h
//  SegmentIntersection,
//  PolygonT::InPolygonTest, and
//  PolygonT::ClipSegments
// 
// SegmentIntersection: 
//  calculate the intersection(s) of two line segments
// PolygonT::InPolygonTest:
//  return the status of a point with respect to the polygon. It can be inside, outside or on the polygon edge.
// PolygonT::ClipSegments
//  use the polygon to clip a path, or a consecutive set of points which define a set of line segments by each pair of 
//  adjacent points of the path
//
// Polygon.cpp
//  Enhance the code in the testing classes that specialize PolygonT as needed

template< typename COORDTYPE >
class Point2DT
{
public:
     Point2DT() = default;
     Point2DT(COORDTYPE x, COORDTYPE y ) { _point={x,y};}
     //Point2DT &operator=( const Point2DT& pt ) {
     //     if ( this != &pt ) {
     //          std::memcpy( _point.get(), pt._point.get(), 2*sizeof(COORDTYPE));
     //     }
     //}
     
     //Copy assignment operator for assigning one Point2DT object to another
     Point2DT& operator=(const Point2DT& pt) {
        if (this != &pt) { // Check for self-assignment
            _point = pt._point; 
        }
        return *this; // Return the current object to allow chained assignments
     }
     
    Point2DT operator-(const Point2DT& pt) const {
        return Point2DT(_point[0] - pt._point[0], _point[1] - pt._point[1]);
    }

     virtual ~Point2DT() = default;
     friend std::ostream& operator<<(std::ostream& stream, const Point2DT& obj)
    {
        stream << obj.X() << "," << obj.Y();
        return stream;
    }
     auto X() const -> COORDTYPE { return _point[0];}
     auto Y() const -> COORDTYPE { return _point[1];}
private:
     std::array<COORDTYPE,2> _point;
};


template< typename COORDTYPE >
class PointXYT
{
public:
     PointXYT() = default;
     PointXYT(COORDTYPE ptx, COORDTYPE pty ) : x(ptx), y(pty) {}
     PointXYT &operator=( const PointXYT& pt ) {
          if ( this != &pt ) {
               x = pt.x;
               y = pt.y;
          }
     }

     PointXYT operator-(const PointXYT& pt) const {
        return PointXYT(x - pt.x, y - pt.y);
    }

     virtual ~PointXYT() = default;
     friend std::ostream& operator<<(std::ostream& stream, const PointXYT& obj)
    {
        stream << obj.x << "," << obj.y;
        return stream;
    }

     COORDTYPE x,y;
};

auto PolygonTest() -> void
{   
    
    std::cout<<"---------------------------"<<std::endl;
    std::cout<<"Testing InPolygonTest"<<std::endl;
    std::cout<<"---------------------------"<<std::endl;
    {
        using UIntPoint = Point2DT<uint64_t>;
        std::vector<UIntPoint> pointArray = { {0, 0}, {3, 0}, {1, 4}, {1, 5}, {0, 2} };
        PolygonT<std::vector<UIntPoint>> polygon(pointArray);
        polygon.InPolygonTest(UIntPoint(2, 0));  // Edge
        polygon.InPolygonTest(UIntPoint(1, 4));  // Edge
        polygon.InPolygonTest(UIntPoint(1, 1));  // Inside
    }
    {
        using doublePoint = Point2DT<double>;
        std::deque<doublePoint> pointArray = { {-1, -1}, {3, -2}, {2, 1}, {4, 5}, {2, 3} };
        PolygonT<std::deque<doublePoint>> polygon(pointArray);
        polygon.InPolygonTest(doublePoint(2, 3)); 
        polygon.InPolygonTest(doublePoint(5, 5)); 
        polygon.InPolygonTest(doublePoint(3, -2));
    }
    {
        using IntPoint = Point2DT<int>;
        std::array<IntPoint, 5> pointArray = { { {-1, -1}, {3, -2}, {2, 1}, {4, 5}, {-2, 2} } };
        PolygonT<std::array<IntPoint, 5>> polygon(pointArray);
        polygon.InPolygonTest(IntPoint(2, 3));     
        polygon.InPolygonTest(IntPoint(-3, 3));    
        polygon.InPolygonTest(IntPoint(-1, -1));   
    }
    {
        using doublePoint = PointXYT<double>;
        std::deque<doublePoint> pointArray = { {-2, -2}, {3, -2}, {2, 1}, {4, 5}, {3, 3} };
        PolygonT<std::deque<doublePoint>> polygon(pointArray);
        polygon.InPolygonTest(doublePoint(2, 3));  
        polygon.InPolygonTest(doublePoint(5, -3)); 
        polygon.InPolygonTest(doublePoint(-2, -2));
    }
    {
        using IntPoint = Point2DT<int>;
        std::vector<IntPoint> pointArray = { {-1, -1}, {3, -2}, {2, 1}, {4, 5}, {-2, 2} };
        PolygonT<std::vector<IntPoint>> polygon(pointArray);
     
        polygon.InPolygonTest(IntPoint(2, 3));
    }

    std::cout<<"---------------------------"<<std::endl;
    std::cout<<"Testing Clip"<<std::endl;
    std::cout<<"---------------------------"<<std::endl;
    {   
        // simple case
        using floatPoint = Point2DT<float>;
        std::vector<floatPoint> pointArray = { {0, 0}, {5, 0}, {5, 5}, {0, 5} };
        PolygonT<std::vector<floatPoint>> polygon(pointArray);
        std::vector<floatPoint> toclip = { {-1.0f, -1.0f},{0.0f, 1.0f}, {3.0f,3.0f} }, clipped;
        polygon.ClipSegments(toclip, clipped);
    }

    {
          using floatPoint = Point2DT<float>;
          // no-convex polygon
          std::vector<floatPoint> pointArray = { {0, 0}, {5, 0}, {5, 10}, {0, 10}, {0,7.5}, {2.5,7.5},{2.5,2.5}, {0, 2.5}};
          PolygonT<std::vector<floatPoint>> polygon(pointArray);
          std::vector<floatPoint> toclip = { {1.0f, -1.0f},{1.5f, 1.5f}, {0.5f,13.0f} }, clipped;
          polygon.ClipSegments(toclip, clipped);
      }

    std::cout<<"---------------------------"<<std::endl;
    std::cout<<"Testing SegmentIntersection"<<std::endl;
    std::cout<<"---------------------------"<<std::endl;
    {
        using IntPoint = Point2DT<int>;

        // Test SegmentIntersection
        std::vector<IntPoint> intersections;

        SegmentIntersection(IntPoint(0, 0), IntPoint(3, 3), IntPoint(0, 3), IntPoint(3, 0), intersections);
        if (!intersections.empty()) { std::cout<<"Overlap at: "; PrintPolygon(intersections); }
        else std::cout<<"parrallel or overlapping";
        std::cout<<"\n";

        intersections.clear();
        SegmentIntersection(IntPoint(0, 0), IntPoint(3, 3), IntPoint(1, 1), IntPoint(4, 4), intersections);
        //if (!intersections.empty()) PrintPolygon(intersections);
        if (!intersections.empty()) { std::cout<<"Overlap at: "; PrintPolygon(intersections); }
        else std::cout<<"parrallel or overlapping";
        std::cout<<"\n";


    }
}



int main()
{
    PolygonTest();
}

