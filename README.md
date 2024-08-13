# PolygonProject

A C++ project for performing geometric operations on polygons, including point-in-polygon tests, segment clipping, and line segment intersections. 

It uses c++ template and **SFINAE**, allowing the code to work with different point types by checking for specific members and methods.

## Features

- Segment Intersection: Compute intersection(s) of two line segments.
- Point-in-Polygon Test: Determine if a point is inside, outside, or on the edge of a polygon using the ray-casting algorithm.
- Segment Clipping: Clip a series of line segments against a polygon.

## Getting Started

### Compilation

To compile the project, use the following command:

```bash
g++ -std=c++17 -o exe Polygon.cpp
