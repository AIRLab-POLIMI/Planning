#include "rrt_planning/grid/Gridmap.h"
#include <cmath>
#include <Eigen/Dense>

using namespace std;

namespace rrt_planning
{

Gridmap::Gridmap((Map& map, double gridResolution): map(map),
      gridResolution(gridResolution)
{
      Bounds bounds = map.getBounds();

      maxX = floor((bounds.maxX - bounds.minX) / gridResolution);
      maxY = floor((bounds.maxY - bounds.minY) / gridResolution);
      minX = minY = 0;

      computeBounds();
}


void Gridmap::computeBounds()
{
      vector<Vector3d> borders;
      borders.push_back(Vector3d(0, 0, 1));
      borders.push_back(Vector3d(maxX-1, 0, 1));
      borders.push_back(Vector3d(maxX-1, maxY-1, 1));
      borders.push_back(Vector3d(0, maxY-1, 1));

      for(int i = 0; i < 3; i++)
          lines.push_back(borders[i].cross(borders[i+1]));

      lines.push_back(borders[3].cross(borders[0]));
}

Cell Gridmap::computeMiddle(const Cell& a, const Cell& b, const Cell& action)
{
    Cell middle;
    int dx, dy;

    dx = abs(b.first - a.first);
    dy = abs(b.second - a.second);

    if(dx == 1 && dy == 1)
    {
        middle.first = a.first;
        middle.second = b.second;

        if(!checkFree(middle))
            return middle;
        else
            return Cell(b.first, a.second);

    }
    else if((dx == 1 && dy == 0) || (dy == 1 && dx == 0))
    {
        if(a == action)
            return b;
        else if(b == action)
            return a;

        return (isCorner(a) ? a : b);
    }

    //compute mid point
    middle.first = (b.first > a.first) ? (a.first + abs(dx/2)) : (b.first + abs(dx/2));
    middle.second = (b.second > a.second) ? (a.second + abs(dy/2)) : (b.second + abs(dy/2));

    return middle;
}


bool Gridmap::checkFree(const Cell& cell)
{
    for(int i = 0; i < 2 && cell.first-i > 0; i++){
        for(int j = 0; j < 2 && cell.second-j > 0; j++){
            if(isFree(Cell(cell.first-i, cell.second-j)))
                return true;
        }
    }

    return false;
}

bool Gridmap::isCorner(const Cell& cell)
{
    int black = 0;

    for(int i = -1; i <= 0; i++)
    {
        for(int j = -1; j<=0; j++)
        {
            if(!isFree(Cell(cell.first+i, cell.second+j)))
            {
                black++;
            }
        }
    }

    return ((isFree(Cell(cell.first, cell.second-1)) && isFree(Cell(cell.first-1, cell.second))) ||
            (isFree(cell) && isFree(Cell(cell.first-1,cell.second-1)))) && (black > 0);
}

bool Gridmap::clockwise(const Cell& start, const Cell& middle, int x, int y)
{
    double angle = std::atan2(middle.second - start.second, middle.first - start.first) - std::atan2(y - start.second, x - start.first);
    if(fabs(angle) > M_PI){
        angle = ( angle > 0 ) ? (angle - 2*M_PI) : (angle + 2*M_PI);
    }
    return (angle < 0);
}

vector<Cell> Gridmap::findSubgoals(const Cell& entry_point, const Cell& exit_point)
{
    Cell dummy(-1,-1);
    Cell middle = computeMiddle(entry_point, exit_point, dummy);
    Vector3d spoint(entry_point.first, entry_point.second, 1);
    Vector3d endpoint(exit_point.first, exit_point.second, 1);
    Vector3d line = spoint.cross(endpoint);

    int c = (-middle.first * -line(1)) + (-middle.second * line(0));
    Vector3d normal(-line(1), line(0), c);

    vector<Cell> intersection = findIntersection(normal);

    if(clockwise(entry_point, middle, intersection[0].first, intersection[0].second))
    {
        swap(intersection[0],intersection[1]);
    }

    return intersection;
}

Cell Gridmap::findSubgoal(const Cell& a, const Cell& b)
{
    Vector3d spoint(a.first, a.second, 1);
    Vector3d endpoint(b.first, b.second, 1);
    Vector3d line = spoint.cross(endpoint);

    vector<Cell> intersection = findIntersection(line);

    if(isSameDirection(a, b, intersection[0]))
    {
        return intersection[0];
    }

    return intersection[1];
}

vector<Cell> Gridmap::getPoints(const Cell& a, const Cell& b, bool clockwise)
{
    int sx = (b.first - a.first) > 0 ? 1 : -1;
    int sy = (b.second - a.second) > 0 ? 1 : -1;
    vector<Cell> points;
    int first, second;

    if(!clockwise)
    {
        if(sx == sy)
        {
            first = (sx > 0) ? maxX-1 : 0;
            second = (sy > 0) ? 0 : maxY-1;
            points.push_back(Cell(first, b.second));
            points.push_back(Cell(b.first, second));
        }
        else
        {
            first = (sx > 0) ? 0 : maxX-1;
            second = (sy > 0) ? maxY-1 : 0;
            points.push_back(Cell(b.first, second));
            points.push_back(Cell(first, b.second));
        }
    }
    else
    {
        if(sx == sy)
        {
            first = (sx > 0) ? 0 : maxX-1;
            second = (sy > 0) ? maxY-1 : 0;
            points.push_back(Cell(b.first, second));
            points.push_back(Cell(first, b.second));
        }
        else
        {
            first = (sx > 0) ? maxX-1 : 0;
            second = (sy > 0) ? 0 : maxY-1;
            points.push_back(Cell(first, b.second));
            points.push_back(Cell(b.first, second));
        }
    }

    return points;
}

vector<Cell> Gridmap::findIntersection(const Vector3d& line)
{
    vector<Cell> intersections;
    set<Cell> candidates;
    Vector3d point;

    for(auto bound : lines){
        point = line.cross(bound);
        point = point/point(2);
        if(isInside(point)){
             candidates.insert(Cell(point(0), point(1)));
        }
    }

    auto it = candidates.begin();
    intersections.push_back(Cell(it->first, it->second));
    it++;
    intersections.push_back(Cell(it->first, it->second));

    //Safe check
    if(candidates.size() == 3){
        it++;
        intersections[1].first = it->first;
        intersections[1].second = it->second;
    }

    return intersections;
}

bool Gridmap::los(const Cell& a, const Cell& b, vector<Cell>& collision, Modes mode)
{

    int x0 = a.first;
    int y0 = a.second;
    Cell old(x0, y0);
    int x1 = b.first;
    int y1 = b.second;
    int dx = x1 - x0;
    int dy = y1 - y0;
    int f = 0;
    int sx, sy;
    sx = sy = 1;

    if(dy < 0){
        dy = -dy;
        sy = -1;
    }
    if(dx < 0){
        dx = -dx;
        sx = -1;
    }


    auto& c1 = (dx > dy) ? x0 : y0;
    auto& c2 = (dx > dy) ? y0 : x0;
    auto& c3 = (dx > dy) ? x1 : y1;
    auto& c4 = (dx > dy) ? y1 : x1;
    auto& c5 = (dx > dy) ? dx : dy;
    auto& c6 = (dx > dy) ? dy : dx;
    auto& c7 = (dx > dy) ? sx : sy;
    auto& c8 = (dx > dy) ? sy : sx;


    bool curr = isFree(Cell(x0 + (sx -1)/2, y0 + (sy -1)/2));
    if(!(mode == exit_point)){
        curr = checkFree(a);
    }

    bool prev = curr;
    collision.clear();

    //Check middle point not inside obstacle
    if(curr && mode == exit_point){
        collision.push_back(old);
        return true;
    }


    while(c1 != c3)
    {
        old.first = x0;
        old.second = y0;

        f = f + c6;
        if(c6 == 0)
        {
            if(dy == 0)
            {
                curr = (isFree(Cell(x0 + (sx -1)/2, y0)) || isFree(Cell(x0 + (sx -1)/2, y0 -1)));
            }
            else
            {
                curr = (isFree(Cell(x0, y0 + (sy -1)/2)) || isFree(Cell(x0 -1, y0 + (sy -1)/2)));
            }
        }
        else if(f >= c5)
        {
            curr = isFree(Cell(x0 + (sx -1)/2, y0 + (sy -1)/2));
            c2 = c2 + c8;
            f = f - c5;
            if(curr==prev && f != 0)
            {
                curr = isFree(Cell(x0 + (sx -1)/2, y0 + (sy -1)/2));
                old.first = x0;
                old.second = y0;
            }
        }
        else
        {
            curr = isFree(Cell(x0 + (sx -1)/2, y0 + (sy -1)/2));
        }

        c1 = c1 + c7;

       //Change of pixel color
       if(curr != prev){
           switch(mode){
           case collision_points:
               if(collision.size() == 1 && isAdjacent(old, b))
               {
                   collision.push_back(b);
               }
               else
               {
                   collision.push_back(old);
               }
               if(collision.size() == 2)
               {
                  return false;
               }
               break;
           case exit_point:
               collision.push_back(old);
               return true;
            case is_los:
               return false;
           }
        }

       prev = curr;
    }


    if(mode == collision_points)
    {
        collision.push_back(b);
        if(collision.size() >= 2)
            return false;

     }

    return (mode != exit_point);
}

Cell Gridmap::followObstacle(const Cell& action, const Cell& subgoal)
{
    int x0 = action.first;
    int y0 = action.second;
    int x1 = subgoal.first;
    int y1 = subgoal.second;
    int dx = x1 - x0;
    int dy = y1 - y0;
    int sx, sy;

    Cell old(x0, y0);
    bool corner = false;
    bool curr = true;

    sx = sy = 1;

    if(dy < 0)
    {
        dy = -dy;
        sy = -1;
    }
    if(dx < 0)
    {
        dx = -dx;
        sx = -1;
    }

    auto& c1 = (dx > dy) ? x0 : y0;
    auto& c3 = (dx > dy) ? x1 : y1;
    auto& c7 = (dx > dy) ? sx : sy;

    if(dy == 0)
    {
        curr = !(isFree(Cell(x0 + (sx -1)/2, y0)) && isFree(Cell(x0 + (sx -1)/2, y0 -1)));
    }
    else
    {
        curr = !(isFree(Cell(x0, y0 + (sy -1)/2)) && isFree(Cell(x0 -1, y0 + (sy -1)/2)));
    }

    while(!corner && curr && c1!=c3)
    {
        old = Cell(x0, y0);
        if(dy == 0)
        {
            curr = (isFree(Cell(x0 + (sx -1)/2, y0)) || isFree(Cell(x0 + (sx -1)/2, y0 -1)));
        }
        else
        {
            curr = (isFree(Cell(x0, y0 + (sy -1)/2)) || isFree(Cell(x0 -1, y0 + (sy -1)/2)));
        }
        if(old != action)
        {
            corner = isCorner(old);
        }
        c1 = c1 + c7;
    }

    if(c1 == c3)
    {
        old = subgoal;
    }

    return old;
}

Vector3d Gridmap::findLine(const Cell& a, const Cell& b)
{
    Vector3d spoint(a.first, a.second, 1);
    Vector3d endpoint(b.first, b.second, 1);
    Vector3d line = spoint.cross(endpoint);

    return line;
}


bool Gridmap::isInside(const Vector3d& point)
{
    return ((point(0) >= 0) && (point(0) < maxX) && (point(1) >= 0) && (point(1) < maxY));
}


bool Gridmap::isSameDirection(const Cell& a, const Cell& b, const Cell& s)
{
    if((b.first - a.first > 0) != (s.first - a.first > 0))
    {
        return false;
    }
    else if((b.first - a.first == 0) && ((b.second - a.second > 0) != (s.second - a.second > 0)))
    {
        return false;
    }

    return true;
}

Eigen::VectorXd Grid::toMapPose(int X, int Y)
{
    Bounds bounds = map.getBounds();

    Eigen::VectorXd pos(2);

    pos(0) = X * gridResolution + bounds.minX;
    pos(1) = Y * gridResolution + bounds.minY;

    return pos;
}

double Gridmap::distance(const Cell& a, const Cell& b)
{
    return sqrt(pow(a.first - b.first, 2) + pow(a.second - b.second, 2));
}


}
