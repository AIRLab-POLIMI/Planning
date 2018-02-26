#include "rrt_planning/Voronoi.h"
#include <chrono>

#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/Marker.h>

//#define VIS_CONF
//#define PRINT_CONF


//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_planning::Voronoi, nav_core::BaseGlobalPlanner)

using namespace std;
using namespace Eigen;

//Default Constructor
namespace rrt_planning
{

const Cell Voronoi::S_NULL = make_pair(-1, -1);

Voronoi::Voronoi()
{
    grid = nullptr;
    map = nullptr;
}

Voronoi::Voronoi(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    initialize(name, costmap_ros);
}


void Voronoi::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{

    //Get parameters from ros parameter server
    ros::NodeHandle private_nh("~/" + name);
    private_nh.param("discretization", discretization, 0.2);
    pub = private_nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1);

    map = new ROSMap(costmap_ros);
    grid = new Grid(*map, discretization);

	Bounds bounds = map->getBounds();
	sizeX = floor((bounds.maxX - bounds.minX) / discretization);
	sizeY = floor((bounds.maxY - bounds.minY) / discretization);

	visualizer.initialize(private_nh);
}

bool Voronoi::makePlan(const geometry_msgs::PoseStamped& start,
                                const geometry_msgs::PoseStamped& goal,
                                std::vector<geometry_msgs::PoseStamped>& plan)
{

#ifdef VIS_CONF
    visualizer.clean();
#endif
    //Init the position of the special states
    s_start = grid->convertPose(start);
    s_goal = grid->convertPose(goal);

    //Test starting position
    if(!grid->isFree(s_start))
    {
#ifdef PRINT_CONF
        ROS_INFO("Invalid starting position");
#endif
        return false;
    }

    //Test target position
    if(!grid->isFree(s_goal))
    {
#ifdef PRINT_CONF
        ROS_INFO("Invalid target position");
#endif
        return false;
    }

    std::chrono::steady_clock::time_point t = std::chrono::steady_clock::now();
	bool skeleton = initVoronoi();

    std::chrono::duration<double> Tcurrent = std::chrono::steady_clock::now() - t;
#ifdef PRINT_CONF
    ROS_FATAL_STREAM("Time to compute skeleton: " << Tcurrent.count());
#endif


	if(!skeleton)
	{
		ROS_FATAL_STREAM("Failed to compute voronoi skeleton");
		return false;
	}
#ifdef VIS_CONF
	displayVoronoiGrid();

#endif

	//Initialization
	int goal_x = s_goal.first;
	int goal_y = s_goal.second;
	int start_x = s_start.first;
	int start_y = s_start.second;
    vector<Cell> path1;
	vector<Cell> path2;
	vector<Cell> path3;
	bool res1 = false;
	bool res2 = false;
	bool res3 = false;


	//Start query

	if(!voronoi_.isVoronoi(goal_x, goal_y))
	{
		res3 = findPath(path3, goal_x, goal_y, start_x, start_y, 0, 1);
		goal_x = path3.back().first;
		goal_y = path3.back().second;
		reverse(path3.begin(), path3.end());
	}else
	{
		res3 = true;
	}

	if(!voronoi_.isVoronoi(start_x, start_y))
	{
		res1 = findPath(path1, start_x, start_y, goal_x, goal_y, 0, 1);
		start_x = path1.back().first;
		start_y = path1.back().second;
		reverse(path1.begin(), path1.end());
	}else
	{
		res1 = true;
	}

	res2 = findPath(path2, start_x, start_y, goal_x, goal_y, 1, 0);

	if(!res1 && !res2 && !res3)
	{
		ROS_FATAL("Failed to find full path");
		return false;
	}

	path1.insert(path1.end(), path2.begin(), path2.end());
	path1.insert(path1.end(), path3.begin(), path3.end());

	vector<Cell> final_path = smoothPath(&path1);
	//vector<Cell> final_path = path1;

	//Publish plan
	vector<VectorXd> path;
    path.push_back(grid->toMapPose(s_start.first, s_start.second));
	for(auto p : final_path)
	{
		path.push_back(grid->toMapPose(p.first, p.second));
	}
    path.push_back(grid->toMapPose(s_goal.first, s_goal.second));
	publishPlan(path, plan, start.header.stamp, start, goal);
#ifdef VIS_CONF
   // visualizer.displayPlan(plan);
#endif
    return true;
}

bool Voronoi::findPath(std::vector<Cell>& path, int init_x, int init_y, int goal_x,
							int goal_y,  bool check_is_voronoi_cell, bool stop_at_voronoi)
{
	std::vector<std::pair<int, int> > delta;
    delta.push_back( {-1, 0} );     // go up
    delta.push_back( {0, -1} );     // go left
    delta.push_back( {1, 0} );      // go down
    delta.push_back( {0, 1} );      // go right
    delta.push_back( {-1, -1} );    // up and left
    delta.push_back( {-1, 1} );     // up and right
    delta.push_back( {1, -1} );     // down and left
    delta.push_back( {1,  1} );     // down and right

    // cost of movement
    float cost = 1;


    // closed cells grid (same size as map grid)
    bool **closed=NULL;
    closed = new bool*[sizeX];
    for (int x=0; x<sizeX; x++) {
        (closed)[x] = new bool[sizeY];
    }

    for (int y=sizeY-1; y>=0; y--) {
        for (int x=0; x<sizeX; x++) {
            (closed)[x][y] = false;
        }
    }


    // actions (number of delta's row) cells grid (same size as map grid)
    int **action=NULL;
    action = new int*[sizeX];
    for (int x=0; x<sizeX; x++) {
        (action)[x] = new int[sizeY];
    }
    for (int y=sizeY-1; y>=0; y--) {
        for (int x=0; x<sizeX; x++) {
            (action)[x][y] = -1;
        }
    }

    // set current cell
    int x = init_x;
    int y = init_y;

    // set cost
    float g = 0;

    //f = heuristic(x,y) + g;

    // vector of open (for possible expansion) nodes
    std::vector<std::tuple<float, int, int> > open;
    open.push_back( std::make_tuple( g, x, y ) );

    // path found flag
    bool found = false;
    // no solution could be found flag
    bool resign = false;

    while( !found && !resign )
    {
        if (open.size() == 0)
        {
            resign = true;
            path.clear();
            return false;
        }
        else
        {
            // sort open by cost
            sort(open.begin(), open.end());
            reverse(open.begin(), open.end());
            // get node with lowest cost
            std::tuple<float, int, int> next = open[open.size()-1];
            open.pop_back();
            g = std::get<0>(next);
            x = std::get<1>(next);
            y = std::get<2>(next);

            // check, whether the solution is found (we are at the goal)
            if(stop_at_voronoi)
            {
                // if stop_at_voronoi is set, we stop, when get path to any voronoi cell
                if(voronoi_.isVoronoi(x,y))
                {
                    found = true;
                    goal_x = x;
                    goal_y = y;
                    continue;
                }
            }
            else
            {
                if ( x == goal_x && y == goal_y )
                {
                    found = true;
                    continue;
                }
            }
            for( int i=0; i < delta.size(); i++ )
            {
                // expansion
                int x2 = x + std::get<0>(delta[i]);
                int y2 = y + std::get<1>(delta[i]);

                // check new node to be in grid bounds
                if ( x2 >= 0 && x2 < sizeX && y2 >= 0 && y2 < sizeY )
                {
                    // check new node not to be in obstacle
                    if(voronoi_.isOccupied(x2,y2))
                    {
                        continue;
                    }
                    // check new node was not early visited
                    if ( closed[x2][y2] ){
                        continue;
                    }

                    // check new node is on Voronoi diagram
                    if (!voronoi_.isVoronoi(x2,y2) && check_is_voronoi_cell){
                        continue;
                    }

                    float g2 = g + cost;
                    //                        f2 = heuristic(x2,y2) + g2;
                    open.push_back( std::make_tuple( g2, x2, y2 ) );
                    closed[x2][y2] = true;
                    action[x2][y2] = i;
                }
            }
        }
    }

    // Make reverse steps from goal to init to write path
    x = goal_x;
    y = goal_y;

    int i = 0;
    path.clear();

    while( x != init_x || y != init_y )
    {
        path.push_back(make_pair(x, y));
        i++;

        int x2 = x - std::get<0>( delta[ action[x][y] ] );
        int y2 = y - std::get<1>( delta[ action[x][y] ] );


        x = x2;
        y = y2;
    }

    reverse(path.begin(), path.end());
    return true;

}


bool Voronoi::initVoronoi()
{
	bool **map=NULL;
	map = new bool*[sizeX];

	for(int x = 0; x < sizeX; x++)
	{
		(map)[x] = new bool[sizeY];
	}

	for(int y = sizeY-1; y >= 0; y--)
	{
		for(int x = 0; x < sizeX; x++)
		{
			Cell c = make_pair(x, y);
			(map)[x][y] = !grid->isVoronoiFree(c);
		}
	}

	voronoi_.initializeMap(sizeX, sizeY, map);
	voronoi_.update();
	voronoi_.prune();

	return true;
}

std::vector<Cell> Voronoi::smoothPath(std::vector<Cell>* original_path)
{
	vector<Cell> newpath = *original_path;
	int size = newpath.size();

    for(int i = 1; i < newpath.size() - 2; i++)
    {
        Cell a = newpath[i];
        Cell b = newpath[i+1];
        //diagonal movement
        if(a.first != b.first && a.second != b.second)
        {
            Cell c = newpath[i + 2];
            if((a.first == c.first) || (a.second == c.second))
            {
                newpath.erase(newpath.begin() + i + 1);
            }
        }
    }
#ifdef PRINT_CONF
    ROS_FATAL_STREAM("Nodes removed before: " << size);
    ROS_FATAL_STREAM("Nodes after: " << newpath.size());
#endif
	return newpath;
}


void Voronoi::publishPlan(std::vector<Eigen::VectorXd>& path, std::vector<geometry_msgs::PoseStamped>& plan,
                                   const ros::Time& stamp, const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal)
{
    plan.push_back(start);

    for(int i = 1; i < path.size() - 1; i++)
    {
        auto&& p1 = path[i];
        auto&& p2 = path[i+1];

        geometry_msgs::PoseStamped msg;

        msg.header.stamp = stamp;
        msg.header.frame_id = "map";

        msg.pose.position.x = p1(0);
        msg.pose.position.y = p1(1);
        msg.pose.position.z = 0;

        double angle = atan2(p2(1) - p1(1), p2(0) - p1(0));

        Matrix3d m;
        m = AngleAxisd(angle, Vector3d::UnitZ())
            * AngleAxisd(0, Vector3d::UnitY())
            * AngleAxisd(0, Vector3d::UnitX());

        Quaterniond q(m);

        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        msg.pose.orientation.w = q.w();

        plan.push_back(msg);
    }

    plan.push_back(goal);
}

void Voronoi::displayVoronoiGrid()
{
	visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "voronoi";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = discretization;
    marker.scale.y = discretization;
    marker.scale.z = 0;
    marker.color.a = 0.5;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

	for(int i = 0; i < sizeX; i++)
	{
		for(int j = 0; j < sizeY; j++)
		{
			if(voronoi_.isVoronoi(i, j))
			{
				geometry_msgs::Point p;
		        VectorXd pos = grid->toMapPose(i, j);
		        p.x = pos(0);
		        p.y = pos(1);
		        p.z = 0;
		        marker.points.push_back(p);
			}
		}
	}
    pub.publish(marker);
}

Voronoi::~Voronoi()
{
    if(grid)
        delete grid;

    if(map)
        delete map;
}


};
