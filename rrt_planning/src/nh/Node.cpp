#include "rrt_planning/nh/Node.h"

using namespace std;
using namespace Eigen;

namespace rrt_planning
{

Node::Node()
{
	parent = nullptr;
}

Node::Node(const VectorXd& state, Node* parent, double cost):
				state(state), parent(parent), cost(cost) {}

Node::Node(const VectorXd& state, Node* parent, double cost, vector<VectorXd> mp) :
				state(state), parent(parent), cost(cost), mp(mp) {}

void Node::addSubgoal(const VectorXd& subgoal)
{
	rrt_planning::Sub pair(subgoal(0), subgoal(1));
	subgoals.insert(pair);
}

void Node::addTriangle(const Triangle& t)
{
	closed_area.push_back(t);
}

bool Node::insideArea(const VectorXd& p)
{
	for(auto t : closed_area)
	{
		if(t.contains(p))
			return true;
	}

	return false;
}

bool Node::contains(const VectorXd& subgoal)
{
	Sub pair(subgoal(0), subgoal(1));
	return (subgoals.count(pair) == 1);
}

Node* Node::setParent(Node* p)
{
	parent = p;
}

Node* Node::getParent()
{
	return parent;
}

VectorXd Node::getState()
{
	return state;
}

vector<VectorXd> Node::getMotionPrimitives()
{
	return mp;
}

double Node::getCost()
{
	return cost;
}

}
