#ifndef INCLUDE_RRT_PLANNING_NH_CORNERINDEX_H_
#define INCLUDE_RRT_PLANNING_NH_CORNERINDEX_H_

#include "rrt_planning/rrt/Cover_Tree.h"

namespace rrt_planning
{

class CornerCoverWrapper
{
public:
    CornerCoverWrapper(Distance* distance, Eigen::VectorXd& corner) : dist(distance), corner(corner)
    {

    }

    inline bool operator ==(const CornerCoverWrapper& obj) const
    {
        return obj.corner == this->corner;
    }

    inline double distance(const CornerCoverWrapper& obj) const
    {
        auto& dist = *this->dist;
        return dist(obj.corner, this->corner);
    }

    inline Eigen::VectorXd getCorner() const
    {
        return corner;
    }

private:
    Eigen::VectorXd corner;
    Distance* dist;
};

class CornerIndex : CoverTree<CornerCoverWrapper>
{
public:

    CornerIndex(Distance& dist) : dist(dist), CoverTree<CornerCoverWrapper>(1e3)
    {

    }

    inline void insert(Eigen::VectorXd& p)
    {
        CoverTree<CornerCoverWrapper>::insert(CornerCoverWrapper(&dist, p));
    }

    inline void remove(Eigen::VectorXd& p)
    {
        CoverTree<CornerCoverWrapper>::remove(CornerCoverWrapper(&dist, p));
    }

    inline Eigen::VectorXd getNearestNeighbour(const Eigen::VectorXd& x)
    {
        Eigen::VectorXd tmp = x;
        CornerCoverWrapper tmpWrapped(&dist, tmp);
        auto result = CoverTree<CornerCoverWrapper>::kNearestNeighbors(tmpWrapped, 1);

        return result.back().getCorner();
    }

    inline std::vector<Eigen::VectorXd> getNearestNeighbours(const Eigen::VectorXd& x, int k)
    {
        Eigen::VectorXd tmp = x;
        CornerCoverWrapper tmpWrapped(&dist, tmp);
        auto result = CoverTree<CornerCoverWrapper>::kNearestNeighbors(tmpWrapped, k);

        std::vector<Eigen::VectorXd> neighbors;
        for (auto n: result)
        {
          neighbors.push_back(n.getCorner());
        }

        return neighbors;
    }


private:
    Distance& dist;
};

}


#endif /* INCLUDE_RRT_PLANNING_NH_CORNERINDEX_H_ */
