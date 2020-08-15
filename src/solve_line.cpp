#include <a_star.h>
#include <maze.h>

using namespace std;
using namespace ecn;

// a node is a x-y position, we move from 1 each time
class Position : public Point
{
    typedef std::unique_ptr<Position> PositionPtr;

public:
    // constructor from coordinates
    Position(int _x, int _y, int d) : Point(_x, _y), dist(d) {}

    // constructor from base ecn::Point
    Position(ecn::Point p) : Point(p.x, p.y) {}

    int distToParent()
    {
        // in cell-based motion, the distance to the parent is always 1
        return dist;
    }
    bool is_corridor(int i,int j)
    {
        if(maze.isFree(i,j) == 0)
            return false;
        if(maze.isFree(i+1, j) + maze.isFree(i-1, j)
                + maze.isFree(i, j+1) + maze.isFree(i, j-1) == 2)
                return true;
    }
    std::vector<PositionPtr> children()
    {
        // this method should return  all positions reachable from this one
        std::vector<PositionPtr> neighbors;

        for(auto i = -1; i<2; i = i +2)
        {
            int dx = 1;
            while(is_corridor(x+i*dx, y))
                dx++;
            if(!maze.isFree(x+i*dx, y)) // go back 1 cell if reached a wall
                dx--;
            if(dx)
                neighbors.push_back(PositionPtr(new Position(x+i*dx, y, dx)));

        }
        for(auto j = -1; j<2; j = j +2)
        {
            int dy = 1;
            while(is_corridor(x, y+j*dy))
                dy++;
            if(!maze.isFree(x, y+j*dy)) // go back 1 cell if reached a wall
                dy--;
            if(dy)
                neighbors.push_back(PositionPtr(new Position(x, y+j*dy, dy)));

        }
        return neighbors;

    }
protected: int dist;
};



int main( int argc, char **argv )
{
    // load file
    std::string filename = "maze.png";
    if(argc == 2)
        filename = std::string(argv[1]);

    // let Point know about this maze
    Position::maze.load(filename);

    // initial and goal positions as Position's
    Position start = Position::maze.start(),
             goal = Position::maze.end();

    // call A* algorithm
    ecn::Astar(start, goal);

    // save final image
    Position::maze.saveSolution("cell");
    cv::waitKey(0);

}
