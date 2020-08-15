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
    Position(int _x, int _y) : Point(_x, _y) {}

    // constructor from base ecn::Point
    Position(ecn::Point p) : Point(p.x, p.y) {}

    int distToParent()
    {
        // in cell-based motion, the distance to the parent is always 1
        return 1;
    }

    std::vector<PositionPtr> children()
    {
        // this method should return  all positions reachable from this one
        std::vector<PositionPtr> generated;

        for(auto i = -1; i<2; i = i +2)
        {
            if(x+i >= 0 && x+i < maze.width() && y>=0 && y < maze.height() &&
                    maze.isFree(x+i, y))
                generated.push_back(PositionPtr(new Position(x+i, y)));

        }
        for(auto j = -1; j<2; j = j +2)
        {
            if(x >= 0 && x < maze.width() && y+j>=0 && y+j < maze.height() &&
                    maze.isFree(x, y+j))
                generated.push_back(PositionPtr(new Position(x, y+j)));

        }
        return generated;


    }
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
