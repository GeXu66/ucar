//
// Created by stdcat on 3/4/21.
//

#ifndef SRC_ASTAR_H
#define SRC_ASTAR_H

#include "common_include.h"
#include "GridMap.h"

#define isVisual 0

#define  usePriorityQueue 1  //rebuild after modify this define
#if not usePriorityQueue
#define useVectorSort 1
#else
#define useVectorSort 0
#endif

typedef enum NodeState_e{
    FREE_NODE = 0,
    IN_OPENLIST,
    IN_CLOSELIST,
}NodeState_e;

typedef struct Node{
    Vector2i coordinate;
    double g;
    double h;
    int index;
    NodeState_e state;
    Node* father;

    Node(Vector2i src)
	{
		this->coordinate = src;
		this->g = 0;
		this->h = 0;
        this->index = -1;
		this->father = NULL;
        this->state = FREE_NODE;
	}
	Node(int x,int y,Node* father) {
        this->coordinate.x() = x;
        this->coordinate.y() = y;
        this->g = 0;
        this->h = 0;
        this->index = -1;
        this->father = father;
        this->state = FREE_NODE;
    }
    Node(Vector2i src,Node* father) {
        this->coordinate = src;
        this->g = 0;
        this->index = -1;
        this->h = 0;
        this->father = father;
        this->state = FREE_NODE;
    }

    double f(){
        return (this->g + this->h);
    };
}Node;


class AStar{
private:
#if usePriorityQueue
    class cmp{
    public:
        bool operator()(Node* &n1, Node* &n2) const
        {
            return n1->f() > n2->f();
        }
    };
    priority_queue<Node*, vector<Node*>, cmp> openList;
#elif useVectorSort
    static bool cmp(Node *n1, Node *n2){
        return n1->f() < n2->f();
    }
    vector<Node*> openList;
#endif
    double checkSize_A;
    double longCheckSize_A;
    double minAng_A;

    ros::NodeHandle nh;
    ros::Publisher pub;
    int pointId;
    nav_msgs::Path pathMsg;
    void tracePoint(Node* cur);

    GridMap_t *map;

    static constexpr double WeightW = 10;
	static constexpr double WeightWH = 14;

	Node *end_n;


    vector<Node*> mapNode;

	void reset();
	bool checkBound(Node *node);
	bool checkBound(int x, int y);
	bool unWalk(int x, int y);

	bool search( Node *sNode, Node *eNode );
	void extend(Node *current);
	void goTo( int x, int y, Node* father, double g );
	void countH(Node *sNode, Node *eNode);

	void getPath( Node *current );

public:
    AStar(GridMap_t *init_map);
//    void resetMap(GridMap_t newMap);
    nav_msgs::Path getAStarPath( Vector2d start_p, Vector2d end_p );
    nav_msgs::Path curPath();
    vector<Vector3d> getWayPoints();
    vector<Vector3d> getWayPoints(int kernelSize, double miu, double sigma);
};

#endif //SRC_ASTAR_H
