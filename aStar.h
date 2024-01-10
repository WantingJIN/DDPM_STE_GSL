 
#include <list>
#include <algorithm>
#include <iostream>
#include <stdio.h>
#include "controller_STE_multi-robot.h"
#include "Position.h"
#include "nvwa/debug_new.h"

 
class map {
public:
    float xmin, xmax, ymin, ymax, xres, yres;
    int w,h;
    float ** obstacle_list = NULL;
    int nbr_obstacles;
    
    map() {
        xmin = XMIN;
        xmax = XMAX;
        ymin = YMIN;
        ymax = YMAX;
        w = 100; //x
        h = 40; //y
        xres = (xmax - xmin)/w; //0.2;
        yres = (ymax - ymin)/h; //0.2;
        nbr_obstacles = 0;
    }

    map(float a_obstacle_list[][4], int a_nbr_obstacles) {
        xmin = XMIN;
        xmax = XMAX;
        ymin = YMIN;
        ymax = YMAX;
        w = 50; //x
        h = 20; //y
        xres = (xmax - xmin)/w; //0.2;
        yres = (ymax - ymin)/h; //0.2;

        nbr_obstacles = a_nbr_obstacles;

        obstacle_list = new float*[nbr_obstacles];
        
        for (int i = 0; i < nbr_obstacles; i++){
            obstacle_list[i] = new float[4];
            for (int j = 0; j < 4; j++)
                obstacle_list[i][j] = a_obstacle_list[i][j];
        }
    }

    map(const map& another_map) {
        xmin = another_map.xmin;
        xmax = another_map.xmax;
        ymin = another_map.ymin;
        ymax = another_map.ymax;
        w = another_map.w; //x
        h = another_map.h; //y
        xres = another_map.xres;
        yres = another_map.yres;

        nbr_obstacles = another_map.nbr_obstacles;

        obstacle_list = new float*[nbr_obstacles];
        
        for (int i = 0; i < nbr_obstacles; i++){
            obstacle_list[i] = new float[4];
            for (int j = 0; j < 4; j++)
                obstacle_list[i][j] = another_map.obstacle_list[i][j];
        }
    }

    map& operator= (const map& another_map){
        if(&another_map == this)
            return *this;
        xmin = another_map.xmin;
        xmax = another_map.xmax;
        ymin = another_map.ymin;
        ymax = another_map.ymax;
        w = another_map.w; //x
        h = another_map.h; //y
        xres = another_map.xres;
        yres = another_map.yres;

        if(obstacle_list != NULL){
            for(int i = 0; i < nbr_obstacles; i++)
                delete[] obstacle_list[i];
            delete[] obstacle_list;
        }

        nbr_obstacles = another_map.nbr_obstacles;

        obstacle_list = new float*[nbr_obstacles];
        
        for (int i = 0; i < nbr_obstacles; i++){
            obstacle_list[i] = new float[4];
            for (int j = 0; j < 4; j++)
                obstacle_list[i][j] = another_map.obstacle_list[i][j];
        }
        return *this;
    }

    ~map(){
        for(int i = 0; i < nbr_obstacles; i++)
            delete[] obstacle_list[i];
        delete[] obstacle_list;
    }

    void add_obstacles(float a_obstacle_list[][4], int a_nbr_obstacles){
        nbr_obstacles = a_nbr_obstacles;

        obstacle_list = new float*[nbr_obstacles];
        
        for (int i = 0; i < nbr_obstacles; i++){
            obstacle_list[i] = new float[4];
            for (int j = 0; j < 4; j++)
                obstacle_list[i][j] = a_obstacle_list[i][j];
        }
    }

    int operator() ( float x, float y ) {
        if(nbr_obstacles == 0 || obstacle_list == NULL)
            return 0;
        else{
            for(int i = 0; i < nbr_obstacles; i++){
                if(x >= obstacle_list[i][0] && x <= obstacle_list[i][1] && y >= obstacle_list[i][2] && y <= obstacle_list[i][3])
                    return 1;
            }
            return 0;
        }
    }

};
 
class node {
public:
    bool operator == (const node& o){
        return pos == o.pos;
    }
    bool operator == (const Position& o){
        return pos == o; 
    }
    bool operator < (const node& o){
        return dist + cost < o.dist + o.cost; 
    }
    Position pos, parent;
    float dist, cost;
};
 
class aStar {
public:
    aStar() {
        neighbours[5] = Position( -m.xres, -m.yres ); neighbours[7] = Position(  m.xres, -m.yres );
        neighbours[4] = Position( -m.xres,  m.yres ); neighbours[6] = Position(  m.xres,  m.yres );
        neighbours[3] = Position(  0,      -m.yres ); neighbours[0] = Position( -m.xres,  0 );
        neighbours[2] = Position(  0,       m.yres ); neighbours[1] = Position(  m.xres,  0 );
    }

    void astar_clear(){
        open.clear();
        closed.clear();
    }

    float calcDist( Position p ){
        // need a better heuristic
        float x = end.x - p.x;
        float y = end.y - p.y;
        return( x * x + y * y );
    }
 
    bool isValid( Position& p ) {
        return ( p.x > m.xmin && p.y > m.ymin && p.x < m.xmax && p.y < m.ymax );
    }
 
    bool existPoint( Position& p, float cost ) {
        std::list<node>::iterator i;

        // is p in the closed list ?
        i = std::find( closed.begin(), closed.end(), p );
        if( i != closed.end() ) {
            if(i->cost + i->dist < cost) 
                return true;
            else{ 
                closed.erase(i); 
                return false; 
            }
        }

        // is p in the open list ?
        i = std::find( open.begin(), open.end(), p );
        if( i != open.end() ) {
            if(i->cost + i->dist < cost) 
                return true;
            else { 
                open.erase(i); 
                return false; 
            }
        }
        return false;
    }
 
    bool fillOpen( node& n , int v) {
        float stepCost, nc, dist;
        Position neighbour;
 
        for( int x = 0; x < 8; x++ ) {
            // one can make diagonals have different cost
            stepCost = n.pos.distance_from(neighbour); //x < 4 ? 1 : 1;
            neighbour = n.pos + neighbours[x];
            if( neighbour.distance_from(end) < sqrt(pow(m.xres,2)+pow(m.yres,2)) ) 
                return true;
 
            if( isValid( neighbour ) && m( neighbour.x, neighbour.y ) != 1 ) {
                nc = stepCost + n.cost;
                dist = calcDist( neighbour );
                if( !existPoint( neighbour, nc + dist ) ) {
                    node m;
                    m.cost = nc;
                    m.dist = dist;
                    m.pos = neighbour; 
                    m.parent = n.pos;
                    open.push_back(m);
                }
            }else{
                if(v){
                    printf("point %f %f is valid %d, m %d\n", 
                        neighbour.x, neighbour.y,
                        isValid( neighbour ), m( neighbour.x, neighbour.y ));
                }
            }
        }
        return false;
    }
 
    bool search( const Position& s, const Position& e, map& mp , int v) {
        node n; end = e; start = s; m = mp;
        n.cost = 0.0; n.pos = s; n.dist = calcDist( s ); 
        open.push_back( n );
        while( !open.empty() ) {
            //open.sort();
            if(v){
                for( std::list<node>::iterator i = open.begin(); i != open.end(); i++ )
                    std::cout<< "{" << i->pos.x<< ", " << i->pos.y << "}";
                std::cout<< std::endl;
            }
            node n = open.front();
            open.pop_front();
            closed.push_back( n );
            if( fillOpen( n , v) ) return true;
        }
        return false;
    }


 
    int path( std::list<Position>& path ) {
        path.push_front( end );
        float cost = 1 + closed.back().cost; 
        path.push_front( closed.back().pos );
        Position parent = closed.back().parent;
 
        for( std::list<node>::reverse_iterator i = closed.rbegin(); i != closed.rend(); i++ ) {
            if( ( *i ).pos == parent && !( ( *i ).pos == start ) ) {
                path.push_front( ( *i ).pos );
                parent = ( *i ).parent;
            }
        }
        path.push_front( start );
        return cost;
    }
 
    map m; 
    Position end, start;
    Position neighbours[8];
    std::list<node> open;
    std::list<node> closed;
};
 
// int main( int argc, char* argv[] ) {
//     float margin = 0.15;
//     float obstacles[][4] = {{5-margin, 5.5+margin, 2.5-margin, 4+margin}, 
//                             {5-margin, 8+margin, 2.5-margin, 3+margin}, 
//                             {9-margin, 12+margin, 1-margin, 1.5+margin}, 
//                             {11.5-margin, 12+margin, 0-margin, 1.5+margin}};
//     map m;
//     m.add_obstacles(obstacles, 4);

//     point s(38,2), e(18,16);
//     aStar as;
 
//     if( as.search( s, e, m ) ) {
//         std::list<point> path;
//         int c = as.path( path );
//         for( float y = YMIN-0.5; y < YMAX+0.5; y+=0.2 ) {
//             for( float x = XMIN-0.5; x < XMAX+0.5; x+=0.2 ) {
//                 if( x < XMIN || y < YMIN || x > XMAX || y > YMAX || m( x, y ) == 1 )
//                     std::cout << "o";
//                     // printf("%f %f in obstacle\n", x, y);
//                 else {
//                     if( std::find( path.begin(), path.end(), point( (int)((x-XMIN)/0.2), (int)((y-YMIN)/0.2) ) )!= path.end() )
//                         std::cout << "x";
//                     else 
//                         std::cout << ".";
//                     // printf("outside obstacles\n");
//                 }
//             }
//             std::cout << "\n";
//         }
 
//         std::cout << "\nPath cost " << c << ": ";
//         for( std::list<point>::iterator i = path.begin(); i != path.end(); i++ ) {
//             std::cout<< "(" << ( *i ).x << ", " << ( *i ).y << ") ";
//         }
//     }
//     std::cout << "\n\n";
//     return 0;
// }
 