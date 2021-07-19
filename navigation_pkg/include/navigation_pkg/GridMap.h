//
// Created by stdcat on 3/3/21.
//

#ifndef SRC_GRIDMAP_H
#define SRC_GRIDMAP_H

#include "common_include.h"

typedef enum inflationType_e{
    I_RECTANGLE = 0,
    I_ROUND
}inflationType_e;

class GridMap_t{
private:
    int map_width;
    int map_height;
    double map_reso;
    double origin_x;
    double origin_y;
    inflationType_e inflationType;

    vector<int> data;/* 0 for free 1 for obstacle */
    vector<int> preData;/* 0 for free 1 for obstacle */
    vector<bool> _hash;/* Specify Origin Data */
public:


    GridMap_t( int map_width, int map_height, double map_reso, double origin_x, double origin_y, vector<int8_t> gridData){
        this->map_width = map_width;
        this->map_height = map_height;
        this->map_reso = map_reso;
        this->origin_x = origin_x;
        this->origin_y = origin_y;
        this->inflationType = I_RECTANGLE;

        data.resize(map_width*map_height);
        _hash.resize(map_width*map_height);

        for (int i = 0; i < data.size(); ++i) {
            data[i] = (gridData[i] >=0 && gridData[i] <= 30)? 0:1;
            _hash[i] = !(gridData[i] >= 0 && gridData[i] <= 30);
            preData.emplace_back(-1);
        }
    }

    GridMap_t( int map_width, int map_height, double map_reso, double origin_x, double origin_y, vector<int8_t> gridData, inflationType_e type_i){
        this->map_width = map_width;
        this->map_height = map_height;
        this->map_reso = map_reso;
        this->origin_x = origin_x;
        this->origin_y = origin_y;
        this->inflationType = type_i;

        data.resize(map_width*map_height);
        _hash.resize(map_width*map_height);
        for (int i = 0; i < data.size(); ++i) {
            data[i] = (gridData[i] >=0 && gridData[i] <= 30)? 0:1;
            _hash[i] = !(gridData[i] >= 0 && gridData[i] <= 30);
            preData.emplace_back(-1);
        }
    }

    GridMap_t(){
        this->map_width = 0;
        this->map_height = 0;
        this->map_reso = 0;
        this->origin_x = 0;
        this->origin_y = 0;
        this->inflationType = I_RECTANGLE;
        data.clear();
        _hash.clear();
        preData.clear();
    }

    void inflate(double inflation_r){
//        for (int i = 0; i < _hash.size(); ++i) {
//            if (_hash[i]) data[i] = 1;
//            else data[i] = 0;
//        }
        if (inflationType == I_RECTANGLE){
            for (int i = 0; i < data.size(); ++i) {

                if ( _hash[i] ){
                    Vector2d coord = getXY_from_GridIndex(i);

                    int _t = int(inflation_r / map_reso);
                    for (int k = 0; k < _t ; ++k) {
                        for (int l = 0; l < _t; ++l) {

                            int inflation_gridIndex = get_gridIndex_from_XY(coord.x()+k*map_reso, coord.y()+l*map_reso);
                            if(inflation_gridIndex != -1){
                                if( data[inflation_gridIndex] == 0 ) {
                                    data[inflation_gridIndex] = 3;
                                }
                            }

                            inflation_gridIndex = get_gridIndex_from_XY(coord.x()+k*map_reso, coord.y()-l*map_reso);
                            if(inflation_gridIndex != -1){
                                if( data[inflation_gridIndex] == 0 ) {
                                    data[inflation_gridIndex] = 3;
                                }
                            }

                            inflation_gridIndex = get_gridIndex_from_XY(coord.x()-k*map_reso, coord.y()+l*map_reso);
                            if(inflation_gridIndex != -1){
                                if( data[inflation_gridIndex] == 0 ) {
                                    data[inflation_gridIndex] = 3;
                                }
                            }

                            inflation_gridIndex = get_gridIndex_from_XY(coord.x()-k*map_reso, coord.y()-l*map_reso);
                            if(inflation_gridIndex != -1){
                                if( data[inflation_gridIndex] == 0 ) {
                                    data[inflation_gridIndex] = 3;
                                }
                            }
                        }
                    }
                }
            }
        }
        else if ( inflationType == I_ROUND){
            for (int i = 0; i < data.size(); ++i) {

               if ( _hash[i] ){
                    Vector2d coord = getXY_from_GridIndex(i);

                    int _t = int(inflation_r / map_reso);
                    double d_theta = 2 * pi / 50;

                    int inflation_gridIndex;
                    for(int r=0; r < 1.05*_t; r++) {
                        for (double theta = 0; theta <= 2 * pi; theta += d_theta) {
                            inflation_gridIndex = get_gridIndex_from_XY(coord.x() + r * cos(theta) * map_reso,
                                                                        coord.y() + r * sin(theta) * map_reso);
                            if (inflation_gridIndex != -1) {
                                if (data[inflation_gridIndex] == 0) {
                                    data[inflation_gridIndex] = 1;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    void inflate(double inner_r, double extern_r){
        for (int i = 0; i < data.size(); ++i) {
            if ( _hash[i] ){

                Vector2d coord = getXY_from_GridIndex(i);

                int _t = int((inner_r) / map_reso)+1;
                for (int k = 0; k < _t ; ++k) {
                    for (int l = 0; l < _t; ++l) {

                        int inflation_gridIndex = get_gridIndex_from_XY(coord.x()+k*map_reso, coord.y()+l*map_reso);
                        if(inflation_gridIndex != -1){
                            if( data[inflation_gridIndex] == 0 ) {
                                data[inflation_gridIndex] = 2;

                            }
                        }

                        inflation_gridIndex = get_gridIndex_from_XY(coord.x()+k*map_reso, coord.y()-l*map_reso);
                        if(inflation_gridIndex != -1){
                            if( data[inflation_gridIndex] == 0 ) {
                                data[inflation_gridIndex] = 2;

                            }
                        }

                        inflation_gridIndex = get_gridIndex_from_XY(coord.x()-k*map_reso, coord.y()+l*map_reso);
                        if(inflation_gridIndex != -1){
                            if( data[inflation_gridIndex] == 0 ) {
                                data[inflation_gridIndex] = 2;

                            }
                        }

                        inflation_gridIndex = get_gridIndex_from_XY(coord.x()-k*map_reso, coord.y()-l*map_reso);
                        if(inflation_gridIndex != -1){
                            if( data[inflation_gridIndex] == 0 ) {
                                data[inflation_gridIndex] = 2;

                            }
                        }
                    }
                }
            }
        }
        for (int i = 0; i < data.size(); ++i) {
            if ( _hash[i] ){
                Vector2d coord = getXY_from_GridIndex(i);

                int _t = int((inner_r+extern_r) / map_reso)+1;
                for (int k = 0; k < _t ; ++k) {
                    for (int l = 0; l < _t; ++l) {

                        int inflation_gridIndex = get_gridIndex_from_XY(coord.x()+k*map_reso, coord.y()+l*map_reso);
                        if(inflation_gridIndex != -1){
                            if( data[inflation_gridIndex] == 0 ) {
                                data[inflation_gridIndex] = 3;

                            }
                        }

                        inflation_gridIndex = get_gridIndex_from_XY(coord.x()+k*map_reso, coord.y()-l*map_reso);
                        if(inflation_gridIndex != -1){
                            if( data[inflation_gridIndex] == 0 ) {
                                data[inflation_gridIndex] = 3;

                            }
                        }

                        inflation_gridIndex = get_gridIndex_from_XY(coord.x()-k*map_reso, coord.y()+l*map_reso);
                        if(inflation_gridIndex != -1){
                            if( data[inflation_gridIndex] == 0 ) {
                                data[inflation_gridIndex] = 3;

                            }
                        }

                        inflation_gridIndex = get_gridIndex_from_XY(coord.x()-k*map_reso, coord.y()-l*map_reso);
                        if(inflation_gridIndex != -1){
                            if( data[inflation_gridIndex] == 0 ) {
                                data[inflation_gridIndex] = 3;

                            }
                        }
                    }
                }
            }
        }

    }

    int get_gridIndex_from_XY(double x, double y){


        int i = int((y - origin_y)/map_reso);
        int j = int((x - origin_x)/map_reso);
        if (i<0 || j<0 || i>= map_height || j>=map_width){
            return -1;
        }
        int grid_index = i*map_width + j;
        if(grid_index > map_width*map_height) return -1;
        return grid_index;
    }

    int get_gridIndex_from_XY(Vector2d src){


        int i = int((src.y() - origin_y)/map_reso);
        int j = int((src.x() - origin_x)/map_reso);
        if (i<0 || j<0 || i>= map_height || j>=map_width){
            return -1;
        }
        int grid_index = i*map_width + j;
        if(grid_index > map_width*map_height) return -1;
        return grid_index;
    }

    int get_gridIndex_from_GridXY(Vector2i src){
        if (src.x()<0 || src.y()<0 || src.x()>=map_width || src.y()>= map_height) return -1;
        int gridIndex = src.y()*map_width+src.x();
        return gridIndex;
    }

    int get_gridIndex_from_GridXY(int x, int y){
        if (x<0 || y<0 || x>=map_width || y >= map_height) return -1;
        int gridIndex = y*map_width+x;
        return gridIndex;
    }

    Vector2d getXY_from_GridIndex(int gridIndex){
        Vector2d  res;
        int i,j;
        i = int(gridIndex/map_width);
        j = int(gridIndex%map_width);
        res.x() = j*map_reso + origin_x;
        res.y() = i*map_reso + origin_y;
        return res;
    }

    Vector2d getXY_from_GridXY(int x, int y){
        double _x, _y;
        _x = x*map_reso + origin_x;
        _y = y*map_reso + origin_y;
        Vector2d res(_x,_y);
        return res;
    }

    Vector2d getXY_from_GridXY(Vector2i src){
        double _x, _y;
        _x = src.x()*map_reso + origin_x;
        _y = src.y()*map_reso + origin_y;
        Vector2d res(_x,_y);
        return res;
    }

    Vector2i getGridXY_from_GridIndex(int gridIndex){
        int i,j;
        i = int(gridIndex%map_width);
        j = int(gridIndex/map_width);
        Vector2i  res(i,j);
        return res;
    }

    int get_Data_from_GridXY(int x, int y){
        if (x <0 || y<0|| x>=map_width || y>=map_height) return -1;
        int grid_index = y*map_width + x;
        if(grid_index > map_width*map_height) return -1;
        return data[grid_index];
    }



    int width(){
        return map_width;
    }

    int height(){
        return map_height;
    }

    int size(){
        return data.size();
    }

    double reso(){
        return map_reso;
    }

    int& operator [](int index){
        if(index >= data.size()){
            return data[0];
        }
        return data[index];
    }

    _Bit_reference hash(int index){

        if (index >= _hash.size()){
             return _hash[0];
        }
        return _hash[index];
    }

    void pubMsg(ros::Publisher pub){
        visualization_msgs::MarkerArray gridMapMsg;
        gridMapMsg.markers.clear();
        int _id = 0;
        if(data.size() == 0) return;
        for (int i = 0; i < data.size(); ++i) {
            if (data[i] != 0){
                if (data[i] == preData[i]) continue;
                Vector2d coord = getXY_from_GridIndex(i);
                visualization_msgs::Marker grid;

                grid.header.frame_id = "world";

                grid.ns = "grid_map";
                grid.id = _id++;
                grid.action = visualization_msgs::Marker::ADD;
                grid.type = visualization_msgs::Marker::CUBE;

                grid.color.a = 0.5;
                if (data[i] == 1){
                    grid.color.r = 0;
                    grid.color.g = 0;
                    grid.color.b = 0;

                }
                else if(data[i] == 2){
                    grid.color.r = 125;
                    grid.color.g = 100;
                    grid.color.b = 0;

                }
                else if(data[i] == 3){
                    grid.color.r = 0;
                    grid.color.g = 255;
                    grid.color.b = 0;

                }

                grid.scale.x = map_reso;
                grid.scale.y = map_reso;
                grid.scale.z = map_reso;

                grid.pose.position.x = coord.x() ;
                grid.pose.position.y = coord.y() ;
                grid.pose.position.z = map_reso/2;

                gridMapMsg.markers.push_back(grid);
            }
        }
        preData = data;
        pub.publish(gridMapMsg);
    }

};


#endif //SRC_GRIDMAP_H
