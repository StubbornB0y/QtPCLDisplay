#include<nlohmann/json.hpp>
#include "BoundingBox.h"
#include <iostream>
using json = nlohmann::json;
void from_json(const json& j, BoundingBoxParameter& bbox) {
    j.at("x").get_to(bbox.x);
    j.at("y").get_to(bbox.y);
    j.at("z").get_to(bbox.z);
    j.at("w").get_to(bbox.w);
    j.at("l").get_to(bbox.l);
    j.at("h").get_to(bbox.h);
    j.at("rt").get_to(bbox.rt);
    j.at("id").get_to(bbox.id);
    j.at("score").get_to(bbox.score);
}

BoundingBox::BoundingBox()
{
    
}

BoundingBox::BoundingBox(std::string jsonstring)
{
    analysisString(jsonstring);
}

std::vector<BoundingBoxParameter> BoundingBox::analysisString(std::string jsonstring)
{
    try{
        json jsonObject = json::parse(jsonstring);
        std::cout <<jsonObject<<std::endl;
        if(jsonObject.contains("BoundingBox") && !jsonObject.is_null()){
            boxes_array = jsonObject["BoundingBox"].get<std::vector<BoundingBoxParameter>>();
            std::cout <<boxes_array.size()<<std::endl;
        }
    }catch (const std::exception &e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }
    return boxes_array;
    
}