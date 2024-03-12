#include <vector>
struct BoundingBoxParameter {
    float x;
    float y;
    float z;
    float w;
    float l;
    float h;
    float rt;
    int id;
    float score;
};
class BoundingBox
{
private:
    std::vector<BoundingBoxParameter> boxes_array;
    /* data */
public:
    BoundingBox();
    BoundingBox(std::string jsonstring);
    std::vector<BoundingBoxParameter> analysisString(std::string jsonstring);

    const std::vector<BoundingBoxParameter> getParameter(){return boxes_array;}
};

