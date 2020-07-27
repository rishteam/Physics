#include "vector_math.h"

// 定義形狀
//
//        ^ y
//        |
//        e1
//   v2 ------ v1
//    |        |
// e2 |        | e4  --> x
//    |        |
//   v3 ------ v4
//        e3

class FeaturePair
{
public:
    struct Edges
    {
        char inEdge1;
        char outEdge1;
        char inEdge2;
        char outEdge2;
    } e;
    int value;
};


class ClipVertex
{
public:
    FeaturePair fp;
    ClipVertex() { fp.value = 0; }
    Vec2 v;
};

class Vclip
{
public:
    //計算軸
    enum Axis
    {
        FACE_A_X,
        FACE_A_Y,
        FACE_B_X,
        FACE_B_Y
    };

    //邊
    enum EdgeNumbers
    {
        NO_EDGE = 0,
        EDGE1,
        EDGE2,
        EDGE3,
        EDGE4
    };
    void ClipSegmentToLine(ClipVertex *vOut, ClipVertex *vIn, const Vec2& normal, float offset, char clipEdge);
    void Flip(FeaturePair& fp)
    {
        Swap(fp.e.inEdge1, fp.e.inEdge2);
        Swap(fp.e.outEdge1, fp.e.outEdge2);
    }
};