#include "vector_math.h"
#include <deque>
#include <SFML/Graphics.hpp>
#include <SFML/Audio.hpp>

class Box{
public:
    Box(double x, double y, double w, double h);
    ~Box() = default;
    //取分離軸
    void set_x(double x);
    void set_y(double y);
    double get_x();
    double get_y();
    double get_w();
    double get_h();
    std::deque<Vector> getSAT();
    //取頂點
    std::deque<Vector> getVertices();
    void setAngle(double angle);
    void setVertices();
    //拿所有法向量，存SAT
    void findSAT();
    std::pair<double, double> getMinMax(Vector &axis, std::deque<Vector> corner);
    bool SAT_collision(Box &other);
    void draw(sf::RenderWindow &window, bool coll);

private:
    //頂點
    std::deque<Vector> corner;
    //
    std::deque<Vector> Vertices;
    //分離軸
    std::deque<Vector> SAT;
    Vector center;
    double _angle = 0;
    double _w, _h;
};