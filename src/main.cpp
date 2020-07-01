#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>

#include <iostream>
#include "fmt/core.h"
#include "vector_math.h"

#include "Box.h"
#include "Circle.h"
#include "Polygon.h"

#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 600
#define UNIT 1
#define OBJ_COUNT 10


double timer = 0;
int cnt = 220;
int cnt2 = 0;

sf::RenderWindow window(sf::VideoMode(800, 600), "SAT collision");
std::deque<Shape> obj;

int random(int min, int max)
{
    srand(time(NULL));
    int x = rand() % (max - min + 1) + min;
    return x;
}

void vector_math()
{
    /* Vector Math */
    Vector vec(3, 4);
    Vector vec2(1, 0);
    fmt::print("{}\n", vec.getLength());             // => 5
    fmt::print("{}\n", vec.dot(vec2));            // => 3
    fmt::print("{}\n", vec.projectLengthOnto(vec2));   // => 3
    vec2.print_Vector();                         // = (-4, 3)
    vec.normalR().print_Vector();                // = (4, -3)
}

//keyboard_test
void keyboard_move(Shape *a, Shape *b)
{
    auto pos_a = a->getPosition();
    auto pos_b = b->getPosition();
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::W))
        pos_a.y -= UNIT;
    else if (sf::Keyboard::isKeyPressed(sf::Keyboard::S))
        pos_a.y += UNIT;
    else if (sf::Keyboard::isKeyPressed(sf::Keyboard::A))
        pos_a.x -= UNIT;
    else if (sf::Keyboard::isKeyPressed(sf::Keyboard::D))
        pos_a.x += UNIT;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Up))
        pos_b.y -= UNIT;
    else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Down))
        pos_b.y += UNIT;
    else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left))
        pos_b.x -= UNIT;
    else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right))
        pos_b.x += UNIT;

    a->setPosition(pos_a);
    auto pos = sf::Mouse::getPosition(window);
    b->setPosition(pos.x, pos.y);
}

// judge collision
void judge(Shape *a, Shape *b)
{
    if (a->isCollide(*b))
    {
        a->selected = true;
        b->selected = true;
    }
    else
    {
        a->selected = false;
        b->selected = false;
    }
}

int main()
{
    window.setFramerateLimit(60);

    // for (int i = 0; i < OBJ_COUNT; i++)
    //     obj.push_back(Circle(random(0,800), random(0,800), 50));

    //Shape objects
    Shape *box = new Box(400, 400, 100, 200);
    Shape *box2 = new Box(500, 500, 100, 200);
    Shape *cir = new Circle(300, 300, 100);
    Shape *cir2 = new Circle(200, 200, 100);
    std::deque<Vector> tmp;
    tmp.push_back({0, 0});
    tmp.push_back({0, 100});
    tmp.push_back({250, 50});
    tmp.push_back({300, 210});
    tmp.push_back({70, 20});
    Shape *poly = new Polygon(tmp, Vector(300, 200));
    std::deque<Vector> tmp2;
    tmp2.push_back({-100, -100});
    tmp2.push_back({0, 100});
    tmp2.push_back({250, 50});
    Shape *poly2 = new Polygon(tmp2, Vector(200, 200));

    sf::ConvexShape polygon;

    // run the program as long as the window is open
    while (window.isOpen())
    {
        //detect close event
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
            if (event.type == sf::Event::Resized)
            {
                sf::FloatRect visibleArea(0.f, 0.f, event.size.width, event.size.height);
                window.setView(sf::View(visibleArea));
            }
            if (event.type == sf::Event::MouseWheelScrolled)
            {
                if (event.mouseWheelScroll.wheel == sf::Mouse::VerticalWheel)
                    std::cout << "wheel type: vertical" << std::endl;
                else if (event.mouseWheelScroll.wheel == sf::Mouse::HorizontalWheel)
                    std::cout << "wheel type: horizontal" << std::endl;
                else
                    std::cout << "wheel type: unknown" << std::endl;
                std::cout << "wheel ent: " << event.mouseWheelScroll.delta << std::endl;
                std::cout << "mouse x: " << event.mouseWheelScroll.x << std::endl;
                std::cout << "mouse y: " << event.mouseWheelScroll.y << std::endl;
            }
            if (event.type == sf::Event::MouseButtonPressed)
            {
                if (event.mouseButton.button == sf::Mouse::Left)
                {
                    std::cout << "the right button was pressed" << std::endl;
                    std::cout << "mouse x: " << event.mouseButton.x << std::endl;
                    std::cout << "mouse y: " << event.mouseButton.y << std::endl;
                }
            }
        }

        // box->setRotation(cnt);
        // fmt::print("{}\n", cnt);
        poly2->setRotation(cnt);
        keyboard_move(poly2, cir);
        judge(poly2, cir);

        if (cnt == 360)
            cnt = 0;
        else
            cnt += 1;

        if (cnt2 == 0)
            cnt2 = 360;
        else
            cnt2 -= 1;

        // clear screen
        window.clear(sf::Color::Black);
        poly2->set_debug_draw();
        window.draw(*poly2);
        cir->set_debug_draw();
        window.draw(*cir);
        window.display();
    }
}