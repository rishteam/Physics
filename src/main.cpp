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
#define DELAY 0.1

double timer = 0;
int cnt = 220;
int cnt2 = 80;

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
    b->setPosition(pos_b);
}

// judge collision
void judge(Shape *a, Shape *b)
{
    if (a->isCollide(*b))
    {
        a->selected = true;
        b->selected = true;
        // fmt::print("collide\n");
    }
    else
    {
        a->selected = false;
        b->selected = false;
        // fmt::print("not collide\n");
    }
}


int main()
{
    sf::RenderWindow window(sf::VideoMode(800, 600), "SAT collision");
    window.setFramerateLimit(60);

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
    tmp2.push_back({0, 0});
    tmp2.push_back({0, 100});
    tmp2.push_back({250, 50});
    tmp.push_back({300, 210});
    tmp.push_back({70, 20});
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

        // update position data
        // setting rotate angle
        keyboard_move(poly, poly2);
        judge(poly, poly2);

        // clear screen
        window.clear(sf::Color::Black);
        poly->set_debug_draw();
        window.draw(*poly);
        poly2->set_debug_draw();
        window.draw(*poly2);
        // cir2->set_debug_draw();
        // window.draw(*cir2);
        // box->set_debug_draw();
        // window.draw(*box);
        // box2->set_debug_draw();
        // window.draw(*box2);
        // poly->set_debug_draw();
        // window.draw(*poly);
        // poly->set_debug_draw();
        window.display();
    }
}