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
int cnt = 0;
int cnt2 = 360;

void vector_math()
{
    /* Vector Math */
    // Vector vec(3, 4);
    // Vector vec2(1, 0);
    // fmt::print("{}\n", vec.getLength());             // => 5
    // fmt::print("{}\n", vec.dot(vec2));            // => 3
    // fmt::print("{}\n", vec.projectLengthOnto(vec2));   // => 3
    // vec2.print_Vector();                         // = (-4, 3)
    // vec.normalR().print_Vector();                // = (4, -3)
}

int main() {

    sf::Clock clock;
    sf::RenderWindow window(sf::VideoMode(800, 600), "SAT collision test");

    // run the program as long as the window is open
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        //timer
        float unit_time = clock.getElapsedTime().asSeconds();
        clock.restart();
        timer += unit_time;

        // build three object
        Shape *cir = new Circle(300, 300, 200);
        /* polygon collision test */
        // std::deque<Vector> tmp;
        // tmp.push_back({100, 120});
        // tmp.push_back({120, 130});
        // tmp.push_back({150, 160});
        // tmp.push_back({200, 200});
        // tmp.push_back({300, 250});

        // Shape *poly = new Polygon(tmp, Vector(150, 130));
        Shape *box = new Box(400, 400, 100, 200);

        //get position data
        sf::Vector2f box_pos = box->getPosition();
        sf::Vector2f cir_pos = cir->getPosition();
        // sf::Vector2f poly_pos = poly->getPosition();
        // fmt::print("{} {}\n", poly_pos.x, poly_pos.y);

        //rotate angle per unit_time
        // if (timer > DELAY)
        // {
        //     if (cnt == 360)
        //         cnt = 0;
        //     else
        //         cnt += 1;

        //     if (cnt2 == 0)
        //         cnt2 = 360;
        //     else
        //         cnt2 -= 1;
        //     timer = 0;
        // }


        // if (sf::Keyboard::isKeyPressed(sf::Keyboard::W))
        //     box1_pos.y -= UNIT;
        // else if (sf::Keyboard::isKeyPressed(sf::Keyboard::S))
        //     box1_pos.y += UNIT;
        // else if (sf::Keyboard::isKeyPressed(sf::Keyboard::A))
        //     box1_pos.x -= UNIT;
        // else if (sf::Keyboard::isKeyPressed(sf::Keyboard::D))
        //     box1_pos.x += UNIT;
        // if (sf::Keyboard::isKeyPressed(sf::Keyboard::Up))
        //     box_pos.y -= UNIT;
        // else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Down))
        //     box_pos.y += UNIT;
        // else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left))
        //     box_pos.x -= UNIT;
        // else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right))
        //     box_pos.x += UNIT;

        // update position data
        // box1.setPosition(box1_pos);
        // box2.setPosition(box2_pos);
        // setting rotate angle
        // box1.setRotation(cnt);
        // box2.setRotation(cnt2);

        // judge collision
        if (cir->isCollide(*box))
        {
            
            cir->selected = true;
            box->selected = true;
            fmt::print("collide\n");
        }
        else
        {
            cir->selected = false;
            box->selected = false;
            fmt::print("not collide\n");
        }

        // clear screen
        window.clear(sf::Color::Black);
        // use SFML::drawable to draw box1, box2
        box->set_debug_draw();
        window.draw(*box);
        cir->set_debug_draw();
        window.draw(*cir);
        // poly->set_debug_draw();
        // window.draw(*poly);
        window.display();
    }
}