#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>

#include <iostream>
#include "fmt/core.h"
#include "vector_math.h"
#include "Box.h"

#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 600
#define UNIT 1
#define DELAY 0.1

// Box judge(Box &box)
// {
//     Box tmp = buffer[0];
//     buffer.pop_front();
//     if (box.get_x() - box.get_w() / 2 <= 0 ||
//         box.get_x() + box.get_w() / 2 >= WINDOW_WIDTH ||
//         box.get_y() - box.get_h() / 2 <= 0 ||
//         box.get_y() + box.get_h() / 2 >= WINDOW_HEIGHT)
//     {
//         // std::cout << "out of range" << '\n';
//         return tmp;
//     }
//     else
//     {
//         // std::cout << "Inside" << '\n';
//         return box;
//     }
// }
double timer = 0;
int cnt = 0;
int cnt2 = 360;


int main() {

    /* 向量測試 */
    // Vector vec(3, 4);
    // Vector vec2(1, 0);
    // fmt::print("{}\n", vec.getLength());             // => 5
    // fmt::print("{}\n", vec.dot(vec2));            // => 3
    // fmt::print("{}\n", vec.projectLengthOnto(vec2));   // => 3
    // vec2.print_Vector();                         // = (-4, 3)
    // vec.normalR().print_Vector();                // = (4, -3)

    /* Box 碰撞測試 */
    Box box1(200, 200, 100, 200);
    Box box2(0, 0, 300, 150);
    sf::Clock clock;

    sf::RenderWindow window(sf::VideoMode(800, 600), "My window");

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
        //rotate angle per unit_time
        if (timer > DELAY)
        {
            if (cnt == 360)
                cnt = 0;
            else
                cnt += 1;

            if (cnt2 == 0)
                cnt2 = 360;
            else
                cnt2 -= 1;
            timer = 0;
        }

        //get position data
        sf::Vector2f box1_pos = box1.getPosition();
        sf::Vector2f box2_pos = box2.getPosition();

        if (sf::Keyboard::isKeyPressed(sf::Keyboard::W))
            box1_pos.y -= UNIT;
        else if (sf::Keyboard::isKeyPressed(sf::Keyboard::S))
            box1_pos.y += UNIT;
        else if (sf::Keyboard::isKeyPressed(sf::Keyboard::A))
            box1_pos.x -= UNIT;
        else if (sf::Keyboard::isKeyPressed(sf::Keyboard::D))
            box1_pos.x += UNIT;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Up))
            box2_pos.y -= UNIT;
        else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Down))
            box2_pos.y += UNIT;
        else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left))
            box2_pos.x -= UNIT;
        else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right))
            box2_pos.x += UNIT;

        //update position data
        box1.setPosition(box1_pos);
        box2.setPosition(box2_pos);
        box1.setRotation(cnt);
        box2.setRotation(cnt2);

        // //judge test
        if (box1.isCollide(box2))
        {
            fmt::print("Collid\n");
        }
        else
        {
            fmt::print("No Collide\n");
        }
        window.clear(sf::Color::Black);
        box1.set_draw();
        box2.set_draw();
        window.draw(box1);
        window.draw(box2);
        window.display();
    }
}