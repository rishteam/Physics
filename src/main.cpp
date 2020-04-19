#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>

#include <iostream>
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


int main() {

    /* 向量測試 */

    // Vector vec(3, 4);
    // Vector vec2(1, 0);
    // vector test:
    // std::cout << vec.getLength() << '\n';              // => 5
    // std::cout << vec.dot(vec2)               << '\n';  // => 3
    // std::cout << vec.projectLengthOnto(vec2) << '\n';  // => 3
    //std::cout << vec2 << '\n';  // = (-4, 3)
    //std::cout << vec.normalR()               << '\n';  // = (4, -3)

    /* Box 碰撞測試 */
    Box box1(300, 300, 100, 200);
    Box box2(200, 200, 300, 150);
    sf::Clock clock;

    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "My window");

    // run the program as long as the window is open
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }
        window.clear();
        // buffer.push_back(box1);
        // buffer.push_back(box2);

        float unit_time = clock.getElapsedTime().asSeconds();
        clock.restart();
        timer += unit_time;


        if (sf::Keyboard::isKeyPressed(sf::Keyboard::W))
            box1.set_y(box1.get_y() - UNIT);
        else if (sf::Keyboard::isKeyPressed(sf::Keyboard::S))
            box1.set_y(box1.get_y() + UNIT);
        else if (sf::Keyboard::isKeyPressed(sf::Keyboard::A))
            box1.set_x(box1.get_x() - UNIT);
        else if (sf::Keyboard::isKeyPressed(sf::Keyboard::D))
            box1.set_x(box1.get_x() + UNIT);
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Up))
            box2.set_y(box2.get_y() - UNIT);
        else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Down))
            box2.set_y(box2.get_y() + UNIT);
        else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left))
            box2.set_x(box2.get_x() - UNIT);
        else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right))
            box2.set_x(box2.get_x() + UNIT);

        if (timer > DELAY)
        {
            if (cnt == 360)
                cnt = 0;
            else
                cnt += 1;
            timer = 0;
        }
        box1.setAngle(cnt);
        box2.setAngle(cnt);

        //test
        if (box1.SAT_collision(box2))
        {
            box1.draw(window, true);
            box2.draw(window, true);
        }
        else
        {
            box1.draw(window, false);
            box2.draw(window, false);
        }
        window.display();
    }
}