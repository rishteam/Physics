#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>

#include <iostream>
#include "fmt/core.h"
#include "vector_math.h"

#include "World.h"
#include "Box.h"
#include "Circle.h"
#include "Polygon.h"

#include "imgui.h"
#include "imgui-SFML.h"

#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 600
#define UNIT 1
#define OBJ_COUNT 10

sf::Font font;
double timer = 0;
int cnt = 220;
int cnt2 = 0;

sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH,  WINDOW_HEIGHT), "physics(SAT) example");
std::vector <Shape*> obj;


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
void judge()
{
    //clear all
    for (int i = 0; i < obj.size(); i++)
    {
        obj[i]->selected = false;
    }

    //judge
    for (int i = 0; i < obj.size(); i++)
    {
        for(int j = 0; j < obj.size(); j++)
        {
            if (i != j && obj[i]->isCollide(*obj[j]))
            {
                obj[i]->selected = true;
                obj[j]->selected = true;
            }
        }
    }
}

void draw_obj(World& world)
{
    int idx = 0;
    for(auto &obj : world.bodies)
    {
        obj->set_debug_draw();
//        fmt::print("{}: ({}, {})\n", idx++, obj->getPosition().x, obj->getPosition().y);
        window.draw(*obj);
    }
}


void generate_obj()
{
    srand(time(NULL));
    for (int i = 0; i < OBJ_COUNT; i++)
    {
        obj.push_back(new Circle(randomint(0, 800), randomint(0, 600), randomint(30, 50)));
    }
}


void rotate()
{
    if (cnt == 360)
        cnt = 0;
    else
        cnt += 1;

    if (cnt2 == 0)
        cnt2 = 360;
    else
        cnt2 -= 1;
}


int main()
{
    window.setFramerateLimit(60);
    ImGui::SFML::Init(window);
    World world(Vec2(0.0, -9.8), (float)WINDOW_WIDTH, (float)WINDOW_HEIGHT);
    //Shape objects
    Shape *box = new Box(400, 500, 800, 100, MAX_float);
    Shape *box2 = new Box(400, 200, 50, 25, 1000000000);
    Shape *cir = new Circle(300, 300, 100);
    Shape *cir2 = new Circle(200, 200, 100);
    std::deque<Vec2> tmp;
    tmp.push_back({0, 0});
    tmp.push_back({0, 100});
    tmp.push_back({250, 50});
    tmp.push_back({300, 210});
    tmp.push_back({70, 20});
    Shape *poly = new Polygon(tmp, Vec2(300, 200));
    std::deque<Vec2> tmp2;
    tmp2.push_back({-100, -100});
    tmp2.push_back({0, 100});
    tmp2.push_back({250, 50});
    Shape *poly2 = new Polygon(tmp2, Vec2(200, 200));

    world.Add(box);
    world.Add(box2);





    sf::Clock deltaClock;
    // run the program as long as the window is open
    while (window.isOpen())
    {
        //detect close event
        sf::Event event;
        while (window.pollEvent(event))
        {
            ImGui::SFML::ProcessEvent(event);

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

        ImGui::SFML::Update(window, deltaClock.restart());

        ImGui::Begin("New object ");
        if (ImGui::Button("click new object"))
        {
            float tmp_x = randomint(0, 800), tmp_y = 100, tmp_w = 25, tmp_h = 25 , tmp_mass = 100;
            Shape *new_box = new Box(tmp_x, tmp_y, tmp_w, tmp_h, tmp_mass);
            world.Add(new_box);
        }
        ImGui::End();

        world.Step(world.timeStep);

        // keyboard_move(poly2, cir);
        // judge();
        // rotate();

        // clear screen
        window.clear(sf::Color::Black);
        draw_obj(world);
        ImGui::SFML::Render(window);
        window.display();
    }
}