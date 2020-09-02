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

#define WINDOW_WIDTH 1280
#define WINDOW_HEIGHT 720
#define UNIT 1
#define OBJ_COUNT 10

sf::Font font;
double timer = 0;
int cnt = 220;
int cnt2 = 0;
static bool f_keepSimulate = true;
static bool f_showContactPoints = true;

World world(Vec2(0.0, -9.8), 800.0f, 600.0f);
sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH,  WINDOW_HEIGHT), "Physics");
//std::vector <Shape*> obj;


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
//void judge()
//{
//    //clear all
//    for (int i = 0; i < obj.size(); i++)
//    {
//        obj[i]->selected = false;
//    }
//
//    //judge
//    for (int i = 0; i < obj.size(); i++)
//    {
//        for(int j = 0; j < obj.size(); j++)
//        {
//            if (i != j && obj[i]->isCollide(*obj[j]))
//            {
//                obj[i]->selected = true;
//                obj[j]->selected = true;
//            }
//        }
//    }
//}

void draw_obj(World& world)
{
    int idx = 0;
    //show world bodies
    for(auto &obj : world.bodies)
    {
        obj->set_debug_draw();
        window.draw(*obj);
   }

    for(auto &jit : world.joints)
    {
        Box* box1 = dynamic_cast<Box*>(jit->b1);
        Box* box2 = dynamic_cast<Box*>(jit->b2);

        Mat22 R1(box1->angle);
        Mat22 R2(box2->angle);

        Vec2 x1 = box1->position;
        Vec2 p1 = x1 + R1 * jit->localAnchor1;

        Vec2 x2 = box2->position;
        Vec2 p2 = x2 + R2 * jit->localAnchor2;

        Vec2 screen_x1 = World::ConvertWorldToScreen(x1);
        Vec2 screen_p1 = World::ConvertWorldToScreen(p1);

        Vec2 screen_x2 = World::ConvertWorldToScreen(x2);
        Vec2 screen_p2 = World::ConvertWorldToScreen(p2);

        sf::Vertex conn1[] = {
            sf::Vector2f(screen_x1.x, screen_x1.y),
            sf::Vector2f(screen_p1.x, screen_p1.y)
        };

        sf::Vertex conn2[] = {
            sf::Vector2f(screen_x2.x, screen_x2.y),
            sf::Vector2f(screen_p2.x, screen_p2.y)
        };

        window.draw(conn1, 2, sf::Lines);
        window.draw(conn2, 2, sf::Lines);
    }


    // show contact points
    if(f_showContactPoints)
    {
        for (auto iter = world.arbiters.begin(); iter != world.arbiters.end(); ++iter)
        {
            const Arbiter& arbiter = iter->second;
            for (int i = 0; i < arbiter.numContacts; ++i)
            {
                sf::CircleShape circle;
                Vec2 p = arbiter.contacts[i].position;
                Vec2 w = World::ConvertWorldToScreen(p);
                circle.setPosition(w.x - 3, w.y - 3);
                circle.setRadius(3);
                circle.setFillColor(sf::Color::White);
                window.draw(circle);
            }
        }
    }

}


//void generate_obj()
//{
//    srand(time(NULL));
//    for (int i = 0; i < OBJ_COUNT; i++)
//    {
//        obj.push_back(new Circle(randomint(0, 800), randomint(0, 600), randomint(30, 50)));
//    }
//}


//void rotate()
//{
//    if (cnt == 360)
//        cnt = 0;
//    else
//        cnt += 1;
//
//    if (cnt2 == 0)
//        cnt2 = 360;
//    else
//        cnt2 -= 1;
//}

void demo1()
{
    world.Clear();
    //地板
    Shape *floor = new Box(400, 500, 800, 100, MAX_float);
    world.Add(floor);
    //方塊
    Shape *box = new Box(400, 300, 30, 30, 1000);
    world.Add(box);
}

void demo2()
{
    world.Clear();
    //地板
    Shape *floor = new Box(400, 500, 800, 100, MAX_float);
    world.Add(floor);

    //方塊
    Shape* tmp = new Box(600, 200, 25, 25, 10);
    Box* box = dynamic_cast<Box*>(tmp);
    box->friction = 1.0f;
    world.Add(box);

    //關節點
    Joint* j = new Joint();
    j->Set(box, floor, Vec2(400, 200));
    world.AddJoints(j);
}

void demo3()
{
    world.Clear();
    Shape *skew = new Box(300, 370, 300, 10, MAX_float);
    skew->rotate(20);
    world.Add(skew);

    Shape *skew2 = new Box(500, 230, 300, 10, MAX_float);
    skew2->rotate(-20);
    world.Add(skew2);

    Shape *skew3 = new Box(300, 100, 300, 10, MAX_float);
    skew3->rotate(20);
    world.Add(skew3);

    Shape *obst1 = new Box(300, 280, 10, 50, MAX_float);
    world.Add(obst1);

    Shape *obst2 = new Box(500, 150, 10, 50, MAX_float);
    world.Add(obst2);

    Shape *floor = new Box(400, 500, 800, 50, MAX_float);
    world.Add(floor);

    float friction[5] = {0.75f, 0.5f, 0.35f, 0.1f, 0.0f};
    for (int i = 0; i < 5; ++i)
    {
        Shape *tmp = new Box(200 + (float)i * 25, 20, 20, 20, 10);
        Box* box = dynamic_cast<Box*>(tmp);
        box->friction = friction[i];
        world.Add(box);
    }
}

void demo4()
{
    world.Clear();
    Shape *floor = new Box(400, 500, 800, 100, MAX_float);
    world.Add(floor);

    for (int j = 0; j < 5; j++) {
        float y_idx = j * 55;
        Shape *tmp = new Box(400, 425 - y_idx, 50, 50, 10);
        Box *box = dynamic_cast<Box *>(tmp);
        box->friction = 0.5;
        world.Add(box);
    }
}

void demo5()
{
    world.Clear();
    Shape *floor = new Box(400, 500, 800, 100, MAX_float);
    world.Add(floor);

    for (int j = 0; j < 10; j++) {
        for (int i = 0; i < 10-j; ++i) {
            float x_idx = i * 70;
            float y_idx = j * 55;

            Shape *tmp = new Box((75 + 35 * j ) + x_idx, 400 - y_idx , 50, 50, 10);
            Box *box = dynamic_cast<Box *>(tmp);
            box->friction = 0.5;
            world.Add(box);
        }
    }
}


int main()
{
    window.setFramerateLimit(60);
    ImGui::SFML::Init(window);
    sf::Clock deltaClock;

    //Shape-circle objects use example
    Shape *cir = new Circle(300, 300, 100);

    //Shape-polygon objects use example
    std::deque<Vec2> tmp;
    tmp.push_back({0, 0});
    Shape *poly = new Polygon(tmp, Vec2(300, 200));


    while (window.isOpen())
    {
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
                std::cout << "the right button was pressed" << std::endl;
                std::cout << "mouse x: " << event.mouseButton.x << std::endl;
                std::cout << "mouse y: " << event.mouseButton.y << std::endl;
            }
        }


        //Render IMGUI
        ImGui::SFML::Update(window, deltaClock.restart());
        ImGui::Begin("Debug");

        static float x,y,w,h,m;
        ImGui::InputFloat("X", &x, 1.0f, 10.0f, "%.3f");
        ImGui::InputFloat("Y", &y, 1.0f, 10.0f, "%.3f");
        ImGui::InputFloat("W", &w, 1.0f, 10.0f, "%.3f");
        ImGui::InputFloat("H", &h, 1.0f, 10.0f, "%.3f");
        ImGui::InputFloat("Mass", &m, 1.0f, 10.0f, "%.3f");


        if (ImGui::Button("New box"))
        {
            Shape *new_box = new Box(x, y, w, h, m);
            world.Add(new_box);
        }

        ImGui::Separator();
        ImGui::Checkbox("Keep Simulate", &f_keepSimulate);
        ImGui::SameLine();
        ImGui::Checkbox("Show contact Points", &f_showContactPoints);

        if (ImGui::CollapsingHeader("Box's Data"))
        {
            for(int i = 0; i < world.bodies.size(); i++)
            {
                Box* box = dynamic_cast<Box*>(world.bodies.at(i));
                ImGui::Text("Box%d:", i);
                ImGui::Text("[Physics] Center: (%f, %f)", box->getPhysicsData().first.x, box->getPhysicsData().first.y);
                ImGui::Text("[Physics] width:%f, height:%f", box->getwh().x, box->getwh().y);
                ImGui::Text("[Physics] mass: %f", box->getMass());
                ImGui::Text("[Physics] angle: %f", box->getPhysicsData().second );
                ImGui::Text("[Physics] friction: %f", box->getfriction());
                ImGui::Text("[Screen] Center: (%f, %f)", box->getPosition().x, box->getPosition().y);
                ImGui::Text("[Screen] Angle: %f", radiansToDegrees(box->getRotation()));
                ImGui::Separator();
            }
        }

        if (ImGui::CollapsingHeader("Arbiters"))
        {
            ImGui::Text("Arbiters size: %d", world.arbiters.size());
            for(auto arbiter : world.arbiters)
            {
                ImGui::Text("Arbiters: Contact %d", arbiter.second.numContacts);
                ImGui::Separator();
            }
        }

        if (ImGui::CollapsingHeader("Demo")) {
            if (ImGui::Button("Demo1: A single box")) {
                demo1();
            }
            if (ImGui::Button("Demo2: Simple Pendulum")) {
                demo2();
            }
            if (ImGui::Button("Demo3: Varying Friction coefficients")) {
                demo3();
            }
            if (ImGui::Button("Demo4: Randomized Stacking")) {
                demo4();
            }
            if (ImGui::Button("Demo5: Pyramid Stacking")){
                demo5();
            }
            if (ImGui::Button("Clear")) {
                world.Clear();
            }
        }
        ImGui::End();

        if(f_keepSimulate || sf::Keyboard::isKeyPressed(sf::Keyboard::Space))
        {
            world.Step(world.timeStep);
        }

        // clear screen
        window.clear(sf::Color::Black);
        draw_obj(world);
        ImGui::SFML::Render(window);
        window.display();
    }
}