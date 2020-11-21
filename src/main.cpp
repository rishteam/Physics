#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>

#include <iostream>
#include "fmt/core.h"
#include "vector_math.h"

#include "Box.h"
#include "Circle.h"
#include "Polygon.h"
#include "Arbiter.h"

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

World world(Vec2(0.0, -9.8), 1280.0f, 780.0f);
sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH,  WINDOW_HEIGHT), "Physics");

// Polygon-Polygon contact Point
void PolygonToPolygonContactPoint(sf::Event event)
{

    std::deque<Vec2> tmp;
    tmp.push_back({-10, 10});
    tmp.push_back({10, 10});
    tmp.push_back({10, -10});
    tmp.push_back({0, -20});
    tmp.push_back({-10, -10});

    Shape *poly = new Polygon(tmp, Vec2(0, 0));
    poly->SetMatrix(0.0f);

    std::deque<Vec2> tmp2;
    tmp2.push_back({-10, 10});
    tmp2.push_back({10, 10});
    tmp2.push_back({10, -10});
    tmp2.push_back({0, -20});
    tmp2.push_back({-10, -10});

    Shape *poly2 = new Polygon(tmp, Vec2(0, 0));

    poly2->setPosition(Vec2(event.mouseMove.x, event.mouseMove.y));

    Arbiter tmpM(poly, poly2);

    // Collide Detection
    if (poly->Collide(&tmpM, *poly2))
    {
        poly->selected = true;
        poly2->selected = true;
    }

    // Draw
    poly->setDebugDraw();
    window.draw(*poly);
    poly2->setDebugDraw();
    window.draw(*poly2);

    for(int i = 0; i < tmpM.contactCounter; i++)
    {
        sf::CircleShape circle;
        circle.setRadius(5);
        circle.setFillColor(sf::Color::Green);
        Vec2 cp = World::ConvertWorldToScreen(Vec2(tmpM.contacts[i].position.x, tmpM.contacts[i].position.y));
        circle.setOrigin(5, 5);
        circle.setPosition(cp.x, cp.y);
        window.draw(circle);
    }
}

// Circle-Polygon contact Point
void CircleToPolygonContactPoint(sf::Event event)
{
    std::deque<Vec2> tmp2;
    tmp2.push_back({-10, 10});
    tmp2.push_back({10, 10});
    tmp2.push_back({10, -10});
    tmp2.push_back({-10, -10});

    Shape *poly = new Polygon(tmp2, Vec2(0, 0));
    poly->SetMatrix(0.6666f);

    Circle *circle = new Circle(10, 10, 10);
    Arbiter tmpM(poly, circle);

    // Update with mouse
    circle->setPosition(Vec2(event.mouseMove.x, event.mouseMove.y));

    // Collide Detection
    if (poly->Collide(&tmpM, *circle))
    {
        poly->selected = true;
        circle->selected = true;
    }

    // Draw
    poly->setDebugDraw();
    window.draw(*poly);
    circle->setDebugDraw();
    window.draw(*circle);

    for(int i = 0; i < tmpM.contactCounter; i++)
    {
        sf::CircleShape circle;
        circle.setRadius(5);
        circle.setFillColor(sf::Color::Green);
        Vec2 cp = World::ConvertWorldToScreen(Vec2(tmpM.contacts[i].position.x, tmpM.contacts[i].position.y));
        circle.setOrigin(5, 5);
        circle.setPosition(cp.x, cp.y);
        window.draw(circle);
    }
}

// Circle-Circle contact Point
void CircleToCircleContactPoints(sf::Event event)
{
    Circle *circle = new Circle(10, 10, 10);
    Circle *circle2 = new Circle(10, 10, 5);
    Arbiter tmpM(circle, circle2);

    // Update with mouse
    circle2->setPosition(Vec2(event.mouseMove.x, event.mouseMove.y));

    // Collide Detection
    if (circle2->Collide(&tmpM, *circle))
    {
        circle->selected = true;
        circle2->selected = true;
    }

    // Draw
    circle->setDebugDraw();
    window.draw(*circle);
    circle2->setDebugDraw();
    window.draw(*circle2);

    for(int i = 0; i < tmpM.contactCounter; i++)
    {
        sf::CircleShape circle;
        circle.setRadius(5);
        circle.setFillColor(sf::Color::Green);
        Vec2 cp = World::ConvertWorldToScreen(Vec2(tmpM.contacts[i].position.x, tmpM.contacts[i].position.y));
        circle.setOrigin(5, 5);
        circle.setPosition(cp.x, cp.y);
        window.draw(circle);
    }
};

void BoxToBox(sf::Event event)
{
    Shape *box = new Box(0, 0, 10, 10);
    box->SetMatrix(0.6666f);

    Shape *box2 = new Box(0, 0, 10, 10);

    // Update with mouse
    box2->setPosition(Vec2(event.mouseMove.x, event.mouseMove.y));

    Arbiter tmpM(box, box2);

    // Collide Detection
    if (box->Collide(&tmpM, *box2))
    {
        box->selected = true;
        box2->selected = true;
    }

    // Draw
    box->setDebugDraw();
    window.draw(*box);
    box2->setDebugDraw();
    window.draw(*box2);

    for(int i = 0; i < tmpM.contactCounter; i++)
    {
        sf::CircleShape circle;
        circle.setRadius(5);
        circle.setFillColor(sf::Color::Green);
        Vec2 cp = World::ConvertWorldToScreen(Vec2(tmpM.contacts[i].position.x, tmpM.contacts[i].position.y));
        circle.setOrigin(5, 5);
        circle.setPosition(cp.x, cp.y);
        window.draw(circle);
    }
}


void PhysicsDemo1()
{
    world.Clear();
    std::deque<Vec2> tmp;
    tmp.push_back({-5, 5});
    tmp.push_back({5, 5});
    tmp.push_back({5, -5});
    tmp.push_back({-5, -5});

    Shape *poly = new Polygon(tmp, Vec2(0, 0));
    poly->SetMatrix(0.6666f);

    std::deque<Vec2> tmp2;
    tmp2.push_back({-30, 10});
    tmp2.push_back({30, 10});
    tmp2.push_back({30, -10});
    tmp2.push_back({-30, -10});

    Shape *poly2 = new Polygon(tmp2, Vec2(0, -20));
    poly2->mass = FLT_MAX;

    world.Add(poly);
    world.Add(poly2);
}

void drawObject()
{
    for(auto &bd : world.bodies)
    {
        bd->setDebugDraw();
        window.draw(*bd);
    }
}

int main()
{
    window.setFramerateLimit(60);
    ImGui::SFML::Init(window);
    sf::Clock deltaClock;

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

            window.clear(sf::Color::Black);
        }

        ImGui::SFML::Update(window, deltaClock.restart());
        ImGui::Begin("Contact Points Detection");
        if (ImGui::CollapsingHeader("Cases")) {
            if(ImGui::Button("Simple Demo"))
            {
                PhysicsDemo1();
            }
//            if (ImGui::Button("Case1: Circle To Polygon")) {
//                CircleToPolygonContactPoint(event);
//            }
//            if (ImGui::Button("Case2: Polygon To Polygon")) {
//                PolygonToPolygonContactPoint(event);
//            }
//            if (ImGui::Button("Demo3: Circle To Circle")) {
//                CircleToCircleContactPoints(event);
//            }
//            if (ImGui::Button("Demo4 : Box To Box")){
//                BoxToBox(event);
//            }

        }
        if (ImGui::CollapsingHeader("Box's Data"))
        {
            for(int i = 0; i < world.bodies.size(); i++)
            {
                ImGui::Text("Object %d:", i);
                ImGui::Text("[Physics] Center: (%f, %f)", world.bodies[i]->position.x, world.bodies[i]->position.y);
                ImGui::Text("[Physics] angle: %f", radiansToDegrees(world.bodies[i]->angle) );
                ImGui::Text("[Physics] mass: %f", world.bodies[i]->mass);
                ImGui::Text("[Physics] invMass: %f", world.bodies[i]->invMass);
                ImGui::Text("[Physics] I: %f", world.bodies[i]->I);
                ImGui::Text("[Physics] invI: %f", world.bodies[i]->invI);
                ImGui::Text("[Physics] Friction: %f", world.bodies[i]->friction);
                ImGui::Separator();
            }
        }

        if (ImGui::CollapsingHeader("Arbiters"))
        {
            ImGui::Text("Arbiters size: %d", world.arbiters.size());
            for(auto arbiter : world.arbiters)
            {
                ImGui::Text("Arbiters: Contact %d", arbiter.second.contactCounter);
                ImGui::Separator();
            }
        }
        ImGui::End();

        if(sf::Keyboard::isKeyPressed(sf::Keyboard::Space))
        {
            world.Step(world.timeStep);
        }
        window.clear(sf::Color::Black);
        drawObject();
        ImGui::SFML::Render(window);
        window.display();
    }
}