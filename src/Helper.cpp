#include "Helper.h"

#include <random>

extern sf::RenderWindow window;
extern sf::Font font;

int randInt(int lower, int upper)
{
    std::random_device rd;
    std::mt19937 generator(rd());
    std::uniform_int_distribution<int> rander(lower, upper);
    auto res = rander(generator);
    return res;
}

sf::Text & drawText(const std::string &s, const sf::Vector2f &pos, uint32_t size, const sf::Color &color, uint32_t style,
                    const sf::Vector2f &origin)
{
    static sf::Text text;
    text.setFont(font);
    text.setPosition(pos);
    text.setOrigin(origin);
    text.setString(s);
    text.setCharacterSize(size);
    text.setFillColor(color);
    text.setStyle(style);
    window.draw(text);
    return text;
}

// drawCircle({100, 200}, 20, sf::Color::Red);
sf::CircleShape & drawCircle(const sf::Vector2f &pos, uint32_t radius, const sf::Color &fill, float thickness, const sf::Color &outline, uint32_t mode)
{
    static sf::CircleShape cir;
    cir.setPosition(pos);
    cir.setRadius(radius);
    cir.setFillColor(fill);
    cir.setOutlineColor(outline);
    cir.setOutlineThickness(thickness);
    if(mode == CirCenter)
        cir.setOrigin(0.5f*radius, 0.5f*radius);
    window.draw(cir);
    return cir;
}

sf::RectangleShape& drawRect(const sf::Vector2f &pos, const sf::Vector2f &size, const sf::Color &fill, float thickness, const sf::Color &outline, uint32_t mode)
{
    static sf::RectangleShape rect;
    rect.setPosition(pos);
    rect.setSize(size);
    rect.setFillColor(fill);
    rect.setOutlineThickness(thickness);
    rect.setOutlineColor(outline);
    if(mode == CirCenter)
        rect.setOrigin(0.5f*size.x, 0.5f*size.y);
    window.draw(rect);
    return rect;
}