#pragma once

#include <cstdint>
#include <SFML/Graphics.hpp>

extern sf::Font font;;
extern sf::RenderWindow window;

int randInt(int lower, int upper);

/**
 * @brief Draw texts
 * @param s A string of text
 * @param pos Position
 * @param size Font size
 * @param color Text color
 * @param style Text style
 * @param origin Origin
 */
sf::Text & drawText(const std::string &s, const sf::Vector2f &pos, uint32_t size, const sf::Color &color,
              uint32_t style = sf::Text::Regular, const sf::Vector2f &origin=sf::Vector2f(0, 0));

enum CircleMode
{
    CirLeftUp,
    CirCenter
};
/**
 * @brief Draw a circle
 * @param pos Position (x, y)
 * @param radius Radius of the circle
 * @param fill Fill Color
 * @param thickness Outline thickness
 * @param outline Outline color
 * @param mode Center Mode
 */
sf::CircleShape & drawCircle(const sf::Vector2f &pos, uint32_t radius, const sf::Color &fill, float thickness=0.f, const sf::Color &outline=sf::Color::Red, uint32_t mode=CirCenter);

enum RectMode
{
    RectLeftUp,
    RectCenter
};
/**
 * @brief Draw a rect
 * @details Usage:
 * @code
 * drawRect({100, 200}, {50, 50}, sf::Color::Red);
 * @endcode
 * @param pos Position (x, y)
 * @param size Size (width, height)
 * @param fill Fill Color
 * @param thickness Outline thickness
 * @param outline Outline color
 * @param mode Center Mode
 */
sf::RectangleShape& drawRect(const sf::Vector2f &pos, const sf::Vector2f &size, const sf::Color &fill, float thickness=0.f, const sf::Color &outline=sf::Color::Red, uint32_t mode=RectLeftUp);
