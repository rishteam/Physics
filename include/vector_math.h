#pragma once
#include <iostream>
#include <limits>
#include <cmath>
#include <deque>

#include <fmt/core.h>
#include <SFML/Graphics.hpp>

/**
 * @file vector_math.h
 * @author  halloworld <william31212@gmail.com>
 * @brief 更詳細的向量數學計算function，繼承sf::Vector2<float>
 */

/** @brief 徑度與度度量的轉換 */
// #define M_PI 3.14159265358979323846 2643383279 5028841971 6939937510 5820974944 5923078164 0628620899 8628034825 3421170679 8214808651 3282306647 0938446095 5058223172 5359408128 4811174502 8410270193 8521105559 6446229489 5493038196 4428810975 6659334461 2847564823 3786783165 2712019091 4564856692 3460348610 4543266482 1339360726 0249141273 7245870066 0631558817 4881520920 9628292540 9171536436 7892590360 0113305305 4882046652 1384146951 9415116094 3305727036 5759591953 0921861173 8193261179 3105118548 0744623799 6274956735 1885752724 8912279381 8301194912 9833673362 4406566430 8602139494 6395224737 1907021798 6094370277 0539217176 2931767523 8467481846 7669405132 0005681271 4526356082 7785771342 7577896091 7363717872 1468440901 2249534301 4654958537 1050792279 6892589235 4201995611 2129021960 8640344181 5981362977 4771309960 5187072113 4999999837 2978049951 0597317328 1609631859 5024459455 3469083026 4252230825 3344685035 2619311881 7101000313 7838752886 5875332083 8142061717 7669147303 5982534904 2875546873 1159562863 8823537875 9375195778 1857780532
// 1712268066 1300192787 6611195909 2164201989
#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)

/** @brief float_max, float_min常數 */
#define MAX_float std::numeric_limits<float>::max()
#define MIN_float std::numeric_limits<float>::min()

    /**
 * @brief 補足sf::Vector2f無法做的計算，自行建立一個特化的Vector
 * @details 包括向量長度、內積、左右法向量等數學基本工具
 */
    class Vector : public sf::Vector2<float> {
public:
    /**
     * @brief 向量x, y
     */
    Vector(float x, float y);
    /**
     * @brief Deconstructer
     */
    ~Vector() = default;
    /**
     * @brief 印出向量
     */
    void print_Vector();
    /**
     * @brief 獲得向量長度
     */
    float getLength();
    /**
     * @brief 計算兩向量內積
     * @param vec2 另一個向量
     * @retval float 內積結果
     */
    float dot(Vector &vec2);
    /**
     * @brief 計算vec在vec2上的正射影(投影長)
     * @param vec2 另一個向量
     * @retval float 投影長
     */
    float projectLengthOnto(Vector &vec2);
    /**
     * @brief 計左法向量
     */
    Vector normalL();
    /**
     * @brief 右法向量
     */
    Vector normalR();
    /**
     * @brief 旋轉
     * @param angle 旋轉角度(徑度)
     * @retval float 旋轉過後的向量
     */
    void rotate(float angle);
    //x′1=[(x1−x0)cosθ−(y1−y0)sinθ]+x0
    //y′1=[(y1−y0)cosθ+(x1−x0)sinθ]+y0
    /**
     * @brief 旋轉
     * @details 依照參考點旋轉該向量
     * @param ref 參考點
     * @param angle 旋轉角度(徑度)
     * @retval float 旋轉過後的向量
     */
    void rotate_ref(float angle, Vector &ref);
};

std::pair<float, float> getMinMax(Vector &axis, std::deque<Vector> corner);