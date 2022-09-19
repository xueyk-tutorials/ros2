/*
 * @Description: 
 * @Version: 1.0
 * @Author: xueyuankui
 * @Date: 2022-09-14 22:39:08
 * @LastEditors: xueyuankui
 * @LastEditTime: 2022-09-14 22:39:08
 */
#ifndef POLYGON_BASE_REGULAR_POLYGON_HPP
#define POLYGON_BASE_REGULAR_POLYGON_HPP

namespace polygon_base
{
  class RegularPolygon
  {
    public:
      virtual void initialize(double side_length) = 0;
      virtual double area() = 0;
      virtual ~RegularPolygon(){}

    protected:
      RegularPolygon(){}
  };
}  // namespace polygon_base

#endif  // POLYGON_BASE_REGULAR_POLYGON_HPP