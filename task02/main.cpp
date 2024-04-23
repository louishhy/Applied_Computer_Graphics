#include <filesystem>
// #include <experimental/filesystem> // uncomment here if the <filesystem> cannot be included above
//
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include "Eigen/Core"
//
#include "parse_svg.h"
// Use <optional> for better readability in quadratic function solving.
#include <optional>
// Define M_PI if it is not defined.
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/***
 * signed area of a triangle connecting points (p0, p1, p2) in counter-clockwise order.
 * @param p0 1st point xy-coordinate
 * @param p1 2nd point xy-coordinate
 * @param p2 3rd point xy-coordinate
 * @return signed area (float)
 */
float area(
    const Eigen::Vector2f &p0,
    const Eigen::Vector2f &p1,
    const Eigen::Vector2f &p2) {
  const auto v01 = p1 - p0;
  const auto v02 = p2 - p0;
  // return 0.5f * (v01[0] * v02[1] - v01[1] * v02[0]); // right handed coordinate
  return 0.5f * (v01[1] * v02[0] - v01[0] * v02[1]); // left-handed coordinate (because pixel y-coordinate is going down)
}


/***
 * compute number of intersection of a ray against a line segment
 * @param org ray origin
 * @param dir ray direction (unit normal)
 * @param ps one of the two end points
 * @param pe the other end point
 * @return number of intersection
 */
int number_of_intersection_ray_against_edge(
    const Eigen::Vector2f &org,
    const Eigen::Vector2f &dir,
    const Eigen::Vector2f &ps,
    const Eigen::Vector2f &pe) {
  auto a = area(org, org + dir, ps);
  auto b = area(org, pe, org + dir);
  auto c = area(org, ps, pe);
  auto d = area(dir+ps, ps, pe);
  if (a * b > 0.f && d * c < 0.f) { return 1; }
  return 0;
  // the following code was a bug
  //auto d = area(org + dir, ps, pe);
  //if (a * b > 0.f && d * c > 0.f && fabs(d) > fabs(c)) { return 1; }
}

/***
 * (Added by student for better readability)
 * A class to store the solution of a quadratic equation.
 * It has two optional fields to store the roots.
*/
class QuadraticSolution{
public:
  std::optional<float> root1;
  std::optional<float> root2;

  QuadraticSolution(){
    root1 = std::nullopt;
    root2 = std::nullopt;
  }

  // Overload the << operator for easy printing.
  friend std::ostream& operator<<(std::ostream &os, const QuadraticSolution &sol){
    os << "Root 1: ";
    if (sol.root1.has_value()){ 
      os << sol.root1.value(); 
    }
    else{ os << "None"; }
    os << ", Root 2: ";
    if (sol.root2.has_value()){ 
      os << sol.root2.value(); 
    }
    else{ os << "None"; }
    return os;
  }
};


/***
 * (Added by student for better readability)
 * Solve the quadratic equation denoted by ax^2 + bx + c = 0
*/
QuadraticSolution solve_quadratic(float a, float b, float c){
  QuadraticSolution sol;
  float discriminant = b * b - 4 * a * c;
  if (discriminant < 0) { return sol; }    // There are no solution.
  sol.root1 = (-b + std::sqrt(discriminant)) / (2 * a);
  sol.root2 = (-b - std::sqrt(discriminant)) / (2 * a);
  return sol;
}

/***
 *
 * @param org ray origin
 * @param dir ray direction (unit vector)
 * @param ps one of the two end points
 * @param pc control point
 * @param pe the other end point
 * @return the number of intersections
 */
int number_of_intersection_ray_against_quadratic_bezier(
    const Eigen::Vector2f &org,
    const Eigen::Vector2f &dir,
    const Eigen::Vector2f &ps,
    const Eigen::Vector2f &pc,
    const Eigen::Vector2f &pe) {
  // comment out below to do the assignment
  // return number_of_intersection_ray_against_edge(org, dir, ps, pe);
  // write some code below to find the intersection between ray and the quadratic
  // 1. Calculate one of the normal vector for dir.
  Eigen::Vector2f normal(-dir[1], dir[0]);
  assert(dir.dot(normal) == 0);
  // 2. Calculate t, since now the equation has nothing to do with s anymore.
  // The Bernstein eqn is given by p(t) = (1-t)^2 ps + t(1 - t) pc + t^2 pe.
  // p(t) is perpendicular to normal. Hence, <p(t), normal> = 0
  // We can then solve the eqn wrt. t.
  // Define the coefficient for the quadratic eqn ax^2 + bx + c = 0
  float a = (ps - 2 * pc + pe).dot(normal);
  float b = 2 * (pc - ps).dot(normal);
  float c = (ps - org).dot(normal);
  // Solve the quadratic eqn.
  QuadraticSolution sol = solve_quadratic(a, b, c);
  // Calculate the roots and intersection.
  float intersection_count = 0;
  // If there are no solutions, return 0.
  if (!sol.root1.has_value() && !sol.root2.has_value()){ return 0; }
  // Get the first root.
  float t_root1 = sol.root1.value();
  Eigen::Vector2f p_t_root1 = std::pow(1 - t_root1, 2) * ps + t_root1 * (1 - t_root1) * pc + std::pow(t_root1, 2) * pe;
  // Solve for s for the first root.
  float s_root1 = (p_t_root1 - org).dot(dir) / dir.squaredNorm();
  if (t_root1 > 0 && t_root1 < 1 && s_root1 > 0){ intersection_count++; }
  // If the second root equals to the first root, early termination. Return the intersection count.
  if (sol.root1 == sol.root2){ return intersection_count; }
  // Get the second root.
  float t_root2 = sol.root2.value();
  Eigen::Vector2f p_t_root2 = std::pow(1 - t_root2, 2) * ps + t_root2 * (1 - t_root2) * pc + std::pow(t_root2, 2) * pe;
  float s_root2 = (p_t_root2 - org).dot(dir) / dir.squaredNorm();
  if (t_root2 > 0 && t_root2 < 1 && s_root2 > 0){ intersection_count++; }
  return intersection_count;
}

/*
int number_of_intersection_ray_against_quadratic_bezier_debug(
    const Eigen::Vector2f &org,
    const Eigen::Vector2f &dir,
    const Eigen::Vector2f &ps,
    const Eigen::Vector2f &pc,
    const Eigen::Vector2f &pe) {
  // comment out below to do the assignment
  // return number_of_intersection_ray_against_edge(org, dir, ps, pe);
  // write some code below to find the intersection between ray and the quadratic
  // 1. Calculate one of the normal vector for dir.
  Eigen::Vector2f normal(-dir[1], dir[0]);
  assert(dir.dot(normal) == 0);
  // 2. Calculate t, since now the equation has nothing to do with s anymore.
  // The Bernstein eqn is given by p(t) = (1-t)^2 ps + t(1 - t) pc + t^2 pe.
  // p(t) is perpendicular to normal. Hence, <p(t), normal> = 0
  // We can then solve the eqn wrt. t.
  // Define the coefficient for the quadratic eqn ax^2 + bx + c = 0
  float a = (ps - 2 * pc + pe).dot(normal);
  float b = 2 * (pc - ps).dot(normal);
  float c = (ps - org).dot(normal);
  // Solve the quadratic eqn.
  QuadraticSolution sol = solve_quadratic(a, b, c);
  // Print
  std::cout << "a: " << a << ", b: " << b << ", c: " << c << std::endl;
  std::cout << sol << std::endl;
  std::cout << std::endl;
  // If there are no solutions, return 0.
  if (!sol.root1.has_value() && !sol.root2.has_value()){ return 0; }
  float t_root1 = sol.root1.value();
  float t_root2 = sol.root2.value();
  // Solve for s for each t_root.
  Eigen::Vector2f p_t_root1 = std::pow(1 - t_root1, 2) * ps + t_root1 * (1 - t_root1) * pc + std::pow(t_root1, 2) * pe;
  Eigen::Vector2f p_t_root2 = std::pow(1 - t_root2, 2) * ps + t_root2 * (1 - t_root2) * pc + std::pow(t_root2, 2) * pe;
  // Print out the vector
  std::cout << "p_t_root1: " << p_t_root1 << std::endl;
  std::cout << "p_t_root2: " << p_t_root2 << std::endl;
  float s_root1 = (p_t_root1 - org).dot(dir) / dir.squaredNorm();
  float s_root2 = (p_t_root2 - org).dot(dir) / dir.squaredNorm();
  // Output the s values
  std::cout << "s_root1: " << s_root1 << ", s_root2: " << s_root2 << std::endl;
  float intersection_count = 0;
  if (t_root1 > 0 && t_root1 < 1 && s_root1 > 0){ intersection_count++; }
  if (t_root2 > 0 && t_root2 < 1 && s_root2 > 0){ intersection_count++; }
  return intersection_count;
}
*/


void debug(){
  std::cout << "Debugging..." << std::endl;
  // Test the quadratic solver.
  QuadraticSolution sol = solve_quadratic(1, 3, -4);
  std::cout << sol << std::endl;
  std::cout << std::endl;
  // Test the intersection of ray against quadratic bezier.
  Eigen::Vector2f org(1, 0.1);
  // Create 30 degree vector for dir
  float degree = 0;
  Eigen::Vector2f dir(std::cos(degree * M_PI / 180), std::sin(degree * M_PI / 180));
  Eigen::Vector2f ps(0, 0);
  Eigen::Vector2f pc(1, 1);
  Eigen::Vector2f pe(2, 0);
  int count = number_of_intersection_ray_against_quadratic_bezier_debug(org, dir, ps, pc, pe);
  std::cout << "Count: " << count << std::endl;
}

int main() {
  debug();
  const auto input_file_path = std::filesystem::path(PROJECT_SOURCE_DIR) / ".." / "asset" / "r.svg";
  const auto [width, height, shape] = acg::svg_get_image_size_and_shape(input_file_path);
  if (width == 0) { // something went wrong in loading the function
    std::cout << "file open failure" << std::endl;
    abort();
  }
  const std::vector<std::string> outline_path = acg::svg_outline_path_from_shape(shape);
  const std::vector<std::vector<acg::Edge>> loops = acg::svg_loops_from_outline_path(outline_path);
  //
  std::vector<unsigned char> img_data(width * height, 255); // grayscale image initialized white
  for (unsigned int ih = 0; ih < height; ++ih) {
    for (unsigned int iw = 0; iw < width; ++iw) {
      const auto org = Eigen::Vector2f(iw + 0.5, ih + 0.5); // pixel center
      const auto dir = Eigen::Vector2f(60., 20.); // search direction
      int count_cross = 0;
      for (const auto &loop: loops) { // loop over loop (letter R have internal/external loops)
        for (const auto &edge: loop) { // loop over edge in the loop
          if (edge.is_bezier) { // in case the edge is a quadratic Bézier
            count_cross += number_of_intersection_ray_against_quadratic_bezier(
                org, dir,
                edge.ps, edge.pc, edge.pe);
          } else { // in case the edge is a line segment
            count_cross += number_of_intersection_ray_against_edge(
                org, dir,
                edge.ps, edge.pe);
          }
        }
      }
      if (count_cross % 2 == 1) { // Jordan's curve theory
        img_data[ih * width + iw] = 0; // paint black if it is inside
      }
    }
  }
  const auto output_file_path = std::filesystem::path(PROJECT_SOURCE_DIR) / "output.png";
  stbi_write_png(output_file_path.string().c_str(), width, height, 1, img_data.data(), width);
}
