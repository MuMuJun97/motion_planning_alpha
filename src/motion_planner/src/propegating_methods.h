#ifndef PROPEGATING_METHODS_H
#define PROPEGATING_METHODS_H
#include "function_simplified.hpp"

using namespace func_simplified;
using namespace Clothoid;
//using namespace utility_methods;
class propegating_methods
{
public:
    propegating_methods();
    ~propegating_methods();
public:
    /*
     * propegate with curve
     * /
     */
    bool curve_propegation(
        tree<type_node_point>::iterator parent_node, 
        type_road_point sample_node, 
        std::vector<type_node_point>& new_nodes);
    /*
     * pointer to propegating method.
     */
    fun_simple** p_func_method_propegating;
private:
    /*
     * engine for the random sample.
     */
    std::random_device ran_device;
public:
    /*
     * paremeters for clothoid curve.
     */
    static constexpr double NODES_DENSITY = 0.25;


};

#endif // PROPEGATING_METHODS_H
