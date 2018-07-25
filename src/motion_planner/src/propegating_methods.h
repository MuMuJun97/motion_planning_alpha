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
    bool curve_propegation(tree<type_node_point>::iterator parent_node, type_road_point sample_node,type_node_point& new_node);
    /*
     * pointer to propegating method.
     */
    fun_simple** p_func_method_propegating;
private:
    /*
     * paremeters for clothoid curve.
     */
    //double k,dk,L;
    /*
     * engine for the random sample.
     */
    std::random_device ran_device;

};

#endif // PROPEGATING_METHODS_H
