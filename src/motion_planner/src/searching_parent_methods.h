#ifndef SEARCHING_PARENT_METHODS_H
#define SEARCHING_PARENT_METHODS_H
#include "type_variables.hpp"
#include "function_simplified.hpp"
#include "sampling_methods.h"

using namespace std;
using namespace func_simplified;
//using namespace utility_methods;
class searching_parent_methods
{
public:
    /*
     * searching the parent node
     */
    searching_parent_methods();
    bool searching_parent_node();
    /*
     * pointer to function method
     */
    fun_simple** p_func_methods_searching_parent;
    /*
     * pointer to function sampling.
     */
    sampling_methods** p_sampling_methods_searching_parent;
    tree<type_node_point>::iterator parent_node;
private:


};

#endif // SEARCHING_PARENT_METHODS_H
