#ifndef WATERLINKED_DVL_UTILS_HPP
#define WATERLINKED_DVL_UTILS_HPP

#include "boost/array.hpp"
#include "vector"
#include "boost/range/algorithm/copy.hpp"

template<size_t Size, class Container>
boost::array<typename Container::value_type, Size> as_array(const Container &cont)
{
    assert(cont.size() == Size);
    boost::array<typename Container::value_type, Size> result;
    boost::range::copy(cont, result.begin());
    return result;
}

#endif //WATERLINKED_DVL_UTILS_HPP
