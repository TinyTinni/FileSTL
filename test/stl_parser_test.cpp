#define BOOST_TEST_MODULE STL_IO

#include <boost/test/unit_test.hpp>
#include <boost/mpl/list.hpp>

#include "stlio.hpp"

using test_types = boost::mpl::list<float, double>;

using namespace tyti;

BOOST_AUTO_TEST_CASE_TEMPLATE(read_ascii_from_string, T, test_types)
{
    using solid = stl::basic_solid<T>;
    std::string str = "solid Hi \
        facet normal 0.5 0.6 0.7\
        outer loop\
        vertex 0.5 0.6 0.7\
        vertex 0.5 0.6 0.7\
        vertex 0.5 0.6 0.7\
        endloop\
        endfacet\
        endsolid Hi";

    auto out = stl::read<T>(str.begin(), str.end());

    BOOST_TEST(out.second);
    solid& s = out.first;
    BOOST_TEST(s.header == "Hi");
    BOOST_TEST(s.normals.size() == 1);
    BOOST_TEST(s.vertices.size() == 3);
    BOOST_TEST(s.attributes.size() == 0);
}

