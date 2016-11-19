#define BOOST_TEST_MODULE STL_IO

#include <boost/test/unit_test.hpp>
#include <boost/mpl/list.hpp>

#include "stlio.hpp"

using test_types = boost::mpl::list<float, double>;

using namespace tyti;

BOOST_AUTO_TEST_CASE_TEMPLATE(read_ascii_from_string, T, test_types)
{
    using solid = stl::basic_solid<T>;
    std::string str = "solid Hi\n \
        facet normal 0.5 0.6 0.7\n\
        outer loop\n\
        vertex 0.5 0.6 0.7\n\
        vertex 0.5 0.6 0.7\n\
        vertex 0.5 0.6 0.7\n\
        endloop\n\
        endfacet\n\
        endsolid Hi\n";

    auto out = stl::read<T>(str.begin(), str.end());

    BOOST_TEST(out.second);
    solid& s = out.first;
    BOOST_TEST(s.header == "Hi");
    BOOST_TEST(s.normals.size() == 1);
    BOOST_TEST(s.vertices.size() == 3);
    BOOST_TEST(s.attributes.size() == 0);
}

BOOST_AUTO_TEST_CASE_TEMPLATE(read_ascii_from_file, T, test_types)
{
    const std::string test_data{ std::string{ SOURCE_DIR } +"/testdata/cube_ascii.stl" };

    auto out = stl::read<T>(test_data);
    BOOST_TEST(out.second);
    stl::basic_solid<T>& s = out.first;

    BOOST_TEST(s.normals.size() == 12);
    BOOST_TEST(s.vertices.size() == 3*12);
    BOOST_TEST(s.attributes.size() == 0);
}

BOOST_AUTO_TEST_CASE_TEMPLATE(read_binary_from_file, T, test_types)
{
    const std::string test_data{ std::string{ SOURCE_DIR } +"/testdata/cube_binary.stl" };

    auto out = stl::read<T>(test_data);
    BOOST_TEST(out.second);
    stl::basic_solid<T>& s = out.first;

    BOOST_TEST(s.normals.size() == 12);
    BOOST_TEST(s.vertices.size() == 3 * 12);
    BOOST_TEST(s.attributes.size() == 12);
}