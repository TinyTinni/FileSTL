#define BOOST_TEST_MODULE STL_IO

#include <boost/test/included/unit_test.hpp>
#include <boost/mpl/list.hpp>

#include <sstream>

#include "../stlio.hpp"

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
    const std::string test_data{  SOURCE_DIR "/testdata/cube_ascii.stl" };

    auto out = stl::read<T>(test_data);
    BOOST_TEST(out.second);
    stl::basic_solid<T>& s = out.first;

    BOOST_TEST(s.normals.size() == 12);
    BOOST_TEST(s.vertices.size() == 3*12);
    BOOST_TEST(s.attributes.size() == 0);
}

BOOST_AUTO_TEST_CASE_TEMPLATE(read_binary_from_file, T, test_types)
{
    const std::string test_data{  SOURCE_DIR "/testdata/cube_binary.stl" };

    auto out = stl::read<T>(test_data);
    BOOST_TEST(out.second);
    stl::basic_solid<T>& s = out.first;

    BOOST_TEST(s.normals.size() == 12);
    BOOST_TEST(s.vertices.size() == 3 * 12);
    BOOST_TEST(s.attributes.size() == 12);
}

template<typename T>
void simple_read_test(bool binary)
{
    using solid = stl::basic_solid<T>;
    using vec3 = stl::basic_vec3<T>;

    T n = T(1.0);
    vec3 vec_n(n, n, n);

    solid to_write;
    to_write.header = "to_write_header";
    to_write.vertices.push_back(vec_n);
    to_write.vertices.push_back(vec_n);
    to_write.vertices.push_back(vec_n);
    to_write.normals.push_back(vec_n);

    std::stringstream sstream;
    stl::write(sstream, to_write, binary);

     std::string str_w = sstream.str();
     auto out = stl::read<T>(str_w.begin(), str_w.end());
     BOOST_TEST(out.second);

     solid cmp = out.first;
     BOOST_TEST(to_write.header == cmp.header);
     BOOST_TEST(to_write.vertices.size() == cmp.vertices.size());
     BOOST_TEST(to_write.normals.size() == cmp.normals.size());
}

BOOST_AUTO_TEST_CASE_TEMPLATE(write_binary, T, test_types)
{
    simple_read_test<T>(true);
}

BOOST_AUTO_TEST_CASE_TEMPLATE(write_ascii, T, test_types)
{
    simple_read_test<T>(false);
}