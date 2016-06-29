#include <iostream>
#include <vector>
#include <array>
#include <fstream>


#include <boost/config/warning_disable.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/phoenix_fusion.hpp>
#include <boost/spirit/include/phoenix_stl.hpp>
#include <boost/spirit/include/phoenix_object.hpp>
#include <boost/fusion/include/adapt_struct.hpp>

struct vec3
{
    float x, y, z;
};

typedef std::array<vec3, 3> vertices_t;

struct solid
{
    std::string header; //solid name or header
    std::vector<vec3> normals;
    std::vector<vec3> vertices;
    std::vector<std::uint16_t> attributes; //only available in binary files
};
//
BOOST_FUSION_ADAPT_STRUCT(
    vec3,
    (float, x)
    (float, y)
    (float, z)
)
//
BOOST_FUSION_ADAPT_STRUCT(
    solid,
    (std::string, header)
    (std::vector<vec3>, normals)
    (std::vector<vec3>, vertices)
    (std::vector<std::uint16_t>, attributes)
)



template <typename Iterator>
struct stl_ascii_parser : boost::spirit::qi::grammar<Iterator, solid(), boost::spirit::ascii::space_type>
{
   
    stl_ascii_parser() : stl_ascii_parser::base_type(start)
    { 
        namespace qi = boost::spirit::qi;
        namespace ascii = boost::spirit::ascii;
        namespace fusion = boost::fusion;
        namespace px = boost::phoenix;
        using namespace qi::labels;
        using qi::on_error;
        using qi::fail;
        startSolid %= qi::lit("solid") >> qi::lexeme[+(ascii::char_ - ascii::blank)];

        endSolid = qi::lit("endsolid") >> ascii::string(_r1);

        vector %= qi::float_ >> qi::float_ >> qi::float_;

        vertices %= qi::repeat(3)[qi::lit("vertex") >> vector];

        start =
            startSolid    [px::at_c<0>(_val) = _1]
            
            >> *(qi::lit("facet") >> qi::lit("normal") >> vector[px::push_back(px::at_c<1>(_val), _1)]//normal
                //vertices
                >> qi::lit("outer") >> qi::lit("loop")
                >> vertices[px::insert(px::at_c<2>(_val), px::end(px::at_c<2>(_val)),px::begin(_1), px::end(_1))]
                >> qi::lit("endloop")
                )
            >> qi::lit("endfacet")
            //end
            >> endSolid(px::at_c<0>(_val))
            ;

        start.name("start");
        startSolid.name("startSolid");
        vector.name("vec3");
        vertices.name("vertices");
        endSolid.name("end_tag");

        on_error<fail>
            (
                start
                , std::cout
                << px::val("Error! Expecting ")
                << _4                               // what failed?
                << px::val(" here: \"")
                << px::construct<std::string>(_3, _2)   // iterators to error-pos, end
                << px::val("\"")
                << std::endl
                );
    }

    boost::spirit::qi::rule<Iterator, solid(), boost::spirit::ascii::space_type> start;
    boost::spirit::qi::rule<Iterator, std::string(), boost::spirit::ascii::space_type> startSolid;
    boost::spirit::qi::rule<Iterator, vec3(), boost::spirit::ascii::space_type> vector;
    boost::spirit::qi::rule<Iterator, std::vector<vec3>(), boost::spirit::ascii::space_type> vertices;
    boost::spirit::qi::rule<Iterator, void(std::string), boost::spirit::ascii::space_type> endSolid;
};

template <typename Iterator>
struct stl_binary_parser : boost::spirit::qi::grammar<Iterator, solid()>
{

    stl_binary_parser() : stl_binary_parser::base_type(start)
    {
        namespace spirit = boost::spirit;
        namespace qi = boost::spirit::qi;
        namespace ascii = boost::spirit::ascii;
        namespace px = boost::phoenix;
        using namespace qi::labels;

        vector %= spirit::little_bin_float >> spirit::little_bin_float >> spirit::little_bin_float;

        start =
            qi::repeat(80)[spirit::byte_        [px::at_c<0>(_val) += _1]]              //80x8-bit header
            >> spirit::little_dword             [px::reserve(px::at_c<1>(_val), _1), 
                                                 px::reserve(px::at_c<2>(_val), 3 * _1), 
                                                 px::reserve(px::at_c<3>(_val), _1)]    //# of triangles
            >> *(vector                         [px::push_back(px::at_c<1>(_val), _1)]  //normal
                >> spirit::repeat(3)[vector     [px::push_back(px::at_c<2>(_val), _1)]] //vertices
                >> spirit::little_word          [px::push_back(px::at_c<3>(_val), _1)]  //attribute
                )
            //end
            ;
    }

    boost::spirit::qi::rule<Iterator, solid()> start;
    boost::spirit::qi::rule<Iterator, vec3()> vector;
    boost::spirit::qi::rule<Iterator, void(std::string)> endSolid;
};

template <typename Iterator>
struct stl_parser : boost::spirit::qi::grammar<Iterator, solid(), boost::spirit::ascii::space_type>
{
    stl_parser() : stl_parser::base_type(start)
    {
        start %= stl_ascii_ | boost::spirit::qi::lexeme[stl_binary_];
    }
    stl_ascii_parser<Iterator> stl_ascii_;
    stl_binary_parser<Iterator> stl_binary_;
    boost::spirit::qi::rule<Iterator, solid(), boost::spirit::ascii::space_type> start;
};


int main()
{
    std::string parsing = "solid Hi \
        facet normal 0.5 0.6 0.7\
        outer loop\
        vertex 0.5 0.6 0.7\
        vertex 0.5 0.6 0.7\
        vertex 0.5 0.6 0.7\
        endloop\
        endfacet\
        endsolid Hi";

    stl_parser<std::string::const_iterator > b;
    solid output, output2;
    bool r1 = boost::spirit::qi::phrase_parse(parsing.begin(), parsing.end(), b, boost::spirit::ascii::space, output);

    std::ifstream f("C:\\Users\\Matthias\\Desktop\\ship.stl", std::ios::binary);
    std::string str;
    f.seekg(0, std::ios::end);
    str.resize(f.tellg());
    f.seekg(0, std::ios::beg);
    f.read(&str[0], str.size());
    f.close();

    
    
    bool r2 = boost::spirit::qi::phrase_parse(str.begin(),str.end(), b, boost::spirit::ascii::space, output2);
    system("Pause");
}