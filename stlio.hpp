//The MIT License(MIT)
//
//Copyright(c) 2016 Matthias M�ller
//
//Permission is hereby granted, free of charge, to any person obtaining a copy
//of this software and associated documentation files(the "Software"), to deal
//in the Software without restriction, including without limitation the rights
//to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
//copies of the Software, and to permit persons to whom the Software is
//furnished to do so, subject to the following conditions :
//
//The above copyright notice and this permission notice shall be included in all
//copies or substantial portions of the Software.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//SOFTWARE.


#pragma once

#include <vector>
#include <type_traits>
#include <utility>
#include <fstream>

#include <boost/config/warning_disable.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/phoenix_fusion.hpp>
#include <boost/spirit/include/phoenix_stl.hpp>
#include <boost/spirit/include/phoenix_object.hpp>
#include <boost/fusion/include/adapt_struct.hpp>

namespace tyti {
    namespace stl {

        ////////////////////////////////////////////////////////////////////////////////
        // vector
        template<typename T>
        struct basic_vec3
        {
            using scalar_type = T;
            static_assert(std::is_floating_point<scalar_type>::value, "T must be a floating point");
            T data[3];
            basic_vec3():data{T(0),T(0),T(0)}{}
            basic_vec3(T x, T y, T z):data{x,y,z}{}
            
            T operator[](size_t i)const {return data[i];}
            const T &operator[](size_t i) { return data[i]; }
        };

        using dvec3 = basic_vec3<double>;
        using vec3 = basic_vec3<float>;

        //

        ////////////////////////////////////////////////////////////////////////////////
        // solid type
        template<typename T>
        struct basic_solid
        {
            using vec3 = basic_vec3<T>;
            using scalar_type = T;
            std::string header; //solid name or header
            std::vector<vec3> normals;
            std::vector<vec3> vertices;
            std::vector<std::uint16_t> attributes; //only available in binary files
        };

        using dsolid = basic_solid<double>;
        using solid = basic_solid<float>;

        ////////////////////////////////////////////////////////////////////////////////
        // Parsers
        template <typename T, typename Iterator>
        struct stl_parser_ascii : boost::spirit::qi::grammar<Iterator, basic_solid<T>(), boost::spirit::ascii::space_type>
        {

            stl_parser_ascii() : stl_parser_ascii::base_type(start)
            {
                namespace qi = boost::spirit::qi;
                namespace ql = qi::labels;
                namespace ascii = boost::spirit::ascii;
                namespace fusion = boost::fusion;
                namespace px = boost::phoenix;

                startSolid %= qi::lexeme[qi::lit("solid ") >> (+(ascii::char_ - qi::eol) | qi::attr(""))]; //either return header or empty string if no header was found 

                endSolid = qi::lexeme[qi::lit("endsolid ") >> ascii::string(ql::_r1)];

                vector %= qi::float_ >> qi::float_ >> qi::float_;

                start =
                    startSolid[px::at_c<0>(ql::_val) = ql::_1]

                    >> *(qi::lit("facet") >> qi::lit("normal") >> vector[px::push_back(px::at_c<1>(ql::_val), ql::_1)]//normal
                        //vertices
                        >> qi::lit("outer loop")
                        >> qi::repeat(3)[qi::lit("vertex") >> vector[px::push_back(px::at_c<2>(ql::_val), ql::_1)]]
                        >> qi::lit("endloop")
                        >> qi::lit("endfacet")
                        )
                    //end
                    >> endSolid(px::at_c<0>(ql::_val))
                    ;
            }

            boost::spirit::qi::rule<Iterator, basic_solid<T>(), boost::spirit::ascii::space_type> start;
            boost::spirit::qi::rule<Iterator, std::string(), boost::spirit::ascii::space_type> startSolid;
            boost::spirit::qi::rule<Iterator, basic_vec3<T>(), boost::spirit::ascii::space_type> vector;
            boost::spirit::qi::rule<Iterator, void(std::string), boost::spirit::ascii::space_type> endSolid;
        };

        template <typename T, typename Iterator>
        struct stl_parser_binary : boost::spirit::qi::grammar<Iterator, basic_solid<T>()>
        {

            stl_parser_binary() : stl_parser_binary::base_type(start)
            {
                namespace spirit = boost::spirit;
                namespace qi = boost::spirit::qi;
                namespace ql = qi::labels;
                namespace ascii = boost::spirit::ascii;
                namespace px = boost::phoenix;

                vector %= spirit::little_bin_float >> spirit::little_bin_float >> spirit::little_bin_float;

                start =
                    qi::repeat(80)[spirit::byte_[px::at_c<0>(ql::_val) += ql::_1]]              //80x8-bit header
                    >> spirit::little_dword[px::reserve(px::at_c<1>(ql::_val), ql::_1),
                    px::reserve(px::at_c<2>(ql::_val), 3 * ql::_1),
                    px::reserve(px::at_c<3>(ql::_val), ql::_1)]    //# of triangles
                    >> *(vector[px::push_back(px::at_c<1>(ql::_val), ql::_1)]  //normal
                        >> spirit::repeat(3)[vector[px::push_back(px::at_c<2>(ql::_val), ql::_1)]] //vertices
                        >> spirit::little_word[px::push_back(px::at_c<3>(ql::_val), ql::_1)]  //attribute
                        )
                    //end
                    ;
            }

            boost::spirit::qi::rule<Iterator, basic_solid<T>()> start;
            boost::spirit::qi::rule<Iterator, basic_vec3<T>()> vector;
            boost::spirit::qi::rule<Iterator, void(std::string)> endSolid;
        };

        template <typename T, typename Iterator>
        struct stl_parser : boost::spirit::qi::grammar<Iterator, basic_solid<T>(), boost::spirit::ascii::space_type>
        {
            stl_parser() : stl_parser::base_type(start)
            {
                start %= stl_ascii_ | boost::spirit::qi::lexeme[stl_binary_];
            }
            stl_parser_ascii<T, Iterator> stl_ascii_;
            stl_parser_binary<T, Iterator> stl_binary_;
            boost::spirit::qi::rule<Iterator, basic_solid<T>(), boost::spirit::ascii::space_type> start;
        };

        ////////////////////////////////////////////////////////////////////////////////
        // Interface functions
        ////////////////////////////////////////////////////////////////////////////////


        ////////////////////////////////////////////////////////////////////////////////
        // Readers
        template<typename T, typename IterT>
        inline bool read(basic_solid<T>& out, IterT begin, IterT end)
        {
            stl_parser< T, IterT > p;
            return boost::spirit::qi::phrase_parse(begin, end, p, boost::spirit::ascii::space, out);
        }

        template<typename T, typename IterT>
        inline std::pair< basic_solid<T>, bool > read(IterT begin, IterT end)
        {
            basic_solid<T> o;
            bool r = read(o, begin, end);
            return std::make_pair<basic_solid<T>, bool>(std::move(o), std::move(r));
        }

        template<typename IterT>
        inline std::pair< solid, bool > read(IterT begin, IterT end)
        {
            return read<float>(begin, end);
        }

        ////////////////////////////////////////////////////////////////////////////////

        template<typename T>
        inline bool read(basic_solid<T>& out, const std::string& fileName)
        {
            std::ifstream f(fileName, std::ios::binary);
            std::string str;
            f.seekg(0, std::ios::end);
            str.resize((unsigned)f.tellg());
            f.seekg(0, std::ios::beg);
            f.read(&str[0], str.size());
            f.close();

            return read<T>(out, str.cbegin(), str.cend());
        }

        template<typename T>
        inline std::pair<basic_solid<T>, bool> read(const std::string& fileName)
        {
            basic_solid<T> out;
            bool r = read(out, fileName);
            return std::make_pair<basic_solid<T>, bool>(std::move(out), std::move(r));
        }

        inline std::pair<solid, bool> read(const std::string& fileName)
        {
            return read<float>(fileName);
        }

        ////////////////////////////////////////////////////////////////////////////////
        // Writers

        template<typename T, typename streamT>
        void write_ascii(streamT& out, const basic_solid<T>&s)
        {
            out << "solid " << s.header << "\n";
            
            const size_t num_vertices = std::min(s.normals.size(), s.vertices.size()/3);

            for (size_t i = 0; i < num_vertices; ++i)
            {
                out << "\tfacet normal "
                    << s.normals[i][0] << " "
                    << s.normals[i][1] << " "
                    << s.normals[i][2] << " "
                    << "\n";

                out << "\t\touter loop\n";
                for (size_t j = 0; j < 3; ++j)
                {
                    out << "\t\t\tvertex "
                        << s.vertices[3 * i + j][0] << " "
                        << s.vertices[3 * i + j][1] << " "
                        << s.vertices[3 * i + j][2] << " "
                        << "\n";
                }
                out << "\t\tendloop\n"
                    << "\tendfacet\n";
            }
            out << "endsolid " << s.header << "\n";
        }

        namespace detail 
        {
            template<typename streamT>
            inline void write_vectorF32(streamT& out, const double* p)
            {
                float f = static_cast<float>(p[0]);
                out.write(reinterpret_cast<const char *>(p), sizeof(float));
                f = static_cast<float>(p[1]);
                out.write(reinterpret_cast<const char *>(p), sizeof(float));
                f = static_cast<float>(p[2]);
                out.write(reinterpret_cast<const char *>(p), sizeof(float));                
            }
            
            template<typename streamT>
            inline void write_vectorF32(streamT& out, const float* p)
            {
                out.write(reinterpret_cast<const char*>(p), sizeof(float) * 3);
            }
        }

        template<typename T, typename streamT>
        void write_binary(streamT& out, const basic_solid<T>& s)
        {
            // fill header buffer
            const size_t HEADER_SIZE = 80;
            char header_buffer[HEADER_SIZE];
            const size_t to_write = std::min(s.header.size(), HEADER_SIZE);
            std::copy(s.header.begin(), s.header.end(), header_buffer);
            std::fill(header_buffer+to_write, header_buffer+HEADER_SIZE, char(0));
            
            out.write(header_buffer, 80);

            const size_t num_triangles{ std::min(s.normals.size(), s.vertices.size()/3) };

            out.write(reinterpret_cast<const char*>(&num_triangles), 4);

            for (size_t i = 0; i < num_triangles; ++i)
            {
                //we cannot do direct writ
                detail::write_vectorF32(out, s.normals[i].data);
                detail::write_vectorF32(out, s.vertices[3 * i + 0].data);
                detail::write_vectorF32(out, s.vertices[3 * i + 1].data);
                detail::write_vectorF32(out, s.vertices[3 * i + 2].data);
                if (i < s.attributes.size())
                    out.write(reinterpret_cast<const char*>(&s.attributes[i]), sizeof(uint16_t));
            }
        }

        template<typename T, typename streamT>
        void write(streamT& out, const basic_solid<T>& s, bool binary)
        {

            if (binary)
                write_binary(out, s);
            else
                write_ascii(out, s);
        }

    }
}//end namespaces


BOOST_FUSION_ADAPT_TPL_STRUCT(
(T),
(tyti::stl::basic_solid)(T),
(std::string, header),
(std::vector< tyti::stl::basic_vec3<T> >, normals),
(std::vector< tyti::stl::basic_vec3<T> >, vertices),
(std::vector<std::uint16_t>, attributes)
);

BOOST_FUSION_ADAPT_TPL_STRUCT(
(T),
(tyti::stl::basic_vec3)(T),
(T, data[0])
(T, data[1])
(T, data[2])
);
