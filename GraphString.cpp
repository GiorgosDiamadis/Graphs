

#include "Graph.hpp"
#include "GraphUI.hpp"

using namespace std;

class String
{
  string str;

public:
  String()
  {
    str = string();
  }
  String(istream &in)
  {
    in >> str;
  }

  String(const String &s)
  {
    str = s.str;
  }

  String(const char *c)
  {
    string st(c);
    str = st;
  }
  string &get()
  {
    return str;
  }
  friend ostream &operator<<(ostream &out, const String &mystr)
  {
    out << mystr.str;
    return out;
  }
  bool operator==(const String &other) const
  {
    return str == other.str;
  }
};

int main()
{
  return graphUI<String>();
}
