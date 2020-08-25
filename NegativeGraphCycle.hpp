
#ifndef NEGATIVE_GRAPH_CYCLE_HPP_
#define NEGATIVE_GRAPH_CYCLE_HPP_

#include <exception>

class NegativeGraphCycle : public std::exception
{
public:
  const char *what() const noexcept;
};

const char *NegativeGraphCycle::what() const noexcept
{
  return "Negative Graph Cycle!";
}

#endif
